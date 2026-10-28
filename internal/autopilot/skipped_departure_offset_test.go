package autopilot

import (
	_ "embed"
	"encoding/json"
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/skipped-shoulder-jam.json
var skippedShoulderFixture []byte

func TestSavedSkippedShoulderDepartureRecovers(t *testing.T) {
	var saved struct {
		Bus    sim.Bus
		Driver DriverSnapshot
		Route  sim.Route
	}
	if err := json.Unmarshal(skippedShoulderFixture, &saved); err != nil {
		t.Fatal(err)
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	ad, err := saved.Driver.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	b := &saved.Bus
	start := b.Pos
	restored := false
	for tick := 0; tick < 1800; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, &saved.Route, w, nil, nil, 52, 1./60)
		before := b.Pos
		b.Update(1./60, th, br, st, hb, doors)
		if b.Pos.Distance(before) > b.MaxSpeedFwd/60+.001 {
			t.Fatal("bus jumped")
		}
		if boundary := b.CheckBoundaries(w, 52); boundary.Zone == sim.ZoneHardBarrier {
			t.Fatalf("boundary tick%d %+v", tick, boundary)
		}
		if tick == 30 {
			data, e := json.Marshal(ad.Snapshot())
			if e != nil {
				t.Fatal(e)
			}
			var snap DriverSnapshot
			if e = json.Unmarshal(data, &snap); e != nil {
				t.Fatal(e)
			}
			ad, e = snap.Restore(w)
			if e != nil {
				t.Fatal(e)
			}
			restored = true
		}
		if restored && b.Pos.Distance(start) > 600 {
			return
		}
	}
	t.Fatalf("saved skip remained stopped: %.1fpx offset%.2f", b.Pos.Distance(start), ad.LaneOffset)
}

func TestSkippedDepartureRebasesUnavailableWorldLane(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for model := sim.BusStandard; model < sim.BusModelCount; model++ {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/model%d", lw, h, model), func(t *testing.T) {
					w, u, r := serviceTestWorld(h, lw)
					stop := w.Stops[1]
					p := stop.Pos.Sub(u.Mul(260))
					b := sim.NewBus(p.X, p.Y, h)
					b.SetModel(model)
					ad := NewAutopilotDriver()
					ad.BerserkMode = true // no previous leg/world yet
					ad.ContinuePastSkippedStop(b, stop, lw)
					route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 2}
					th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, nil, lw, 1./60)
					want := p.Sub(ad.drivePath[0]).Dot(r)
					if math.Abs(ad.LaneOffset-want) > .001 {
						t.Fatalf("wrong road reference: %.2f want%.2f", ad.LaneOffset, want)
					}
					b.Update(1./60, th, br, st, hb, doors)
					start := b.Pos
					for tick := 0; tick < 2400; tick++ {
						th, br, st, hb, doors, _ = ad.ComputeControls(b, route, w, nil, nil, lw, 1./60)
						b.Update(1./60, th, br, st, hb, doors)
						if boundary := b.CheckBoundaries(w, lw); boundary.Zone == sim.ZoneHardBarrier {
							t.Fatalf("boundary tick%d %+v", tick, boundary)
						}
						if b.Pos.Sub(start).Dot(u) > 600 {
							return
						}
					}
					t.Fatalf("skip failed to advance: %.1f", b.Pos.Distance(start))
				})
			}
		}
	}
}

func TestSkippedDepartureRetainsValidPassingOffset(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		w, u, r := serviceTestWorld(h, 52)
		stop := w.Stops[1]
		for _, offset := range []float64{0, -52} {
			p := stopRoadPoint(stop, 52, w).Sub(u.Mul(100)).Add(r.Mul(offset))
			b := sim.NewBus(p.X, p.Y, h)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.pathWorld = w
			ad.ContinuePastSkippedStop(b, stop, 52)
			before := ad.LaneOffset
			state := ad.OvertakeState
			ad.PrepareLeg(w, stop, w.Stops[2], 52)
			ad.OvertakeTimer = 2
			if !ad.holdSkippedDepartureLane(b, w, 52, 1./60, 0) {
				t.Fatal("holding valid passage was lost")
			}
			if math.Float64bits(ad.LaneOffset) != math.Float64bits(before) || ad.OvertakeState != state || ad.OvertakeTimer != 2 {
				t.Fatal("valid road passage state was changed")
			}
		}
	}
}

func TestSavedSkippedShoulderRecoveryRetainsFrontTraffic(t *testing.T) {
	var saved struct {
		Bus    sim.Bus
		Driver DriverSnapshot
		Route  sim.Route
	}
	if err := json.Unmarshal(skippedShoulderFixture, &saved); err != nil {
		t.Fatal(err)
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	ad, err := saved.Driver.Restore(w)
	if err != nil {
		t.Fatal(err)
	}
	b := &saved.Bus
	start := b.Pos
	h := w.Stops[232].Heading
	ob := VehicleObstacle{Pos: start.Add(sim.UnitFromAngle(h).Mul(90)), Heading: h, Length: 46, Width: 22}
	for tick := 0; tick < 180; tick++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(b, &saved.Route, w, nil, []VehicleObstacle{ob}, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if hit, _, _ := b.CollidesBody(sim.OBB{Center: ob.Pos, Heading: h, HalfLen: 23, HalfWid: 11}); hit {
			t.Fatal("forced through stopped front traffic")
		}
	}
	if b.Pos.Distance(start) > 5 {
		t.Fatal("front queue no longer holds the bus")
	}
}
