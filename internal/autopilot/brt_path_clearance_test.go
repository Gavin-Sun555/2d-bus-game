package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestBRTLaneReturnClearsWholeArticulatedBusBeforeIsland(t *testing.T) {
	const lw = 52.
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.UnitFromAngle(h)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		w := &sim.World{CenterBRT: true, Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, MedianBusLane: true}}}
		stop := sim.NewBusStop(0, "platform", sim.Vec2{}, h, 8, sim.RGBA(0, 0, 0, 255), lw)
		stop.CenterBRT = true
		stop.UpdateGeometry(lw)
		w.Stops = []*sim.BusStop{stop}
		point := func(s, lat float64) sim.Vec2 { return u.Mul(s).Add(r.Mul(lat)) }
		// The saved slot 3 transition returns across the platform divider from
		// 340px before its center to only 30px before the center.
		path := []sim.Vec2{point(-1600, 78), point(-450, 78), point(-340, 78), point(-30, 26), point(1000, 26)}
		repaired, changed := clearBRTPlatformTransitions(w, path, lw, true)
		if !changed {
			t.Fatal("unsafe platform return was retained")
		}
		bus := sim.NewBus(path[0].X, path[0].Y, h)
		bus.SetModel(sim.BusArticulated)
		ad := NewAutopilotDriver()
		ad.drivePath = repaired
		ad.cumLen = make([]float64, len(repaired))
		for i := 1; i < len(repaired); i++ {
			ad.cumLen[i] = ad.cumLen[i-1] + repaired[i].Distance(repaired[i-1])
		}
		bus.Speed = 80
		for tick := 0; tick < 60*35; tick++ {
			s, lat := ad.trackPath(bus.Pos)
			f := autopilotFrame{bus: bus, world: w, laneWidth: lw, s: s, lateralErr: lat, speedAbs: math.Abs(bus.Speed)}
			steer, _, _, _ := ad.pursuitSteering(f)
			th, br := ad.longitudinalControls(bus.Speed, 80)
			bus.Update(1./60, th, br, steer, false, false)
			if bus.HitsHardBarrier(w, lw) {
				t.Fatalf("heading%.2f hit island pos%v", h, bus.Pos)
			}
			for _, part := range bus.BodyParts() {
				if part.HalfLen > 0 {
					for _, corner := range sim.BodyCorners(part) {
						if corner.Dot(r) < 0 {
							t.Fatal("crossed opposing lane")
						}
					}
				}
			}
			if bus.Pos.Dot(u) > 500 {
				break
			}
			if tick == 60*35-1 {
				t.Fatal("did not pass station")
			}
		}
	}
}

func TestReverseRecoveryDistinguishesRailViaducts(t *testing.T) {
	w := &sim.World{}
	w.BuildMap(sim.MapThemeMetropolis, 52)
	foundLevel, foundViaduct := false, false
	for _, rail := range w.Railways() {
		for _, cross := range rail.Crossings {
			b := sim.NewBus(cross.Center.X, cross.Center.Y, math.Pi/2)
			b.SetModel(sim.BusArticulated)
			blocked := busOverlapsLevelCrossing(b, w)
			if cross.Level {
				foundLevel = true
				if !blocked {
					t.Fatal("level crossing lost reverse protection")
				}
			} else {
				foundViaduct = true
				if blocked {
					t.Fatal("viaduct treated as level crossing")
				}
			}
		}
	}
	if !foundLevel || !foundViaduct {
		t.Fatal("missing crossing coverage")
	}
}

func TestSplicedBRTTransitionIsRepairedOnlyAhead(t *testing.T) {
	const lw = 52.
	w := &sim.World{CenterBRT: true}
	st := sim.NewBusStop(0, "island", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)
	st.CenterBRT = true
	st.UpdateGeometry(lw)
	w.Stops = []*sim.BusStop{st}
	for _, entered := range []bool{false, true} {
		ad := NewAutopilotDriver()
		ad.pathWorld = w
		ad.pathLane = lw
		ad.articulatedPath = true
		ad.drivePath = []sim.Vec2{{X: -1600, Y: 78}, {X: -450, Y: 78}, {X: -340, Y: 78}, {X: -30, Y: 26}, {X: 1000, Y: 26}}
		ad.cumLen = make([]float64, len(ad.drivePath))
		for i := 1; i < len(ad.drivePath); i++ {
			ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
		}
		pos := sim.Vec2{X: -1200, Y: 78}
		if entered {
			pos = sim.Vec2{X: -300, Y: 70}
		}
		ad.trackPath(pos)
		if changed := ad.drivePath[1].X != -450; changed == entered {
			t.Fatalf("entered%v changed%v", entered, changed)
		}
	}
}

func TestRearBerthKeepsBrakingMarginInsideBoardingStrip(t *testing.T) {
	const lw = 52.
	st := sim.NewBusStop(0, "bay", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)
	st.Length = 336
	b := sim.NewBus(-500, st.Pos.Y, 0)
	b.SetModel(sim.BusArticulated)
	for _, extra := range []float64{0, 10} {
		lead := VehicleObstacle{Pos: sim.Vec2{X: -47.9 + extra, Y: st.Pos.Y}, Heading: 0, Length: 100, Width: 36}
		berths := availableStationBerths(b, st, []VehicleObstacle{lead})
		foundRear := false
		for _, target := range berths {
			if target.X < 0 {
				foundRear = true
				pose := *b
				pose.Pos = target.Sub(sim.Vec2{X: 4})
				if !st.ContainsBus(&pose) {
					t.Fatal("selected berth stops outside platform")
				}
			}
		}
		if foundRear != (extra > 0) {
			t.Fatalf("extra%.0f rear%v berths%v", extra, foundRear, berths)
		}
	}
}
