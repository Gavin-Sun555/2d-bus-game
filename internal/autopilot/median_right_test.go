package autopilot

import (
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func medianRightTestWorld(lw, h float64) *sim.World {
	u := sim.UnitFromAngle(h)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	out := r
	w := &sim.World{CenterBRT: true, Roads: []sim.RoadSegment{
		{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
		{P1: r.Mul(-4000), P2: r.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane, MedianBusLane: true},
	}}
	w.RebuildIntersections()
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-3000), h, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", out.Mul(2200), h+math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)}
	for _, s := range w.Stops {
		s.CenterBRT = true
		s.UpdateGeometry(lw)
	}
	return w
}

func TestMedianRightChoiceLocksAndPersists(t *testing.T) {
	for _, blocked := range []bool{false, true} {
		t.Run(fmt.Sprintf("blocked%v", blocked), func(t *testing.T) {
			const lw = 52.
			w := medianRightTestWorld(lw, 0)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			b := sim.NewBus(-1600, lw*.5, 0)
			b.Speed = 150
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			ad.trackPath(b.Pos)
			f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: ad.LastPathS, speedAbs: b.Speed, remaining: 4000}
			if blocked {
				f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1200, Y: 1.5 * lw}, Heading: 0, Length: 46, Width: 22}}
			}
			ad.selectMedianRightTurn(f)
			if ad.medianRightChoice == nil || ad.curbBusRight == blocked {
				t.Fatalf("wrong corridor choice: center%v curb%v", ad.medianRightChoice, ad.curbBusRight)
			}
			// Opposite traffic conditions must not alternate the committed corridor.
			if blocked {
				f.obstacles = nil
			} else {
				f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1200, Y: 1.5 * lw}, Heading: 0, Length: 46, Width: 22}}
			}
			ad.selectMedianRightTurn(f)
			if ad.curbBusRight == blocked {
				t.Fatal("committed turn corridor changed")
			}
			data, e := json.Marshal(ad.Snapshot())
			if e != nil {
				t.Fatal(e)
			}
			var snap DriverSnapshot
			if e = json.Unmarshal(data, &snap); e != nil {
				t.Fatal(e)
			}
			restored, e := snap.Restore(w)
			if e != nil {
				t.Fatal(e)
			}
			if restored.curbBusRight != ad.curbBusRight || restored.medianRightChoice == nil {
				t.Fatal("saved turn choice lost")
			}
		})
	}
}

func TestMedianRightWaitsForCrossLeftThenTurns(t *testing.T) {
	for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("model%d/h%.2f", model, h), func(t *testing.T) {
				const lw = 52.
				w := medianRightTestWorld(lw, h)
				u := sim.UnitFromAngle(h)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				b := sim.NewBus(0, 0, h)
				b.Pos = u.Mul(-1500).Add(r.Mul(lw * .5))
				b.SetModel(model)
				b.Speed = 250
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				it := tls.Intersections[0]
				it.Phase = sim.PhaseEWGreen
				if math.Abs(u.Y) > .5 {
					it.Phase = sim.PhaseNSGreen
				}
				obs := []VehicleObstacle{{Pos: u.Mul(-1000).Add(r.Mul(1.5 * lw)), Heading: h, Length: 46, Width: 22}}
				line := it.GetStopLineDistForHeading(h, lw)
				for tick := 0; tick < 1500; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, obs, lw, 1./60)
					b.Update(1./60, th, br, st, hb, doors)
					if b.Pos.Dot(u)+b.Length/2*math.Cos(b.Heading-h) > -line+1 {
						t.Fatalf("central right crossed wrong phase at tick%d pos%v", tick, b.Pos)
					}
				}
				if ad.curbBusRight {
					t.Fatal("chose occupied right-side lanes")
				}
				it.Phase = sim.MedianBusRightPhase(h)
				request := ad.SignalPriorityRequest(b, tls, lw)
				if request == nil || request.Phase != it.Phase {
					t.Fatal("priority requested wrong phase", request)
				}
				docked := false
				for tick := 0; tick < 6000; tick++ {
					th, br, st, hb, doors, dock := ad.ComputeControls(b, route, w, tls, obs, lw, 1./60)
					b.Update(1./60, th, br, st, hb, doors)
					if hit := b.CheckBoundaries(w, lw); hit.Zone == sim.ZoneHardBarrier {
						t.Fatal("protected turn hit boundary", b.Pos, hit.ObstacleName)
					}
					if violated, notice := tls.CheckPlayerViolation(b, lw); violated {
						t.Fatal(notice)
					}
					if dock {
						docked = true
						break
					}
				}
				if !docked {
					t.Fatalf("protected right turn did not dock: pos%v h%.2f target%.1f", b.Pos, b.Heading, ad.LastTargetSpeed)
				}
			})
		}
	}
}

func TestEarlyRightMergeHoldsCurbUntilPathJoins(t *testing.T) {
	const lw = 52.
	w := medianRightTestWorld(lw, 0)
	ad := NewAutopilotDriver()
	ad.drivePath = []sim.Vec2{{X: -2200, Y: 26}, {X: -1000, Y: 26}, {X: -700, Y: 182}, {X: -600, Y: 182}}
	ad.cumLen = []float64{0, 1200, 1538, 1638}
	ad.pathWorld = w
	b := sim.NewBus(-1400, 182, 0)
	b.Speed = 80
	ad.LaneOffset = 156
	ad.TargetLaneOffset = 156
	ad.approachMerge = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 182}, Heading: 0, RightTurn: true, Junction: sim.Vec2{}}
	if !ad.advanceApproachMerge(autopilotFrame{bus: b, world: w, laneWidth: lw, s: 800, speedAbs: 80}, 1./60) {
		t.Fatal("released curb merge before base path caught up")
	}
	if ad.TargetLaneOffset != 156 {
		t.Fatal("steered back toward central lane", ad.TargetLaneOffset)
	}
}

func TestMedianRightIntoNarrowerRoads(t *testing.T) {
	for _, lanes := range []int{2, 4, 6} {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("exit%d/berserk%v", lanes, berserk), func(t *testing.T) {
				const lw = 52.
				w := medianRightTestWorld(lw, 0)
				w.Roads[1].Lanes = lanes
				w.Roads[1].MedianBusLane = false
				w.Roads[1].Type = sim.RoadStreet2Lane
				w.Stops[1].CenterBRT = false
				w.Stops[1].RoadLanes = lanes
				w.Stops[1].UpdateGeometry(lw)
				b := sim.NewBus(-1600, lw*.5, 0)
				b.SetModel(sim.BusArticulated)
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				obs := []VehicleObstacle{{Pos: sim.Vec2{X: -900, Y: 1.5 * lw}, Heading: 0, Length: 46, Width: 22}}
				docked := false
				for tick := 0; tick < 9000; tick++ {
					tls.Update(1. / 60)
					th, br, st, hb, doors, dock := ad.ComputeControls(b, route, w, tls, obs, lw, 1./60)
					b.Update(1./60, th, br, st, hb, doors)
					if hit := b.CheckBoundaries(w, lw); hit.Zone == sim.ZoneHardBarrier {
						t.Fatalf("narrow exit boundary: %v h%.2f %s", b.Pos, b.Heading, hit.ObstacleName)
					}
					if v, n := tls.CheckPlayerViolation(b, lw); v {
						t.Fatal(n)
					}
					if dock {
						docked = true
						break
					}
				}
				if !docked {
					t.Fatalf("narrow exit failed to dock pos%v speed%.1f target%.1f", b.Pos, b.Speed, ad.LastTargetSpeed)
				}
				if ad.curbBusRight {
					t.Fatal("occupied general lanes selected")
				}
			})
		}
	}
}
