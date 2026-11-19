package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func savedStationFollower(h float64) (*AutopilotDriver, autopilotFrame, *sim.Route, *sim.Bus) {
	const lw = 52.
	u, r := sim.UnitFromAngle(h), sim.Vec2{X: -math.Sin(h), Y: math.Cos(h)}
	w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3500), P2: u.Mul(3500), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
	for i, x := range []float64{-1500, 0, 1500} {
		w.Stops = append(w.Stops, sim.NewBusStop(i, "station", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), lw))
	}
	stop := w.Stops[1]
	p := stop.Pos.Add(u.Mul(-140.816476542778)).Add(r.Mul(-15.211581311552))
	b := sim.NewBus(p.X, p.Y, h+.2012192891361577)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading, b.TrailerInitialized, b.SteeringAngle, b.Speed = h+.1717600071309393, true, -.006904705202231024, 5.305540933954242
	p = stop.Pos.Add(u.Mul(95.908840929757)).Add(r.Mul(1.13710705749))
	lead := sim.NewBus(p.X, p.Y, h-.0147789242980874)
	lead.SetModel(sim.BusArticulated)
	lead.TrailerHeading, lead.TrailerInitialized, lead.DoorsOpen = h+.0388876392412329, true, true
	ad := NewAutopilotDriver()
	ad.DisallowOncomingOvertake = true
	ad.CruiseSpeedLimit = 1000
	route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
	f := autopilotFrame{bus: b, world: w, laneWidth: lw, targetStop: stop, speedAbs: math.Abs(b.Speed), remaining: 240, obstacles: []VehicleObstacle{ObserveBus(lead)}}
	return ad, f, route, lead
}

func TestSavedAngledFollowerBoardsBehindServingArticulatedBus(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("h%.2f", h), func(t *testing.T) {
			ad, f, route, lead := savedStationFollower(h)
			for tick := 0; tick < 60*15; tick++ {
				if tick == 20 {
					// Resuming a save halfway through the short entry must keep
					// the physical arc, rather than chase the bay center again.
					var err error
					ad, err = ad.Snapshot().Restore(f.world)
					if err != nil {
						t.Fatal(err)
					}
				}
				th, br, st, hb, doors, dock := ad.ComputeControls(f.bus, route, f.world, nil, f.obstacles, 52, 1./60)
				f.bus.Update(1./60, th, br, st, hb, doors)
				if hit, _, _ := sim.CheckBusesCollision(f.bus, lead); hit {
					t.Fatal("follower hit serving leader")
				}
				if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
					t.Fatal("follower hit station boundary")
				}
				if dock && f.bus.DoorsOpen {
					if !f.targetStop.ContainsBus(f.bus) {
						t.Fatal("boarded outside bay")
					}
					if math.Abs(sim.NormalizeAngle(f.bus.Heading-h)) >= .075 {
						t.Fatal("follower opened before straightening")
					}
					tail, _ := stationBodyRange(lead, sim.UnitFromAngle(h))
					gap := tail - f.bus.Pos.Dot(sim.UnitFromAngle(h)) - f.bus.Length/2
					if gap < 20 || gap > 40 {
						t.Fatalf("rear berth gap %.2f", gap)
					}
					return
				}
			}
			t.Fatalf("rear berth never served: pos%v yaw%.3f target%.2f plan%+v", f.bus.Pos, sim.NormalizeAngle(f.bus.Heading-h), ad.LastTargetSpeed, ad.stationDock)
		})
	}
}

func TestShortRearBerthAlignmentKeepsSafetyGuards(t *testing.T) {
	for _, name := range []string{"blocked sweep", "moving blocker", "outside bay", "no runway", "moving follower", "no world", "no lead"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _, _ := savedStationFollower(0)
			target := ad.availableStationBerths(f.bus, f.targetStop, f.obstacles)[0]
			switch name {
			case "blocked sweep", "moving blocker":
				ob := VehicleObstacle{Pos: f.bus.Pos.Add(sim.Vec2{X: f.bus.Length/2 + 8}), Heading: 0, Length: 4, Width: 4}
				if name == "moving blocker" {
					ob.Speed = -10
				}
				f.obstacles = append(f.obstacles, ob)
			case "outside bay":
				f.bus.Pos.Y = f.targetStop.Pos.Y - 35
			case "no runway":
				target = f.bus.Pos.Add(sim.Vec2{X: 5})
			case "moving follower":
				f.speedAbs = 20
			case "no world":
				f.world = nil
			case "no lead":
				f.obstacles = nil
			}
			if _, _, ok := stationActuatorAlignment(f, target); ok {
				t.Fatal("unsafe short rear-berth arc accepted")
			}
		})
	}
}
