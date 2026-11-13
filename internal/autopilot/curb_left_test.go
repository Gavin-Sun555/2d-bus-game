package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"reflect"
	"testing"
)

func TestWideCurbLeftPassesStationaryMiddleLaneQueue(t *testing.T) {
	for _, config := range []struct {
		lanes       int
		bus, median bool
	}{{6, false, false}, {8, false, false}, {8, true, false}, {8, true, true}} {
		lanes := config.lanes
		for _, lw := range []float64{52, 78} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
					t.Run(fmt.Sprintf("lanes%d/bus%v/median%v/w%.0f/h%.2f/model%d", lanes, config.bus, config.median, lw, h, model), func(t *testing.T) {
						w, _ := turnTestWorld(lanes, lanes, h, lw)
						if config.bus {
							for i := range w.Roads {
								w.Roads[i].Type = sim.RoadBusBoulevard8Lane
								w.Roads[i].MedianBusLane = config.median
							}
						}
						w.CenterBRT = config.median
						in := sim.UnitFromAngle(h)
						right := sim.Vec2{X: -in.Y, Y: in.X}
						out := sim.Vec2{X: in.Y, Y: -in.X}
						curb := (float64(lanes)/2 - .5) * lw
						// Start on the road, beyond the departure stop at -1400.
						p := in.Mul(-1000).Add(right.Mul(curb))
						bus := sim.NewBus(p.X, p.Y, h)
						bus.SetModel(model)
						bus.Speed = 150
						ad := NewAutopilotDriver()
						ad.DisallowOncomingOvertake = true
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						for _, i := range tls.Intersections {
							i.Phase = sim.PhaseEWLeftGreen
							if math.Abs(in.Y) > .5 {
								i.Phase = sim.PhaseNSLeftGreen
							}
						}
						g := w.JunctionAt(sim.Vec2{}, lw)
						half := math.Abs(in.X)*g.HalfX + math.Abs(in.Y)*g.HalfY
						var obs []VehicleObstacle
						for _, along := range []float64{-half - 55, -half - 140, -half - 225, -half - 310} {
							pos := in.Mul(along).Add(right.Mul(curb - lw))
							obs = append(obs, VehicleObstacle{Pos: pos, Heading: h, Length: 46, Width: 22})
						}
						crossed := false
						for tick := 0; tick < 5400; tick++ {
							before := bus.Pos
							th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, 1./60)
							bus.Update(1./60, th, br, st, hb, doors)
							if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.05 {
								t.Fatal("bus relocated")
							}
							for _, ob := range obs {
								if sweepBodiesCollision(bus.BodyParts(), ob, 0, 0, 0) {
									t.Fatalf("hit parallel queue tick%d", tick)
								}
							}
							if !crossed && bus.Pos.Add(sim.UnitFromAngle(bus.Heading).Mul(bus.Length/2)).Dot(in) >= -half-sim.JunctionStopInset {
								crossed = true
								if math.Abs(bus.Pos.Dot(right)-curb) > 4 || ad.approachMerge != nil {
									t.Fatalf("forced a pre-turn lane change: pos%v merge%v", bus.Pos, ad.approachMerge)
								}
							}
							if bus.Pos.Dot(out) > half+350 {
								if !crossed {
									t.Fatal("entry not checked")
								}
								return
							}
						}
						d := ad.DiagnoseStop(bus, route, w, tls, obs, lw)
						t.Fatalf("curb turn stalled pos%v limits%v", bus.Pos, d.Limits)
					})
				}
			}
		}
	}
}

// An ordinary six-lane road turns left from the inner lane, so a saved
// inner-lane approach is already correct: the curb-left migration must keep
// it, and must never reset the vehicle or its journey.
func TestLegacyInnerLeftApproachKeptOnOrdinarySixLaneRoad(t *testing.T) {
	for _, entered := range []bool{false, true} {
		ad, f, route := loadSixLaneWedge(t)
		if entered {
			f.bus.Pos = sim.Vec2{X: -200, Y: 28930}
		} else {
			f.bus.Pos = sim.Vec2{X: -950, Y: 28930}
		}
		f.bus.Heading = 0
		f.bus.Speed = 0
		f.bus.SteeringAngle = 0
		f.bus.ResetArticulation()
		ad.pathRevision = currentDrivePathRevision - 1
		before := *f.bus
		index, trips := route.CurrentIndex, route.TotalTrips
		old := append([]sim.Vec2(nil), ad.drivePath...)
		ad.upgradeWideCurbPath(f.bus, f.world, 52)
		if f.bus.Pos != before.Pos || f.bus.Heading != before.Heading || f.bus.Speed != before.Speed || f.bus.PassengerCount != before.PassengerCount || f.bus.Durability != before.Durability || route.CurrentIndex != index || route.TotalTrips != trips {
			t.Fatal("path upgrade reset vehicle or journey")
		}
		if !reflect.DeepEqual(ad.drivePath, old) {
			t.Fatalf("entered%v: replaced the inner-lane approach on an ordinary road", entered)
		}
	}
}
