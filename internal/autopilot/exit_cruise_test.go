package autopilot

import (
	"encoding/json"
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestTurnExitKeepsPhysicalLaneAcrossCachedReturnTaper(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lane := range []int{0, 1} {
			t.Run(fmt.Sprintf("h%.2f/lane%d", heading, lane), func(t *testing.T) {
				const lw = 52.
				forward := sim.UnitFromAngle(heading)
				right := sim.Vec2{X: -forward.Y, Y: forward.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: forward.Mul(-3000), P2: forward.Mul(6000), Lanes: 4, Type: sim.RoadAvenue4Lane}}}
				w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", forward.Mul(-2000), heading, 4, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", forward.Mul(5000), heading, 4, sim.RGBA(0, 0, 0, 255), lw)}
				ad := NewAutopilotDriver()
				ad.pathFromID, ad.pathToID = 0, 1
				ad.BerserkMode = true
				ad.CanSkipTargetStop = true
				// Legacy route curves right after the turn. The physical lane must not
				// follow that taper, nor receive the same lane shift a second time.
				ad.drivePath = []sim.Vec2{forward.Mul(400).Add(right.Mul(26)), forward.Mul(800).Add(right.Mul(78)), forward.Mul(4500).Add(right.Mul(78))}
				ad.cumLen = []float64{0, ad.drivePath[1].Distance(ad.drivePath[0]), 0}
				ad.cumLen[2] = ad.cumLen[1] + ad.drivePath[2].Distance(ad.drivePath[1])
				ad.exitCruise = &sim.BusLaneMergeRequest{Heading: heading}
				lateral := (float64(lane) + .5) * lw
				for _, along := range []float64{500, 650, 800, 1200, 1800} {
					pos := forward.Mul(along).Add(right.Mul(lateral))
					bus := sim.NewBus(pos.X, pos.Y, heading)
					s, _ := ad.trackPath(pos)
					f := autopilotFrame{bus: bus, world: w, laneWidth: lw, s: s, remaining: 4000, speedAbs: 100, distToStop: 4000, targetStop: w.Stops[1]}
					if !ad.updateExitCruise(f, 1./60) {
						t.Fatalf("did not enter straight mode at %.0f", along)
					}
					_, _, target, _ := ad.pursuitSteering(f)
					if math.Abs(target.Dot(right)-lateral) > 1e-6 {
						t.Fatalf("lane target drifted to %.1f, want %.1f", target.Dot(right), lateral)
					}
				}
				data, err := json.Marshal(ad.Snapshot())
				if err != nil {
					t.Fatal(err)
				}
				var snapshot DriverSnapshot
				if err = json.Unmarshal(data, &snapshot); err != nil {
					t.Fatal(err)
				}
				restored, err := snapshot.Restore(w)
				if err != nil {
					t.Fatal(err)
				}
				if restored.exitCruise == nil || math.Abs(restored.exitCruise.Target.Dot(right)-lateral) > 1e-6 {
					t.Fatal("lost physical lane on restore")
				}
			})
		}
	}
}

func TestLeftTurnJoinsOutgoingTravelLaneInsideJunction(t *testing.T) {
	for _, lanes := range []int{4, 6} {
		for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			t.Run(fmt.Sprintf("lanes%d/h%.2f", lanes, heading), func(t *testing.T) {
				const lw = 52.
				w, _ := turnTestWorld(lanes, lanes, heading, lw)
				ad := NewAutopilotDriver()
				ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
				in := sim.UnitFromAngle(heading)
				out := sim.Vec2{X: in.Y, Y: -in.X}
				right := sim.Vec2{X: -out.Y, Y: out.X}
				geometry := w.JunctionAt(sim.Vec2{}, lw)
				mouth := math.Abs(out.X)*geometry.HalfX + math.Abs(out.Y)*geometry.HalfY
				checked := false
				for i := 1; i < len(ad.drivePath); i++ {
					a, b := ad.drivePath[i-1], ad.drivePath[i]
					da, db := a.Dot(out), b.Dot(out)
					probe := mouth + 60
					if da <= probe && db >= probe && db > da {
						checked = true
						p := a.Add(b.Sub(a).Mul((probe - da) / (db - da)))
						want := (float64(lanes/2) - .5) * lw
						if math.Abs(p.Dot(right)-want) > 1 || b.Sub(a).Normalize().Dot(out) < .99 {
							t.Fatalf("still changing lanes after turn: %.1f want %.1f", p.Dot(right), want)
						}
					}
				}
				if !checked {
					t.Fatal("missing straight exit")
				}
			})
		}
	}
}

func TestArticulatedLeftExitDoesNotDriveOntoShoulder(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("w%.0f/berserk%v", lw, berserk), func(t *testing.T) {
				w, stops := turnTestWorld(8, 4, 0, lw)
				w.Roads[0].Type = sim.RoadBusBoulevard8Lane
				w.Roads[1].P1.Y = -6000
				w.Roads[1].P2.Y = 6000
				stops[1] = sim.NewBusStop(1, "to", sim.Vec2{Y: -5000}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
				w.Stops = stops
				w.RebuildIntersections()
				bus := sim.NewBus(-1400, .5*lw, 0)
				bus.SetModel(sim.BusArticulated)
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				ad.articulatedPath = true
				ad.PrepareLeg(w, stops[0], stops[1], lw)
				ad.ordinaryBusLeft = true
				ad.PrepareLeg(w, stops[0], stops[1], lw)
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				tls.Intersections[0].Phase = sim.PhaseEWLeftGreen
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				mouth := w.JunctionAt(sim.Vec2{}, lw).HalfY
				for tick := 0; tick < 3600; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					if -bus.Pos.Y > mouth+bus.OverallLength() {
						for _, part := range bus.BodyParts() {
							for _, corner := range sim.BodyCorners(part) {
								if corner.X < -.5 || corner.X > 2*lw+.5 {
									t.Fatalf("exit left travel lanes tick%d pos%v corner%v", tick, bus.Pos, corner)
								}
							}
						}
					}
					if -bus.Pos.Y > mouth+1000 {
						if berserk && lw >= 52 && tick >= 20*60 {
							t.Fatalf("broad left turn retained unnecessary crawl: %.2fs", float64(tick+1)/60)
						}
						t.Logf("turn and exit: %.2fs", float64(tick+1)/60)
						return
					}
				}
				t.Fatalf("failed to finish turn: pos%v speed%.1f", bus.Pos, bus.Speed)
			})
		}
	}
}

func TestBlockedExitCruiseMergeKeepsClearCurrentLaneMoving(t *testing.T) {
	for _, blockedAhead := range []bool{false, true} {
		t.Run(fmt.Sprint(blockedAhead), func(t *testing.T) {
			const lw = 52.
			w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -10000}, P2: sim.Vec2{X: 10000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
			for id, x := range []float64{-9000, 9000} {
				w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", sim.Vec2{X: x}, 0, 8, sim.RGBA(0, 0, 0, 255), lw))
			}
			bus := sim.NewBus(-2000, 3.5*lw, 0)
			bus.SetModel(sim.BusArticulated)
			ad := NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			ad.exitCruise = &sim.BusLaneMergeRequest{Junction: sim.Vec2{X: -4000}, Heading: 0, Target: sim.Vec2{Y: 2.5 * lw}, Joined: true}
			obs := []VehicleObstacle{{Pos: sim.Vec2{X: -2000, Y: 2.5 * lw}, Heading: 0, Length: 46, Width: 22}}
			if blockedAhead {
				obs = append(obs, VehicleObstacle{Pos: sim.Vec2{X: -1920, Y: 3.5 * lw}, Heading: 0, Length: 46, Width: 22})
			}
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
			start := bus.Pos
			for tick := 0; tick < 180; tick++ {
				th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, obs, lw, 1./60)
				if tick == 0 && !blockedAhead && (th <= 0 || ad.LastTargetSpeed <= 0 || ad.LastPursuitPt.Y != start.Y) {
					t.Fatal("blocked lateral merge stopped clear forward corridor")
				}
				bus.Update(1./60, th, br, st, hb, doors)
				for _, ob := range obs {
					if sweepBodiesCollision(bus.BodyParts(), ob, 0, 0, 0) {
						t.Fatalf("contact tick%d", tick)
					}
				}
			}
			if blockedAhead && bus.Pos.Distance(start) > .01 {
				t.Fatal("moved into stopped current-lane leader")
			}
			if !blockedAhead && bus.Pos.X-start.X < 200 {
				t.Fatal("clear current lane remained deadlocked")
			}
		})
	}
}
