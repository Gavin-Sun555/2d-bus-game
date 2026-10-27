package autopilot

import (
	"encoding/json"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// Use the real steering actuator and signal controller in every orientation.
// A forced emergency stop in the middle must not lose the turn or lane intent.
func TestUTurnResumesThroughJunction(t *testing.T) {
	for _, lanes := range []int{2, 4, 6, 8} {
		for _, lw := range []float64{42, 52, 78} {
			for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, berserk := range []bool{false, true} {
					t.Run(fmt.Sprintf("lanes%d/width%.0f/h%.2f/berserk%v", lanes, lw, h, berserk), func(t *testing.T) {
						f := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -f.Y, Y: f.X}
						roadLanes := [2]int{2, 2}
						if math.Abs(f.X) > .5 {
							roadLanes[0] = lanes
						} else {
							roadLanes[1] = lanes
						}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -2000}, P2: sim.Vec2{X: 2000}, Lanes: roadLanes[0]}, {P1: sim.Vec2{Y: -2000}, P2: sim.Vec2{Y: 2000}, Lanes: roadLanes[1]}}}
						w.RebuildIntersections()
						w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", f.Mul(-900), h, lanes, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", f.Mul(-1400), h+math.Pi, lanes, sim.RGBA(0, 0, 0, 255), lw)}
						g := w.JunctionAt(sim.Vec2{}, lw)
						half := math.Abs(f.X)*g.HalfX + math.Abs(f.Y)*g.HalfY
						out := (float64(lanes)/2 - .5) * lw
						ad := NewAutopilotDriver()
						ad.BerserkMode = berserk
						ad.BerserkMaxSpeed = 666.6666667
						ad.drivePath = []sim.Vec2{f.Mul(-900).Add(r.Mul(lw / 2))}
						ad.drivePath = append(ad.drivePath, sim.UTurnPath(sim.Vec2{}, h, lw/2, out, g)...)
						ad.drivePath = append(ad.drivePath, f.Mul(-1400).Sub(r.Mul(out)))
						ad.cumLen = make([]float64, len(ad.drivePath))
						for i := 1; i < len(ad.drivePath); i++ {
							ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
						}
						turn, found := ad.intersectionExitTurn(sim.Vec2{}, f.X, f.Y, half+sim.JunctionStopInset, (float64(lanes)/2+.2)*lw)
						if !found || turn != routeTurnLeft {
							t.Fatalf("U-turn not classified as left movement: %v/%v", turn, found)
						}
						start := f.Mul(-half - sim.JunctionStopInset - 120).Add(r.Mul(lw / 2))
						bus := sim.NewBus(start.X, start.Y, h)
						bus.MaxSpeedFwd = 690
						route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						for _, i := range tls.Intersections {
							if math.Abs(f.X) > .5 {
								if lanes >= 4 {
									i.Phase = sim.PhaseEWLeftGreen
								} else {
									i.Phase = sim.PhaseEWGreen
								}
							} else {
								if lanes >= 4 {
									i.Phase = sim.PhaseNSLeftGreen
								} else {
									i.Phase = sim.PhaseNSGreen
								}
							}
						}
						// Hold the entry red first; the turn commitment must not exempt entry.
						phases := make([]sim.LightPhase, len(tls.Intersections))
						for j, inter := range tls.Intersections {
							phases[j] = inter.Phase
							if math.Abs(f.X) > .5 {
								inter.Phase = sim.PhaseNSGreen
							} else {
								inter.Phase = sim.PhaseEWGreen
							}
						}
						bus.Speed = 60
						for n := 0; n < 240; n++ {
							th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1.0/60)
							bus.Update(1.0/60, th, br, st, hb, d)
							if bus.Pos.Dot(f)+bus.Length/2 > -half-sim.JunctionStopInset {
								t.Fatal("U-turn crossed red entry line")
							}
						}
						for j, inter := range tls.Intersections {
							inter.Phase = phases[j]
						}
						paused, exited := false, false
						acceleratedBeforeExit := false
						hold := 0
						idle := 0
						for step := 0; step < 2400; step++ {
							if !paused && math.Abs(sim.NormalizeAngle(bus.Heading-h+math.Pi/2)) < .15 && w.InsideJunction(bus.Pos, lw, 0) {
								paused = true
								hold = 120
								// Simulate a process restart during the turn, including its cached path.
								raw, err := json.Marshal(ad.Snapshot())
								if err != nil {
									t.Fatal(err)
								}
								var saved DriverSnapshot
								if err = json.Unmarshal(raw, &saved); err != nil {
									t.Fatal(err)
								}
								restored, err := saved.Restore(w)
								if err != nil {
									t.Fatal(err)
								}
								ad = restored
							}
							th, br, st, hb, d, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, 1.0/60)
							if hold == 0 && paused && ad.uTurn != nil &&
								bus.Pos.Dot(f) >= -half-bus.Length/2-80 &&
								math.Cos(bus.Heading-h) < -.995 && math.Abs(bus.SteeringAngle) <= .06 &&
								ad.LastTargetSpeed > 70 && th > 0 && br == 0 {
								if !acceleratedBeforeExit {
									// Releasing the turn cap must still brake for a queue ahead.
									blockedDriver, blockedBus := *ad, *bus
									blockedBus.Speed = 70
									forward := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
									queue := VehicleObstacle{Pos: bus.Pos.Add(forward.Mul(120)), Heading: bus.Heading, Length: bus.Length, Width: bus.Width}
									_, blockedBrake, _, _, _, _ := blockedDriver.ComputeControls(&blockedBus, route, w, tls, []VehicleObstacle{queue}, lw, 1.0/60)
									if blockedDriver.LastTargetSpeed >= 70 || blockedBrake == 0 {
										t.Fatal("straightened U-turn bus failed to brake for outgoing queue")
									}
								}
								acceleratedBeforeExit = true
							}
							if hold > 0 {
								th = 0
								br = 1
								bus.Speed = 0
								hold--
							}
							bus.Update(1.0/60, th, br, st, hb, d)
							if paused && hold == 0 && bus.Pos.Dot(f) < -half-bus.Length-20 && math.Cos(bus.Heading-h) < -.95 {
								signed := bus.Pos.Dot(r)
								yaw := bus.Heading - h
								extent := math.Abs(math.Sin(yaw))*bus.Length/2 + math.Abs(math.Cos(yaw))*bus.Width/2
								if signed+extent >= 0 || signed-extent < -float64(lanes)/2*lw {
									t.Fatalf("body leaves outgoing carriageway: pos%v heading%.2f lateral%.2f extent%.2f", bus.Pos, bus.Heading, signed, extent)
								}
								exited = true
								break
							}
							if hold == 0 && bus.Speed < 1 && w.InsideJunction(bus.Pos, lw, -60) {
								idle++
							} else {
								idle = 0
							}
							if idle > 180 {
								t.Fatalf("turn deadlock: pos%v h%.2f speed%.1f target%.1f hold%.2f road%v", bus.Pos, bus.Heading, bus.Speed, ad.LastTargetSpeed, ad.busTurnHoldDistance, ad.roadSafetyHold)
							}
							if z := w.CheckBoundaries(bus.Pos, bus.Width/2, lw).Zone; z == sim.ZoneHardBarrier {
								t.Fatalf("barrier: %v h%.2f", bus.Pos, bus.Heading)
							}
						}
						if lanes >= 6 && !acceleratedBeforeExit {
							t.Fatal("aligned bus kept U-turn crawl speed until beyond the junction")
						}
						if !paused || !exited {
							t.Fatalf("did not resume and exit: paused%v exited%v pos%v heading%.2f target%.1f", paused, exited, bus.Pos, bus.Heading, ad.LastTargetSpeed)
						}
					})
				}
			}
		}
	}
}
