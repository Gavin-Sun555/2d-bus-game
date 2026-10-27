package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCarYieldCoversEveryCrossedBusLane(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for model := sim.BusStandard; model < sim.BusModelCount; model++ {
				for _, outward := range []bool{false, true} {
					for lane := 0; lane < 4; lane++ {
						t.Run(fmt.Sprintf("w%.0f/h%.2f/model%d/out%v/lane%d", lw, h, model, outward, lane), func(t *testing.T) {
							u := sim.UnitFromAngle(h)
							r := sim.Vec2{X: -u.Y, Y: u.X}
							start, end := 3.5*lw, .5*lw
							if outward {
								start, end = end, start
							}
							b := sim.NewBus(r.X*start, r.Y*start, h)
							b.SetModel(model)
							b.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: r.Mul(end), Heading: h, Remaining: .3}
							body := b.BodyEnvelope()
							car := &sim.TrafficCar{Pos: body.Center.Sub(u.Mul(body.HalfLen + 24 + 40)).Add(r.Mul((float64(lane)+.5)*lw - body.Center.Dot(r))), Heading: h, Length: 48, Width: 22}
							f := carFrame{car: car, bus: b, buses: []*sim.Bus{b}, world: &sim.World{}, laneWidth: lw}
							if speed := (&TrafficManager{}).busMergeYieldSpeed(f, 100); speed != 0 || car.YieldingToBus != b {
								t.Fatalf("crossed lane did not grant priority: speed%.2f yield%v", speed, car.YieldingToBus == b)
							}
						})
					}
				}
			}
		}
	}
}

func TestPlayerBusKeepsCarYieldGrant(t *testing.T) {
	for _, playerFirst := range []bool{false, true} {
		t.Run(fmt.Sprint(playerFirst), func(t *testing.T) {
			player := sim.NewBus(0, 26, 0)
			ai := sim.NewBus(-20, 130, 0)
			player.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 78}, Remaining: .3}
			ai.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 78}, Remaining: .3}
			car := &sim.TrafficCar{Pos: sim.Vec2{X: -130, Y: 78}, Length: 48, Width: 22}
			buses := []*sim.Bus{ai, player}
			if playerFirst {
				buses = []*sim.Bus{player, ai}
			}
			f := carFrame{car: car, bus: player, buses: buses, world: &sim.World{}, laneWidth: 52}
			tm := &TrafficManager{}
			if speed := tm.busMergeYieldSpeed(f, 150); speed != 0 || car.YieldingToBus != player {
				t.Fatalf("player grant displaced by AI: speed%.2f player%v", speed, car.YieldingToBus == player)
			}

			player.Pos = car.Pos // The player is now alongside, without a safe rear slot.
			if speed := tm.busMergeYieldSpeed(f, 150); speed != 0 || car.YieldingToBus != ai {
				t.Fatal("unsafe player request displaced a valid AI slot")
			}
			player.Pos = sim.Vec2{Y: 26}
			player.LaneMergeRequest = nil
			if speed := tm.busMergeYieldSpeed(f, 150); speed != 0 || car.YieldingToBus != ai {
				t.Fatal("AI did not receive slot after player released it")
			}
			ai.LaneMergeRequest = nil
			if speed := tm.busMergeYieldSpeed(f, 150); speed != 150 || car.YieldingToBus != nil {
				t.Fatal("car stayed reserved after both merges finished")
			}
		})
	}
}

func TestCarPriorityDoesNotForceUnsafeIntermediateYield(t *testing.T) {
	for _, scenario := range []string{"alongside", "ahead", "fast rear", "outside", "opposite", "expired"} {
		t.Run(scenario, func(t *testing.T) {
			b := sim.NewBus(0, 26, 0)
			b.LaneMergeRequest = &sim.BusLaneMergeRequest{Target: sim.Vec2{Y: 182}, Remaining: .3}
			c := &sim.TrafficCar{Pos: sim.Vec2{X: -112, Y: 78}, Length: 48, Width: 22}
			switch scenario {
			case "alongside":
				c.Pos.X = 0
			case "ahead":
				c.Pos.X = 110
			case "fast rear":
				c.Speed = 200
			case "outside":
				c.Pos.Y = 260
			case "opposite":
				c.Heading = math.Pi
			case "expired":
				b.LaneMergeRequest.Remaining = 0
			}
			f := carFrame{car: c, bus: b, buses: []*sim.Bus{b}, world: &sim.World{}, laneWidth: 52}
			if speed := (&TrafficManager{}).busMergeYieldSpeed(f, 100); speed != 100 || c.YieldingToBus != nil {
				t.Fatalf("unsafe/unrelated car forced to stop: %.1f", speed)
			}
		})
	}
}

func TestPriorityMergeClearsStationaryIntermediateCar(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for model := sim.BusStandard; model < sim.BusModelCount; model++ {
			t.Run(fmt.Sprintf("h%.2f/model%d", h, model), func(t *testing.T) {
				const lw = 52.
				u := sim.UnitFromAngle(h)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
				w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-3000), h, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", u.Mul(3000), h, 8, sim.RGBA(0, 0, 0, 255), lw)}
				b := sim.NewBus(0, 0, h)
				b.SetModel(model)
				b.Pos = u.Mul(-1500).Add(r.Mul(.5 * lw))
				start := b.Pos
				geometry := w.JunctionAt(sim.Vec2{}, lw)
				stopDistance := geometry.HalfX + sim.JunctionStopInset
				if math.Abs(u.X) < .5 {
					stopDistance = geometry.HalfY + sim.JunctionStopInset
				}
				// At the mandatory-merge holding line the bus cannot create a
				// larger gap by simply driving forward beside the stopped car.
				junction := u.Mul(-1500 + stopDistance + b.Length/2 + 30)
				req := &sim.BusLaneMergeRequest{Target: r.Mul(3.5 * lw), Heading: h, Remaining: .3, RightTurn: true, Junction: junction}
				b.LaneMergeRequest = req
				pub := autopilot.NewAutopilotDriver()
				pub.BerserkMode = true
				pub.CanSkipTargetStop = true
				snap := pub.Snapshot()
				snap.ApproachMerge = req
				snap.SkippedStop = -1
				snap.DrivePath = []sim.Vec2{u.Mul(-3000).Add(r.Mul(.5 * lw)), u.Mul(3000).Add(r.Mul(.5 * lw))}
				snap.PathFromID = 0
				snap.PathToID = 1
				snap.PathLane = lw
				snap.PathSynced = true
				snap.PathRevision = 3
				snap.ArticulatedPath = b.IsArticulated()
				ad, err := snap.Restore(w)
				if err != nil {
					t.Fatal(err)
				}
				body := b.BodyEnvelope()
				car := &sim.TrafficCar{ID: 1, Pos: body.Center.Sub(u.Mul(body.HalfLen + 24 + 40)).Add(r.Mul(lw)), Heading: h, Length: 48, Width: 22, IsEW: math.Abs(u.X) > .5, LaneIndex: 1}
				tm := &TrafficManager{Cars: []*sim.TrafficCar{car}}
				tls := sim.NewTrafficLightSystem()
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				granted, joined := false, false
				for tick := 0; tick < 1800; tick++ {
					tm.StepCar(1./60, car, 0, b, tls, 1, lw, w)
					granted = granted || car.YieldingToBus == b
					ob := autopilot.VehicleObstacle{Pos: car.Pos, Heading: car.Heading, Speed: car.Speed, Length: car.Length, Width: car.Width, YieldingTo: car.YieldingToBus}
					th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, tls, []autopilot.VehicleObstacle{ob}, lw, 1./60)
					b.Update(1./60, th, br, st, hb, doors)
					if hit, _, _ := sim.CheckBusAtBody(b, sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length / 2, HalfWid: b.Width / 2}, sim.OBB{Center: car.Pos, Heading: car.Heading, HalfLen: 24, HalfWid: 11}); hit {
						t.Fatalf("contact at tick%d", tick)
					}
					aligned := true
					for _, part := range b.BodyParts() {
						if part.HalfLen > 0 && math.Abs(part.Center.Dot(r)-3.5*lw)+part.HalfLen*math.Abs(math.Sin(part.Heading-h))+part.HalfWid*math.Abs(math.Cos(part.Heading-h)) > lw/2-.25 {
							aligned = false
						}
					}
					if aligned && b.Pos.Sub(start).Dot(u) > 50 {
						joined = true
						break
					}
				}
				if !granted || !joined {
					t.Fatalf("priority merge stalled: granted%v joined%v distance%.1f pos%v", granted, joined, b.Pos.Distance(start), b.Pos)
				}
				b.LaneMergeRequest = nil
				car.TargetSpeed = 100
				for tick := 0; tick < 60; tick++ {
					tm.StepCar(1./60, car, 0, b, tls, 1, lw, w)
				}
				if car.YieldingToBus != nil || car.Speed < 10 {
					t.Fatal("car did not resume after bus cleared")
				}
			})
		}
	}
}
