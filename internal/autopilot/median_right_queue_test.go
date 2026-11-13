package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestMedianRightPrefersBusLaneUnlessQueueAndCrossingFavorCurb(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
				for _, tc := range []struct {
					name                string
					busQueue, curbQueue int
					middleLane          int
					nearCurb, movingBus bool
					want                bool
				}{
					{name: "empty bus lane"}, {name: "both lanes empty"},
					{name: "only curb queue", curbQueue: 2},
					{name: "equal queues", busQueue: 1, curbQueue: 1},
					{name: "bus queue shorter", busQueue: 1, curbQueue: 2},
					{name: "bus queue longer", busQueue: 2, curbQueue: 1, want: true},
					{name: "one waiting bus and clear curb", busQueue: 1, want: true},
					{name: "middle lane one busy", busQueue: 2, middleLane: 1},
					{name: "middle lane two busy", busQueue: 2, middleLane: 2},
					{name: "curb gap occupied", busQueue: 2, curbQueue: 1, nearCurb: true},
					{name: "flowing bus lane is not a queue", busQueue: 2, movingBus: true},
				} {
					t.Run(fmt.Sprintf("w%.0f/h%.2f/model%d/%s", lw, h, model, tc.name), func(t *testing.T) {
						w := medianRightTestWorld(lw, h)
						tls := sim.NewTrafficLightSystem()
						tls.BuildForWorld(w)
						u := sim.UnitFromAngle(h)
						r := sim.Vec2{X: -u.Y, Y: u.X}
						line := tls.Intersections[0].GetStopLineDistForHeading(h, lw)
						pos := u.Mul(-line - 1400).Add(r.Mul(lw * .5))
						b := sim.NewBus(pos.X, pos.Y, h)
						b.SetModel(model)
						b.Speed = 150
						ad := NewAutopilotDriver()
						ad.BerserkMode = true
						ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
						s, _ := ad.trackPath(pos)
						f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: s, speedAbs: 150, remaining: 4000}
						add := func(along, lane, speed float64) {
							f.obstacles = append(f.obstacles, VehicleObstacle{Pos: pos.Add(u.Mul(along)).Add(r.Mul((lane - .5) * lw)), Heading: h, Length: 46, Width: 22, Speed: speed})
						}
						for i := 0; i < tc.busQueue; i++ {
							speed := 0.
							if tc.movingBus {
								speed = 200
							}
							add(950+float64(i)*130, .5, speed)
						}
						for i := 0; i < tc.curbQueue; i++ {
							along := 1250 + float64(i)*100
							if tc.nearCurb {
								along = 100
							}
							add(along, 3.5, 0)
						}
						if tc.middleLane > 0 {
							add(500, float64(tc.middleLane)+.5, 200)
						}
						ad.selectMedianRightTurn(f)
						if ad.medianRightChoice == nil {
							t.Fatal("did not evaluate upcoming right turn")
						}
						if ad.curbBusRight != tc.want {
							t.Fatalf("curb=%v want %v", ad.curbBusRight, tc.want)
						}
					})
				}
			}
		}
	}
}

func TestMedianRightSettledCurbCruisesWhileCorridorRemainsLocked(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
			for _, model := range []sim.BusModel{sim.BusStandard, sim.BusDoubleDecker, sim.BusArticulated} {
				t.Run(fmt.Sprintf("w%.0f/h%.2f/model%d", lw, h, model), func(t *testing.T) {
					w := medianRightTestWorld(lw, h)
					tls := sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					u := sim.UnitFromAngle(h)
					r := sim.Vec2{X: -u.Y, Y: u.X}
					line := tls.Intersections[0].GetStopLineDistForHeading(h, lw)
					pos := u.Mul(-line - 1400).Add(r.Mul(lw * 3.5))
					b := sim.NewBus(pos.X, pos.Y, h)
					b.SetModel(model)
					b.Speed = 150
					ad := NewAutopilotDriver()
					ad.BerserkMode = true
					ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
					ad.medianRightChoice = &tls.Intersections[0].Center
					ad.curbBusRight = true
					s, _ := ad.trackPath(pos)
					ad.holdLaneOffset(pos.Sub(ad.pointAt(s)).Dot(r))
					ad.approachMerge = &sim.BusLaneMergeRequest{Target: r.Mul(lw * 3.5), Heading: h, RightTurn: true, Junction: tls.Intersections[0].Center}
					f := autopilotFrame{bus: b, world: w, tls: tls, laneWidth: lw, s: s, speedAbs: b.Speed, remaining: 4000, targetStop: w.Stops[1], distToStop: 4000}
					ad.advanceApproachMerge(f, 1./60)
					if ad.approachMerge == nil || !ad.approachMergeJoined || !b.LaneMergeRequest.Joined {
						t.Fatal("physical merge completion was not separated from path lock")
					}
					if speed := ad.targetSpeed(f); speed <= 160 {
						t.Fatal("settled curb cruise still limited", speed)
					}
					// Removing the merge cap never removes the following-distance cap.
					f.obstacles = []VehicleObstacle{{Pos: b.Pos.Add(u.Mul(75)), Heading: h, Length: 46, Width: 22}}
					if speed := ad.targetSpeed(f); speed > 1 {
						t.Fatal("settled curb cruise ignored a stopped leader", speed)
					}
					f.obstacles = nil
					// A lateral error, or a trailer still crossing lanes, must restore maneuver speed.
					b.Pos = b.Pos.Sub(r.Mul(lw * .8))
					f.bus = b
					ad.advanceApproachMerge(f, 1./60)
					if ad.approachMergeJoined {
						t.Fatal("unsettled vehicle treated as merged")
					}
					if speed := ad.targetSpeed(f); speed > 240 {
						t.Fatal("unsettled lane change accelerated", speed)
					}
					if b.IsArticulated() {
						b.Pos = pos
						b.TrailerHeading = h + .3
						ad.advanceApproachMerge(f, 1./60)
						if ad.approachMergeJoined || ad.targetSpeed(f) > 100 {
							t.Fatal("trailer still crossing lanes was allowed to cruise")
						}
					}
				})
			}
		}
	}
}
