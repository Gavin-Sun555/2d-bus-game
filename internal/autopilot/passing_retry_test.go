package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Reproduce the slot-one controller state after a cancelled lane change:
// already back in the curb lane, but still marked Passing behind an 80km/h car.
func passingRetrySetup(heading float64, model sim.BusModel) (*AutopilotDriver, *sim.Bus, *sim.World, *sim.Route, VehicleObstacle) {
	const lw = 52.
	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-10000), P2: u.Mul(10000), Lanes: 4, Type: sim.RoadAvenue4Lane}}}
	for id, along := range []float64{-9000, 9000} {
		w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", u.Mul(along), heading, 4, sim.RGBA(0, 0, 0, 255), lw))
	}
	p := u.Mul(-2000).Add(r.Mul(78))
	bus := sim.NewBus(p.X, p.Y, heading)
	bus.SetModel(model)
	bus.Speed, bus.MaxSpeedFwd = 448.34, 1000
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.BerserkMaxSpeed = 1000
	ad.articulatedPath = bus.IsArticulated()
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
	ad.holdLaneOffset(0)
	ad.OvertakeTimer = 35
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	lead := VehicleObstacle{Pos: p.Add(u.Mul(98.62)), Heading: heading, Speed: 444.444, Length: 46, Width: 22}
	return ad, bus, w, route, lead
}

func TestPassingAtBaseLaneRetriesAndCompletesSafeOvertake(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
			t.Run(fmt.Sprintf("model%d/h%.2f", model, heading), func(t *testing.T) {
				ad, bus, w, route, lead := passingRetrySetup(heading, model)
				u := sim.UnitFromAngle(heading)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				for tick := 0; tick < 480; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, nil, []VehicleObstacle{lead}, 52, 1./60)
					if tick == 0 && ad.TargetLaneOffset > -50 {
						t.Fatal("stale Passing state prevented clear-lane retry")
					}
					before := bus.Pos
					bus.Update(1./60, th, br, st, hb, doors)
					lead.Pos = lead.Pos.Add(u.Mul(lead.Speed / 60))
					if bus.Pos.Distance(before) > math.Abs(bus.Speed)/60+.05 {
						t.Fatal("relocated during overtake")
					}
					if sweepBodiesCollision(bus.BodyParts(), lead, 0, 0, 0) {
						t.Fatalf("contact during overtake tick%d", tick)
					}
					for _, part := range bus.BodyParts() {
						if part.HalfLen <= 0 {
							continue
						}
						for _, corner := range sim.BodyCorners(part) {
							if side := corner.Dot(r); side < -.1 || side > 104.1 {
								t.Fatalf("left travel lanes tick%d side%.1f", tick, side)
							}
						}
					}
					if bus.Pos.Sub(lead.Pos).Dot(u) > bus.OverallLength()+lead.Length/2+30 {
						return
					}
				}
				t.Fatal("did not finish renewed pass")
			})
		}
	}
}

func TestPassingRetryRetainsGapAndSettlingGuards(t *testing.T) {
	for _, scenario := range []string{"alongside", "fast rear", "still steering", "trailer turning", "already passing"} {
		t.Run(scenario, func(t *testing.T) {
			ad, bus, w, route, lead := passingRetrySetup(0, sim.BusArticulated)
			obstacles := []VehicleObstacle{lead}
			switch scenario {
			case "alongside":
				obstacles = append(obstacles, VehicleObstacle{Pos: bus.Pos.Add(sim.Vec2{Y: -52}), Heading: 0, Speed: bus.Speed, Length: 46, Width: 22})
			case "fast rear":
				obstacles = append(obstacles, VehicleObstacle{Pos: bus.Pos.Add(sim.Vec2{X: -250, Y: -52}), Heading: 0, Speed: 900, Length: 46, Width: 22})
			case "still steering":
				bus.SteeringAngle = .12
			case "trailer turning":
				bus.TrailerHeading = .15
			case "already passing":
				bus.Pos.Y = 26
				bus.ResetArticulation()
				ad.holdLaneOffset(-52)
			}
			ad.ComputeControls(bus, route, w, nil, obstacles, 52, 1./60)
			if scenario == "already passing" {
				if ad.OvertakeState != OvertakePassing || ad.TargetLaneOffset != -52 {
					t.Fatalf("reset active pass or changed committed lane: state%d offsets%.2f/%.2f", ad.OvertakeState, ad.LaneOffset, ad.TargetLaneOffset)
				}
			} else if ad.TargetLaneOffset != 0 {
				t.Fatalf("retried into blocked gap or before settling: state%d offsets%.2f/%.2f wheel%.2f", ad.OvertakeState, ad.LaneOffset, ad.TargetLaneOffset, bus.SteeringAngle)
			}
		})
	}
}
