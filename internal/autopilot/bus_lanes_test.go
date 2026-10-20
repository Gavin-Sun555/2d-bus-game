package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func busLaneTestWorld(lw float64) *sim.World {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -3000}, P2: sim.Vec2{X: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane}, {P1: sim.Vec2{Y: -3000}, P2: sim.Vec2{Y: 3000}, Lanes: 4, Type: sim.RoadAvenue4Lane}}}
	w.RebuildIntersections()
	w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", sim.Vec2{X: -2200}, 0, 8, sim.RGBA(0, 0, 0, 255), lw), sim.NewBusStop(1, "to", sim.Vec2{X: 2200}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)}
	return w
}

func TestBusLanePreferenceAndPassing(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, scenario := range []string{"empty", "passing", "return", "shorter queue", "general queue", "unsafe return"} {
			t.Run(fmt.Sprintf("berserk%v/%s", berserk, scenario), func(t *testing.T) {
				const lw = 52.0
				w := busLaneTestWorld(lw)
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
				bus := sim.NewBus(-1300, 3.5*lw, 0)
				bus.Speed = 100
				var obstacles []VehicleObstacle
				initial := 0.0
				if scenario == "return" || scenario == "unsafe return" {
					initial = -lw
					bus.Pos.Y -= lw
					ad.LaneOffset = initial
					ad.TargetLaneOffset = initial
				}
				if scenario == "passing" {
					obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1050, Y: 3.5 * lw}, Heading: 0, Speed: 25, Length: 48, Width: 22}}
				}
				if scenario == "shorter queue" {
					obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -700, Y: 3.5 * lw}, Heading: 0, Speed: 0, Length: 48, Width: 22}, {Pos: sim.Vec2{X: -800, Y: 3.5 * lw}, Heading: 0, Speed: 0, Length: 48, Width: 22}}
				}
				if scenario == "general queue" {
					obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -400, Y: 2.5 * lw}, Heading: 0, Speed: 0, Length: 48, Width: 22}}
				}
				if scenario == "unsafe return" {
					obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -1330, Y: 3.5 * lw}, Heading: 0, Speed: 120, Length: 96, Width: 36}}
				}
				ad.trackPath(bus.Pos)
				var tls *sim.TrafficLightSystem
				if scenario == "shorter queue" || scenario == "general queue" {
					tls = sim.NewTrafficLightSystem()
					tls.BuildForWorld(w)
					for _, it := range tls.Intersections {
						it.Phase = sim.PhaseNSGreen
					}
				}
				frame := autopilotFrame{bus: bus, world: w, tls: tls, obstacles: obstacles, laneWidth: lw, s: ad.LastPathS, speedAbs: 100, remaining: 3000, targetStop: w.Stops[1]}
				if !ad.updateBusLane(frame, .016) {
					t.Fatal("bus-lane policy not applied")
				}
				want := 0.0
				if berserk && (scenario == "passing" || scenario == "shorter queue") {
					want = -lw
				}
				if scenario == "unsafe return" {
					want = initial
				}
				if ad.TargetLaneOffset != want {
					t.Fatalf("target%.1f want%.1f", ad.TargetLaneOffset, want)
				}
			})
		}
	}
}

func TestBusLaneLeftTurnUsesShorterLegalQueue(t *testing.T) {
	for _, queued := range []bool{false, true} {
		t.Run(fmt.Sprint(queued), func(t *testing.T) {
			const lw = 52.0
			w := busLaneTestWorld(lw)
			w.Stops[1] = sim.NewBusStop(1, "left", sim.Vec2{Y: -2200}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			bus := sim.NewBus(-1250, 3.5*lw, 0)
			bus.Speed = 60
			var obstacles []VehicleObstacle
			if queued {
				obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: -650, Y: 3.5 * lw}, Heading: 0, Length: 96, Width: 36}}
			}
			ad.trackPath(bus.Pos)
			f := autopilotFrame{bus: bus, world: w, tls: tls, obstacles: obstacles, laneWidth: lw, s: ad.LastPathS, speedAbs: 60, remaining: 3000, targetStop: w.Stops[1]}
			ad.updateBusLane(f, .016)
			if ad.ordinaryBusLeft != queued {
				t.Fatalf("ordinary-left choice %v for bus queue%v", ad.ordinaryBusLeft, queued)
			}
		})
	}
}

func TestBusLaneLeftTurnWaitsForItsArrow(t *testing.T) {
	const lw = 52.0
	w := busLaneTestWorld(lw)
	w.Stops[1] = sim.NewBusStop(1, "left", sim.Vec2{Y: -2200}, -math.Pi/2, 4, sim.RGBA(0, 0, 0, 255), lw)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	tls.Intersections[0].Phase = sim.PhaseEWGreen
	line := tls.Intersections[0].GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-150, 3.5*lw, 0)
	bus.Speed = 80
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	for step := 0; step < 300; step++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, .016)
		bus.Update(.016, th, br, st, hb, doors)
		if bus.Pos.X+bus.Length/2 > -line+1 {
			t.Fatalf("bus lane left turn used a through green: pos%v", bus.Pos)
		}
	}
	if bus.Speed > 2 {
		t.Fatal("bus did not wait for its left arrow")
	}
}
