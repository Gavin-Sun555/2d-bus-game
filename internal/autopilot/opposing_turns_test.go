package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func turnTestWorld(ewLanes, nsLanes int, heading, laneWidth float64) (*sim.World, []*sim.BusStop) {
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -2200}, P2: sim.Vec2{X: 2200}, Lanes: ewLanes},
		{P1: sim.Vec2{Y: -2200}, P2: sim.Vec2{Y: 2200}, Lanes: nsLanes},
	}}
	w.RebuildIntersections()
	var stops []*sim.BusStop
	for i := 0; i < 2; i++ {
		h := heading + float64(i)*math.Pi
		incoming := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
		outgoing := sim.Vec2{X: math.Cos(h - math.Pi/2), Y: math.Sin(h - math.Pi/2)}
		inLanes, outLanes := ewLanes, nsLanes
		if math.Abs(incoming.Y) > .5 {
			inLanes, outLanes = outLanes, inLanes
		}
		stops = append(stops, sim.NewBusStop(i*2, "entry", incoming.Mul(-1400), h, inLanes, sim.RGBA(0, 0, 0, 255), laneWidth, 0), sim.NewBusStop(i*2+1, "exit", outgoing.Mul(1400), h-math.Pi/2, outLanes, sim.RGBA(0, 0, 0, 255), laneWidth, 0))
	}
	w.Stops = stops
	return w, stops
}

// Run the physical controllers together, without obstacle braking masking a bad
// path. Timing offsets exercise a turning bus against an approaching opponent.
func TestOpposingLeftTurnsPhysicalClearance(t *testing.T) {
	for _, laneWidth := range []float64{42, 52, 78} {
		for _, ew := range []int{2, 4, 6} {
			for _, ns := range []int{2, 4, 6} {
				for _, h := range []float64{0, math.Pi / 2} {
					for _, delay := range []int{0, 60} {
						t.Run(fmt.Sprintf("width%.0f/%dx%d/heading%.2f/delay%d", laneWidth, ew, ns, h, delay), func(t *testing.T) {
							w, stops := turnTestWorld(ew, ns, h, laneWidth)
							var buses [2]*sim.Bus
							var drivers [2]*AutopilotDriver
							var routes [2]*sim.Route
							var reached [2]bool
							for i := 0; i < 2; i++ {
								buses[i] = sim.NewBus(stops[i*2].Pos.X, stops[i*2].Pos.Y, stops[i*2].Heading)
								drivers[i] = NewAutopilotDriver()
								drivers[i].BerserkMode = true
								routes[i] = &sim.Route{StopIDs: []int{i * 2, i*2 + 1}, CurrentIndex: 1}
							}
							for step := 0; step < 2400; step++ {
								for i := 0; i < 2; i++ {
									if i == 1 && step < delay {
										continue
									}
									b := buses[i]
									th, br, st, hb, doors, docked := drivers[i].ComputeControls(b, routes[i], w, nil, nil, laneWidth, .016)
									b.Update(.016, th, br, st, hb, doors)
									reached[i] = reached[i] || docked
								}
								a, b := buses[0], buses[1]
								aa := sim.OBB{Center: a.Pos, Heading: a.Heading, HalfLen: a.Length/2 + 2, HalfWid: a.Width/2 + 2}
								bb := sim.OBB{Center: b.Pos, Heading: b.Heading, HalfLen: b.Length/2 + 2, HalfWid: b.Width/2 + 2}
								if hit, _, _ := sim.CheckOBBCollision(aa, bb); hit {
									t.Fatalf("step %d: opposing buses collide at %v / %v, headings %.2f / %.2f", step, a.Pos, b.Pos, a.Heading, b.Heading)
								}
								if reached[0] && reached[1] {
									return
								}
							}
							t.Fatalf("both buses must complete the turn and dock: %v at %v / %v", reached, buses[0].Pos, buses[1].Pos)
						})
					}
				}
			}
		}
	}
}
