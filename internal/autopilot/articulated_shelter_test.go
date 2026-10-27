package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// The free-bay route shortcut used to bypass final alignment and accelerate
// a still-yawed articulated bus into the shelter. Exercise the real actuator,
// both sections, and arrival completion in each road direction.
func TestArticulatedFreeBayAlignsBeforeShelter(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, berserk := range []bool{false, true} {
			t.Run(fmt.Sprintf("h%.2f/berserk%v", h, berserk), func(t *testing.T) {
				u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 6, Type: sim.RoadExpressway6Lane}}}
				w.RebuildIntersections()
				w.Stops = []*sim.BusStop{sim.NewBusStop(0, "from", u.Mul(-1500), h, 6, sim.RGBA(0, 0, 0, 255), 52), sim.NewBusStop(1, "to", sim.Vec2{}, h, 6, sim.RGBA(0, 0, 0, 255), 52)}
				stop := w.Stops[1]
				p := stop.Pos.Sub(u.Mul(180)).Sub(r.Mul(17))
				b := sim.NewBus(p.X, p.Y, h+.30)
				b.SetModel(sim.BusArticulated)
				b.Speed = 90
				b.SteeringAngle = -.15
				b.TrailerHeading = h + .56
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				ad.articulatedPath = true
				ad.PrepareLeg(w, w.Stops[0], stop, 52)
				ad.drivePath[len(ad.drivePath)-1] = stop.FrontBerthPosition()
				f := autopilotFrame{bus: b, world: w, targetStop: stop, laneWidth: 52, speedAbs: 90, remaining: 276}
				if _, active := ad.stationDockControls(f); !active {
					t.Fatal("free-bay shortcut bypassed articulated alignment")
				}
				docked := false
				for tick := 0; tick < 1800; tick++ {
					th, br, st, hb, d, dock := ad.ComputeControls(b, route, w, nil, nil, 52, 1./60)
					b.Update(1./60, th, br, st, hb, d)
					if result := b.CheckBoundaries(w, 52); result.Zone == sim.ZoneHardBarrier {
						t.Fatalf("barrier tick%d %s pose%v", tick, result.ObstacleName, b.Pos)
					}
					if dock {
						docked = true
						break
					}
				}
				if !docked {
					t.Fatalf("failed to dock pose%v heading%.3f", b.Pos, b.Heading)
				}
			})
		}
	}
}
