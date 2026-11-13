package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestStopDemandPathRebuildPreservesPhysicalLane(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, lateral := range []float64{26, 78, 130, 182} {
			u := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -u.Y, Y: u.X}
			w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-1000), P2: u.Mul(6000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
			for id, d := range []float64{0, 3000} {
				w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", u.Mul(d), heading, 8, sim.RGBA(0, 0, 0, 255), 52))
			}
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			ad.CanSkipTargetStop = true
			ad.skippedDeparture = &skippedStopDeparture{stop: w.Stops[0], position: u.Mul(100).Add(right.Mul(203))}
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], 52)
			// The saved road-only skip follows the middle lane, while the original
			// skipped-stop departure was near the curb. They have different origins.
			ad.drivePath = []sim.Vec2{u.Mul(500).Add(right.Mul(lateral)), u.Mul(3000).Add(right.Mul(lateral))}
			ad.cumLen = []float64{0, 2500}
			ad.LastPathS = 500
			ad.pathSynced = true
			ad.holdLaneOffset(0)
			actual := u.Mul(1000).Add(right.Mul(lateral))
			for _, skip := range []bool{false, true, false} {
				ad.CanSkipTargetStop = skip
				ad.PrepareLeg(w, w.Stops[0], w.Stops[1], 52)
				s, _ := ad.trackPath(actual)
				corridor := ad.pointAt(s).Add(right.Mul(ad.LaneOffset))
				if corridor.Distance(actual) > 1e-6 {
					t.Fatalf("heading %.2f skip %v moved corridor by %.1f", heading, skip, corridor.Distance(actual))
				}
				if math.Abs(ad.TargetLaneOffset-ad.LaneOffset) > 1e-6 {
					t.Fatal("path rebuild injected a lane change")
				}
			}
		}
	}
}
