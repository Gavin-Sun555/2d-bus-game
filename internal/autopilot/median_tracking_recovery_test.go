package autopilot

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestMedianPassingRecoversPhysicalTrackingBeforeCruise(t *testing.T) {
	const lw = 52.
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, error := range []float64{0, 112} {
			t.Run(fmt.Sprintf("h%.2f/error%.0f", h, error), func(t *testing.T) {
				u := sim.UnitFromAngle(h)
				r := sim.Vec2{X: -u.Y, Y: u.X}
				w := medianRightTestWorld(lw, h)
				ad := NewAutopilotDriver()
				ad.BerserkMode = true
				ad.OvertakeState = OvertakePassing
				ad.LaneOffset = lw
				ad.TargetLaneOffset = lw
				ad.drivePath = []sim.Vec2{u.Mul(-3000).Add(r.Mul(lw * .5)), u.Mul(3000).Add(r.Mul(lw * .5))}
				ad.cumLen = []float64{0, 6000}
				b := sim.NewBus(0, 0, h)
				b.Pos = u.Mul(-1600).Add(r.Mul(1.5*lw + error))
				b.Speed = 667
				s, e := ad.trackPath(ad.basePathPosition(b.Pos, w, h, lw))
				limit := ad.recoverySpeed(autopilotFrame{bus: b, world: w, laneWidth: lw, s: s, speedAbs: b.Speed, lateralErr: e}, 667, false)
				if error == 0 && limit != 667 {
					t.Fatal("aligned passing bus lost cruise speed", limit)
				}
				if error > 0 && (limit <= 0 || limit > 150) {
					t.Fatal("off-target passing bus retained high-speed cruise", limit)
				}
			})
		}
	}
}
