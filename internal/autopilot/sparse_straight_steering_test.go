package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

func TestSparseStraightDoesNotBorrowDistantCornerLookahead(t *testing.T) {
	for _, berserk := range []bool{false, true} {
		for _, state := range []int{OvertakeNone, OvertakeChangingLane} {
			b := sim.NewBus(150, 78, math.Pi)
			b.Speed = 440
			ad := NewAutopilotDriver()
			ad.BerserkMode = berserk
			ad.OvertakeState = state
			ad.drivePath = []sim.Vec2{{X: 2000, Y: 78}, {X: 0, Y: 78}, {X: -3400, Y: 75}, {X: -3480, Y: 65}, {X: -3560, Y: 40}, {X: -3640, Y: 0}, {X: -3680, Y: -80}, {X: -3680, Y: -1000}}
			ad.cumLen = make([]float64, len(ad.drivePath))
			for i := 1; i < len(ad.drivePath); i++ {
				ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
			}
			ad.trackPath(b.Pos)
			f := autopilotFrame{bus: b, s: ad.LastPathS, speedAbs: b.Speed, laneWidth: 52}
			_, _, target, _ := ad.pursuitSteering(f)
			if b.Pos.Distance(target) < 85 {
				t.Fatalf("distant corner shortened current straight pursuit: berserk%v target%v", berserk, target)
			}
		}
	}
}
