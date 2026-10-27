package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func laneBusYieldLimit(f carFrame, bus, previous *sim.Bus) (float64, bool) {
	request := bus.LaneMergeRequest
	if request == nil || request.Remaining <= 0 || request.Joined || math.Cos(f.car.Heading-request.Heading) < .98 || math.Cos(bus.Heading-request.Heading) < .5 {
		return 0, false
	}
	forward := sim.UnitFromAngle(request.Heading)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	if math.Abs(f.car.Pos.Sub(request.Target).Dot(right)) > f.laneWidth*.40 {
		return 0, false
	}
	yaw := bus.Heading - request.Heading
	body := bus.BodyEnvelope()
	extent := body.HalfLen*math.Abs(math.Cos(yaw)) + body.HalfWid*math.Abs(math.Sin(yaw))
	gap := body.Center.Sub(f.car.Pos).Dot(forward) - extent - f.car.Length/2 - 30
	minGap := -8.0
	if previous == bus {
		minGap = -30.0
	}
	if gap < minGap || gap > 750 {
		return 0, false
	}
	closingSpeed := math.Max(0, f.car.Speed-bus.Speed*math.Cos(bus.Heading-request.Heading))
	braking := closingSpeed*closingSpeed/(2*220) + closingSpeed*.2
	if previous != bus && f.car.Speed > 35.0 && math.Max(0, gap) < braking {
		return 0, false
	}
	// Reserve only the merge's rear clearance. Distant cars approach that
	// boundary instead of stopping an entire queue where the request began.
	if gap <= 12 {
		return 0, true
	}
	return math.Sqrt(2 * 220 * math.Max(0, gap-12)), true
}
