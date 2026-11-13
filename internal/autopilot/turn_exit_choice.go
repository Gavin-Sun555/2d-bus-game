package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Select the outgoing lane before admission checks reserve the turning curve.
// A queued curb bus lane must not reserve an otherwise empty carriageway.
// Keep the selected curve through the turn; ordinary exit cruising retains it.
func (ad *AutopilotDriver) chooseLeftTurnExit(f autopilotFrame, center sim.Vec2, heading float64) {
	if ad.exitCruise != nil && ad.exitCruise.Junction.Distance(center) < 1 {
		return
	}
	in := sim.UnitFromAngle(heading)
	out := sim.Vec2{X: in.Y, Y: -in.X}
	rIn, rOut := sim.Vec2{X: -in.Y, Y: in.X}, sim.Vec2{X: -out.Y, Y: out.X}
	g := f.world.JunctionAt(center, f.laneWidth)
	entryHalf := math.Abs(in.X)*g.HalfX + math.Abs(in.Y)*g.HalfY
	exitHalf := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
	if center.Sub(f.bus.Pos).Dot(in) < entryHalf+f.bus.Length/2 {
		return
	}
	probe := center.Add(out.Mul(exitHalf + 150))
	coord, ew := center.Y, math.Abs(out.X) > .5
	if !ew {
		coord = center.X
	}
	road, ok := f.world.RoadAt(coord, ew, probe)
	if !ok || !road.HasBusLane() || road.HasMedianBusLane() {
		return
	}
	entry, exit := -1, -1
	for i := max(1, ad.WaypointIdx); i < len(ad.drivePath); i++ {
		d := ad.drivePath[i].Sub(center)
		if entry < 0 && math.Abs(d.Dot(in)+entryHalf) < 2 && d.Dot(rIn) > 0 {
			entry = i
		}
		if entry >= 0 && d.Dot(out) >= exitHalf-1 && ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize().Dot(out) > .98 {
			exit = i
			break
		}
	}
	if entry < 0 || exit <= entry || exit+1 >= len(ad.drivePath) {
		return
	}
	current := int(math.Round(ad.drivePath[exit].Sub(center).Dot(rOut)/f.laneWidth - .5))
	if current < 0 || current >= road.LaneCount()/2 {
		return
	}
	// Count only genuinely slow traffic ahead of the outgoing mouth. Physical
	// traffic inside the junction is still checked by admission and swept bodies.
	score := func(lane int) float64 {
		value := 0.
		for _, ob := range f.obstacles {
			d := ob.Pos.Sub(center)
			if ob.Speed > 20 || math.Cos(ob.Heading-math.Atan2(out.Y, out.X)) < .8 ||
				d.Dot(out) < -f.bus.Length || d.Dot(out) > exitHalf+f.bus.OverallLength()*2+250 {
				continue
			}
			yaw := ob.Heading - math.Atan2(out.Y, out.X)
			half := ob.Length/2*math.Abs(math.Sin(yaw)) + ob.Width/2*math.Abs(math.Cos(yaw))
			if math.Abs(d.Dot(rOut)-(float64(lane)+.5)*f.laneWidth) < half+f.bus.Width/2+3 {
				value++
			}
		}
		return value
	}
	best := score(current)
	if best == 0 {
		return
	}
	lane := current
	inOffset := ad.drivePath[entry].Sub(center).Dot(rIn)
	var chosen []sim.Vec2
	for candidate := road.LaneCount()/2 - 1; candidate >= 1; candidate-- {
		s := score(candidate)
		if s >= best {
			continue
		}
		curve := sim.WideLeftTurnPath(center, in, out, inOffset, (float64(candidate)+.5)*f.laneWidth, g)
		if turnExitCurveClear(f, curve, out) {
			lane, best = candidate, s
			chosen = curve
			if best == 0 {
				break
			}
		}
	}
	if lane == current || best > 0 {
		return
	}
	outOffset := (float64(lane) + .5) * f.laneWidth
	// Skip any cached short run-out already replaced by the new curve.
	tail := exit + 1
	for tail < len(ad.drivePath) && ad.drivePath[tail].Sub(center).Dot(out) <= exitHalf+100 {
		tail++
	}
	// Keep the whole exit reservation in the chosen lane. Reattaching the
	// old curb path immediately after the mouth advertises a second merge
	// into the blocked bus lane before exit cruising can take over.
	runout := math.Max(300, f.bus.OverallLength()*2)
	if tail < len(ad.drivePath) {
		runout = math.Min(runout, math.Max(100, ad.drivePath[tail].Sub(center).Dot(out)-exitHalf-80))
	}
	chosen[len(chosen)-1] = center.Add(out.Mul(exitHalf + runout)).Add(rOut.Mul(outOffset))
	points := append([]sim.Vec2(nil), ad.drivePath[:entry]...)
	points = append(points, chosen...)
	ad.drivePath = append(points, ad.drivePath[tail:]...)
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
	ad.exitCruise = &sim.BusLaneMergeRequest{Junction: center, Heading: math.Atan2(out.Y, out.X), Target: center.Add(rOut.Mul(outOffset)), LeftTurn: true}
}

func turnExitCurveClear(f autopilotFrame, curve []sim.Vec2, out sim.Vec2) bool {
	bodies := sim.AppendPathBodies(nil, curve, f.bus.Length+4, f.bus.Width+4)
	for _, ob := range f.obstacles {
		if math.Abs(ob.Speed) > 5 || sim.UnitFromAngle(ob.Heading).Dot(out) < .8 {
			continue
		}
		for _, other := range predictedObstacleBodies(ob, 0) {
			if other.HalfLen <= 0 {
				continue
			}
			for _, body := range bodies {
				if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
					return false
				}
			}
		}
	}
	return true
}

// During a turn, a straight ray from the nose can point at the curb queue
// even though the selected curve joins a separate general lane. Release that
// following ray only after checking the complete remaining physical sweep.
func (ad *AutopilotDriver) clearsTurnExitQueue(f autopilotFrame, ob VehicleObstacle) bool {
	request := ad.exitCruise
	if request == nil || !request.LeftTurn || request.Joined || f.world == nil || math.Abs(ob.Speed) > 5 || len(ad.drivePath) < 2 {
		return false
	}
	out := sim.UnitFromAngle(request.Heading)
	r := sim.Vec2{X: -out.Y, Y: out.X}
	selected := request.Target.Sub(request.Junction).Dot(r)
	if selected < f.laneWidth || selected > 3*f.laneWidth || sim.UnitFromAngle(ob.Heading).Dot(out) < .98 ||
		math.Abs(ob.Pos.Sub(request.Target).Dot(r)) < (f.bus.Width+ob.Width)/2+8 || ob.Pos.Distance(f.bus.Pos) > 300 {
		return false
	}
	pointAt := func(distance float64) (sim.Vec2, float64) {
		p := ad.pointAt(f.s + distance)
		u := ad.pointAt(f.s + distance + 2).Sub(p).Normalize()
		return p, math.Atan2(u.Y, u.X)
	}
	base, angle := pointAt(0)
	return junctionActualSweepClear(f.bus, ob, pointAt, f.bus.Pos.Sub(base), sim.NormalizeAngle(f.bus.Heading-angle), math.Max(140, f.speedAbs*f.speedAbs/(2*180)+40))
}
