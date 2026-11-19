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
	// Skip any cached short run-out already replaced by the new curve and any
	// intermediate pre-planned taper so the exit stays cleanly in the chosen lane.
	runout := math.Max(300, f.bus.OverallLength()*2)
	tail := turnExitTailIndex(ad.drivePath, exit+1, center, out, exitHalf+100)
	// Keep the whole exit reservation in the chosen lane. Reattaching the
	// old curb path immediately after the mouth advertises a second merge
	// into the blocked bus lane before exit cruising can take over.
	if tail < len(ad.drivePath) {
		runout = math.Min(runout, math.Max(100, ad.drivePath[tail].Sub(center).Dot(out)-exitHalf-80))
	}
	ad.spliceLeftTurnExit(entry, tail, chosen, center, out, rOut, exitHalf, runout, outOffset)
	ad.exitCruise = &sim.BusLaneMergeRequest{Junction: center, Heading: math.Atan2(out.Y, out.X), Target: center.Add(rOut.Mul(outOffset)), LeftTurn: true}
}

// Switch an inner-lane left turn onto a non-overlapping inner curve and exit
// lane whenever an outer left-turner on the same approach would otherwise
// overlap its wide diagonal path across the intersection.
func (ad *AutopilotDriver) chooseParallelLeftTurnExit(f autopilotFrame) {
	if f.world == nil || f.tls == nil || len(ad.drivePath) < 2 || ad.uTurn != nil || f.nearRoundabout ||
		ad.busJunctionEntered || ad.fallbackJunction != nil {
		return
	}
	approach := ad.approachingIntersection(f, true, true)
	inter := approach.approachingInter
	if inter == nil || !approach.isUpcomingLeft || f.world.IsAtRoundabout(inter.Center) {
		return
	}
	center := inter.Center
	heading := approachRoadHeading(f.bus.Heading)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .3 {
		return
	}
	in := sim.UnitFromAngle(heading)
	out := sim.Vec2{X: in.Y, Y: -in.X}
	rIn, rOut := sim.Vec2{X: -in.Y, Y: in.X}, sim.Vec2{X: -out.Y, Y: out.X}
	outHeading := math.Atan2(out.Y, out.X)
	stopDist := inter.GetStopLineDistForHeading(heading, f.laneWidth)
	if _, isUTurn := ad.uTurnMovement(center, in, stopDist); isUTurn {
		return
	}
	g := f.world.JunctionAt(center, f.laneWidth)
	entryHalf := math.Abs(in.X)*g.HalfX + math.Abs(in.Y)*g.HalfY
	exitHalf := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
	if center.Sub(f.bus.Pos).Dot(in) < entryHalf+f.bus.Length/2-2 {
		return
	}
	inCoord, inEW, _, _ := sim.CurrentRoadInfo(f.world, f.bus.Pos, heading, f.laneWidth)
	inRoad, inOK := f.world.RoadAt(inCoord, inEW, f.bus.Pos)
	if !inOK || inRoad.LaneCount() < 4 || inRoad.HasMedianBusLane() {
		return
	}
	probe := center.Add(out.Mul(exitHalf + 150))
	outCoord, outEW := center.Y, math.Abs(out.X) > .5
	if !outEW {
		outCoord = center.X
	}
	exitRoad, outOK := f.world.RoadAt(outCoord, outEW, probe)
	if !outOK || exitRoad.LaneCount() < 4 || exitRoad.HasMedianBusLane() {
		return
	}
	entry, exit := -1, -1
	for i := max(1, ad.WaypointIdx); i < len(ad.drivePath); i++ {
		d := ad.drivePath[i].Sub(center)
		if entry < 0 {
			if math.Abs(d.Dot(in)+entryHalf) < 2 && d.Dot(rIn) > 0 {
				entry = i
			} else if i > 0 {
				prevD := ad.drivePath[i-1].Sub(center)
				if prevD.Dot(in) < -entryHalf && d.Dot(in) >= -entryHalf && d.Dot(rIn) > 0 {
					entry = i
				}
			}
		}
		if entry >= 0 && d.Dot(out) >= exitHalf-1 && ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize().Dot(out) > .98 {
			exit = i
			break
		}
	}
	if entry < 0 || exit <= entry {
		return
	}
	inOffset := ad.drivePath[entry].Sub(center).Dot(rIn)
	if inOffset <= 0 || inOffset > f.laneWidth || math.Abs(f.bus.Pos.Sub(center).Dot(rIn)-inOffset) > f.laneWidth*.6 {
		return
	}
	currentOutOffset := ad.drivePath[exit].Sub(center).Dot(rOut)
	if ad.exitCruise != nil && ad.exitCruise.Junction.Distance(center) < 1 &&
		ad.exitCruise.Target.Sub(center).Dot(rOut) <= .75*f.laneWidth {
		return
	}

	if ad.turnExitGeometry == nil {
		ad.turnExitGeometry = &turnExitGeometryScratch{}
	}
	geometry := ad.turnExitGeometry
	geometry.outer = geometry.outer[:0]
	var outerCorridors [][]sim.OBB
	for _, ob := range f.obstacles {
		if ob.JunctionCorridor != nil && ob.JunctionCorridor.Remaining > 0 &&
			ob.JunctionCorridor.Center.Distance(center) < 1 && len(ob.JunctionCorridor.Bodies) > 0 &&
			math.Cos(ob.JunctionCorridor.EntryHeading-heading) > .98 {
			otherExit := ob.JunctionCorridor.Bodies[len(ob.JunctionCorridor.Bodies)-1].Heading
			otherIn := math.Max(ob.JunctionCorridor.Bodies[0].Center.Sub(center).Dot(rIn), ob.Pos.Sub(center).Dot(rIn))
			if math.Cos(otherExit-outHeading) > .95 && otherIn > inOffset+.25*f.laneWidth {
				outerCorridors = append(outerCorridors, ob.JunctionCorridor.Bodies)
			}
		}
		if ob.JunctionTurn != nil && !ob.JunctionTurn.LaneChange &&
			ob.JunctionTurn.Center.Distance(center) < 1 && len(ob.JunctionTurn.Points) > ob.JunctionTurn.Segment &&
			math.Cos(ob.JunctionTurn.EntryHeading-heading) > .98 &&
			math.Cos(ob.JunctionTurn.ExitHeading-outHeading) > .95 &&
			ob.Pos.Sub(center).Dot(rIn) > inOffset+.25*f.laneWidth {
			pts := ob.JunctionTurn.Points[ob.JunctionTurn.Segment:]
			start := len(geometry.outer)
			geometry.outer = sim.AppendPathBodies(geometry.outer, pts, ob.Length, ob.Width)
			outerCorridors = append(outerCorridors, geometry.outer[start:len(geometry.outer):len(geometry.outer)])
		}
	}
	if len(outerCorridors) == 0 {
		return
	}

	corridorClear := func(bodies []sim.OBB) bool {
		for _, other := range outerCorridors {
			if !sim.CorridorsSeparated(bodies, other) {
				return false
			}
		}
		return true
	}
	extendedBodies := func(curve []sim.Vec2, outOff float64) []sim.OBB {
		end := center.Add(out.Mul(exitHalf + f.bus.Length + 80)).Add(rOut.Mul(outOff))
		return geometry.extendedBodies(curve, end, f.bus.Length+8, f.bus.Width+6)
	}

	// Skip if the current left-turn corridor already clears all outer left-turners.
	currentBodies := geometry.current.build(ad.drivePath[entry:exit+1], f.bus.Length+8, f.bus.Width+6)
	if len(currentBodies) > 0 && currentOutOffset <= 1.6*f.laneWidth && corridorClear(currentBodies) {
		return
	}
	if len(currentBodies) > 0 && f.bus.JunctionCorridor != nil && f.bus.JunctionCorridor.Center.Distance(center) < 1 &&
		corridorClear(f.bus.JunctionCorridor.Bodies) && currentOutOffset <= 1.6*f.laneWidth {
		return
	}

	incoming, outgoing := inRoad.LaneCount(), exitRoad.LaneCount()
	type candidate struct {
		outOff float64
		curve  []sim.Vec2
	}
	var candidates []candidate
	candidates = append(candidates, candidate{
		outOff: .5 * f.laneWidth,
		curve:  sim.InnerLeftTurnPath(center, in, out, inOffset, .5*f.laneWidth, g, incoming, outgoing),
	})
	if outgoing >= 8 {
		candidates = append(candidates,
			candidate{
				outOff: 1.5 * f.laneWidth,
				curve:  sim.InnerLeftTurnPath(center, in, out, inOffset, 1.5*f.laneWidth, g, incoming, outgoing),
			},
			candidate{
				outOff: 1.5 * f.laneWidth,
				curve:  sim.LeftTurnPathWithHandle(center, in, out, inOffset, 1.5*f.laneWidth, g, .46),
			},
		)
	}
	candidates = append(candidates, candidate{
		outOff: .5 * f.laneWidth,
		curve:  sim.LeftTurnPathWithHandle(center, in, out, inOffset, .5*f.laneWidth, g, .44),
	})

	var chosen []sim.Vec2
	chosenOut := 0.0
	for _, cand := range candidates {
		if cand.outOff >= currentOutOffset-1 && math.Abs(cand.outOff-currentOutOffset) > 1 && len(cand.curve) > 0 {
			continue
		}
		if !turnExitCurveClear(f, cand.curve, out) {
			continue
		}
		if corridorClear(extendedBodies(cand.curve, cand.outOff)) {
			chosen = cand.curve
			chosenOut = cand.outOff
			break
		}
	}
	if len(chosen) == 0 {
		return
	}

	runout := math.Max(190, f.bus.Length+80)
	tail := turnExitTailIndex(ad.drivePath, exit+1, center, out, exitHalf+runout+140)
	if tail < len(ad.drivePath) {
		avail := ad.drivePath[tail].Sub(center).Dot(out) - exitHalf - 20
		if avail < runout {
			runout = math.Max(140, avail*0.65)
		}
	}
	ad.spliceLeftTurnExit(entry, tail, chosen, center, out, rOut, exitHalf, runout, chosenOut)
	ad.pathSynced = false
	ad.trackPath(f.bus.Pos)
	ad.exitCruise = &sim.BusLaneMergeRequest{Junction: center, Heading: outHeading, Target: center.Add(rOut.Mul(chosenOut)), LeftTurn: true}
}

// spliceLeftTurnExit installs a chosen left-turn exit curve and avoids
// connecting its run-out across kilometers of collinear road diagonally back to
// a distant curb-lane waypoint.
func (ad *AutopilotDriver) spliceLeftTurnExit(entry, tail int, chosen []sim.Vec2, center, out, rOut sim.Vec2, exitHalf, runout, outOffset float64) {
	chosen[len(chosen)-1] = center.Add(out.Mul(exitHalf + runout)).Add(rOut.Mul(outOffset))
	points := append([]sim.Vec2(nil), ad.drivePath[:entry]...)
	points = append(points, chosen...)
	if tail < len(ad.drivePath) {
		tailRel := ad.drivePath[tail].Sub(center)
		tailAlong := tailRel.Dot(out)
		tailOffset := tailRel.Dot(rOut)
		chosenEndAlong := exitHalf + runout
		if latDelta := math.Abs(tailOffset - outOffset); latDelta > 4 {
			taperLen := math.Max(220, latDelta*3.5)
			if tailAlong-chosenEndAlong > taperLen+40 {
				points = append(points, center.Add(out.Mul(chosenEndAlong+taperLen)).Add(rOut.Mul(tailOffset)))
			}
		}
	}
	ad.drivePath = append(points, ad.drivePath[tail:]...)
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}
}

// turnExitTailIndex advances past any cached run-out covered by the new turn
// curve and any pre-planned outgoing lane-change taper on the same road arm,
// while stopping before a downstream intersection curve or road end.
func turnExitTailIndex(path []sim.Vec2, start int, center, out sim.Vec2, minAlong float64) int {
	tail := start
	for tail < len(path)-1 {
		pt := path[tail]
		seg := path[tail+1].Sub(pt)
		segLen := seg.Length()
		if segLen < 1e-6 {
			tail++
			continue
		}
		segDot := seg.Mul(1 / segLen).Dot(out)
		if segDot <= .70 {
			break
		}
		if pt.Sub(center).Dot(out) < minAlong {
			tail++
			continue
		}
		if segDot > .992 {
			break
		}
		// A gentler angle (0.70..0.992) is either an intermediate lane taper
		// returning parallel to out, or the start of the next turn curve.
		taperEnd := -1
		for k := tail + 1; k < len(path)-1; k++ {
			kSeg := path[k+1].Sub(path[k])
			kLen := kSeg.Length()
			if kLen < 1e-6 {
				continue
			}
			kDot := kSeg.Mul(1 / kLen).Dot(out)
			if kDot <= .70 || path[k].Sub(pt).Dot(out) > 900 {
				break
			}
			if kDot > .992 {
				taperEnd = k
				break
			}
		}
		if taperEnd < 0 {
			break
		}
		tail = taperEnd
		break
	}
	return tail
}

func turnExitCurveClear(f autopilotFrame, curve []sim.Vec2, out sim.Vec2) bool {
	var storage [64]sim.OBB
	bodies := sim.AppendPathBodies(storage[:0], curve, f.bus.Length+4, f.bus.Width+4)
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
