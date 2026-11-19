package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func (ad *AutopilotDriver) fallbackActiveAt(center sim.Vec2) bool {
	return ad != nil && ad.fallbackJunction != nil && ad.fallbackJunction.Distance(center) < 1
}

func (ad *AutopilotDriver) clearPassedFallbackJunction(f autopilotFrame) {
	if ad.fallbackJunction == nil {
		return
	}
	if f.world == nil || len(ad.drivePath) < 2 {
		ad.fallbackJunction = nil
		return
	}
	center := *ad.fallbackJunction
	g := f.world.JunctionAt(center, f.laneWidth)
	for _, part := range f.bus.BodyParts() {
		if part.HalfLen > 0 && f.world.InsideJunction(part.Center, f.laneWidth, part.HalfLen) {
			return
		}
	}
	u := sim.UnitFromAngle(approachRoadHeading(f.bus.Heading))
	pastBox := f.bus.Pos.Sub(center).Dot(u) > 0
	far := f.bus.Pos.Distance(center) > math.Max(g.HalfX, g.HalfY)+f.bus.OverallLength()+120
	if pastBox || far {
		ad.fallbackJunction = nil
	}
}

// When a pre-intersection lane change still cannot complete near the junction,
// continue through the intersection from the current physical lane along the
// route's intended exit while still obeying that movement's traffic signal.
func (ad *AutopilotDriver) checkCurrentLaneJunctionFallback(f autopilotFrame, dt float64) bool {
	if f.world == nil || f.tls == nil || len(ad.drivePath) < 2 || ad.busJunctionEntered ||
		ad.uTurn != nil || f.nearRoundabout || f.bus.DoorsOpen ||
		(ad.State != AutopilotStateCruising && ad.State != AutopilotStateApproaching) {
		ad.blockedTurnMergeTimer = 0
		return false
	}

	coord, ew, multi, six := sim.CurrentRoadInfo(f.world, f.bus.Pos, f.bus.Heading, f.laneWidth)
	if !multi && !six {
		ad.blockedTurnMergeTimer = 0
		return false
	}
	heading := approachRoadHeading(f.bus.Heading)
	if math.Abs(sim.NormalizeAngle(f.bus.Heading-heading)) > .35 {
		ad.blockedTurnMergeTimer = 0
		return false
	}
	road, ok := f.world.RoadAt(coord, ew, f.bus.Pos)
	lanes := road.LaneCount()
	if !ok || lanes < 4 {
		ad.blockedTurnMergeTimer = 0
		return false
	}

	approach := ad.intersectionApproachWithRunway(f, multi, six, true)
	inter := approach.approachingInter
	if inter == nil || f.world.IsAtRoundabout(inter.Center) {
		ad.blockedTurnMergeTimer = 0
		return false
	}

	u := sim.UnitFromAngle(heading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	g := f.world.JunctionAt(inter.Center, f.laneWidth)
	entryHalf := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
	distToBox := inter.Center.Sub(f.bus.Pos).Dot(u) - entryHalf
	crossHalf := inter.ApproachRoadHalfWidth(heading+math.Pi/2, f.laneWidth)
	distToCross := inter.Center.Sub(f.bus.Pos).Dot(u) - crossHalf
	if approach.distToStopLine < -f.bus.Length*0.4 && distToBox < -f.bus.Length*0.4 &&
		!(ad.rightTurnWaiting && ad.busTurnHoldDistance > 0 && distToCross >= f.bus.Length*0.5) {
		ad.blockedTurnMergeTimer = 0
		return false
	}
	stopDist := inter.GetStopLineDistForHeading(heading, f.laneWidth)
	threshold := math.Max(1.6, float64(lanes)/2+.2) * f.laneWidth
	turn, known := ad.intersectionExitTurn(inter.Center, u.X, u.Y, stopDist, threshold)
	if _, isUTurn := ad.uTurnMovement(inter.Center, u, stopDist); !known || isUTurn {
		ad.blockedTurnMergeTimer = 0
		return false
	}

	halfLanes := lanes / 2
	origin := sim.Vec2{Y: coord}
	if !ew {
		origin = sim.Vec2{X: coord}
	}
	actualLat := f.bus.Pos.Sub(origin).Dot(r)
	if actualLat < 0.1*f.laneWidth || actualLat > (float64(halfLanes)+0.75)*f.laneWidth {
		ad.blockedTurnMergeTimer = 0
		return false
	}

	entryLat, exitIdx, hasEntry := ad.fallbackJunctionPlan(inter.Center, origin, u, r, turn, entryHalf, g)
	if !hasEntry {
		ad.blockedTurnMergeTimer = 0
		return false
	}

	clampedEntry := sim.Clamp(entryLat, 0.25*f.laneWidth, float64(halfLanes)*f.laneWidth-0.25*f.laneWidth)
	effectiveTarget := clampedEntry + ad.LaneOffset
	if ad.rightTurnWaiting {
		effectiveTarget = clampedEntry
	}
	if ad.shortBlockedArticulatedLeftMerge(f) && ad.approachMerge.Junction.Distance(inter.Center) < 1 {
		effectiveTarget = ad.approachMerge.Target.Sub(origin).Dot(r)
	}
	if ad.mergeYieldOrClearingActive(f, heading, u, r, actualLat, effectiveTarget) {
		ad.blockedTurnMergeTimer = 0
		return false
	}

	holdBlocked := ad.busTurnHoldDistance > 0 &&
		approach.distToStopLine <= math.Max(380, f.bus.OverallLength()*1.8) &&
		(f.speedAbs < 20 || ad.busTurnHoldDistance < 65)

	taperBlocked := false
	immediate := false
	if approach.distToStopLine <= math.Max(220, f.bus.OverallLength()+60) {
		curLane := max(0, min(halfLanes-1, int(math.Floor(actualLat/f.laneWidth))))
		targetLane := max(0, min(halfLanes-1, int(math.Floor(clampedEntry/f.laneWidth))))
		latMismatch := curLane != targetLane || math.Abs(effectiveTarget-actualLat) > f.laneWidth*0.38
		laneClear := requiredMergeLaneClear(f, heading, actualLat, effectiveTarget)
		blockedForwardMerge := ad.approachMerge != nil && !ad.approachMergeJoined && f.speedAbs < 5 && ad.requiredMergeTrafficSpeed(f) < 10
		if latMismatch && (!laneClear || blockedForwardMerge) {
			taperBlocked = true
			if !laneClear && approach.distToStopLine <= 25 && f.speedAbs < 5 && (curLane != targetLane || turn != routeTurnStraight) {
				immediate = true
			}
		}
	}

	if !immediate {
		if (holdBlocked || taperBlocked) && f.speedAbs < 20 {
			ad.blockedTurnMergeTimer += math.Max(0, dt)
			if ad.blockedTurnMergeTimer < 2.2 {
				return false
			}
		} else if !holdBlocked && !taperBlocked {
			ad.blockedTurnMergeTimer = 0
			return false
		} else {
			return false
		}
	}

	return ad.spliceCurrentLaneJunctionPath(f, inter.Center, origin, u, r, heading, turn, halfLanes, actualLat, approach.distToStopLine, g, exitIdx)
}

func (ad *AutopilotDriver) shortBlockedArticulatedLeftMerge(f autopilotFrame) bool {
	request := ad.approachMerge
	if request == nil || !request.LeftTurn || request.Is2Lane || request.Joined || !f.bus.IsArticulated() ||
		ad.busTurnHoldDistance <= 0 || ad.busTurnHoldDistance > 12 {
		return false
	}
	right := sim.Vec2{X: -math.Sin(request.Heading), Y: math.Cos(request.Heading)}
	shift := math.Abs(f.bus.Pos.Sub(request.Target).Dot(right))
	return shift > f.laneWidth*.4 && shift < f.laneWidth*.75
}

// mergeYieldOrClearingActive reports whether a vehicle is actively yielding for
// the required merge or moving forward in the target lane to clear the space.
func (ad *AutopilotDriver) mergeYieldOrClearingActive(f autopilotFrame, heading float64, u, r sim.Vec2, actualLat, targetLat float64) bool {
	if f.bus == nil {
		return false
	}
	targetPt := f.bus.Pos.Add(r.Mul(targetLat - actualLat))
	pinnedLeft := ad.shortBlockedArticulatedLeftMerge(f)
	for _, ob := range f.obstacles {
		delta := ob.Pos.Sub(targetPt)
		across := math.Abs(delta.Dot(r))
		if ob.YieldingTo == f.bus {
			if !pinnedLeft {
				return true
			}
			// Cars farther back also acknowledge the request. Their yield
			// does not open a slot beside the bus when its nearest car is
			// pinned against the trailer.
			gap := stoppedFollowerGap(f, ob, heading)
			if across < f.laneWidth*.85 && gap >= 8 && gap <= 35 {
				return true
			}
		}
		if ob.Speed > 5 && math.Cos(ob.Heading-heading) > 0.7 {
			along := delta.Dot(u)
			if along > -ob.Length/2 && along < 260 && across < f.laneWidth*0.85 {
				return true
			}
		}
	}
	return false
}

func (ad *AutopilotDriver) fallbackJunctionPlan(
	center, origin, u, r sim.Vec2,
	turn routeTurn,
	entryHalf float64,
	g sim.JunctionGeometry,
) (entryLat float64, exitIdx int, ok bool) {
	out := u
	if turn == routeTurnLeft {
		out = sim.Vec2{X: u.Y, Y: -u.X}
	} else if turn == routeTurnRight {
		out = r
	}
	exitHalf := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY

	start := max(1, ad.WaypointIdx)
	foundEntry := false
	exitIdx = -1
	for i := start; i < len(ad.drivePath); i++ {
		a := ad.drivePath[i-1].Sub(center)
		b := ad.drivePath[i].Sub(center)
		if !foundEntry {
			fa, fb := a.Dot(u), b.Dot(u)
			if fa <= -entryHalf+2 && fb >= -entryHalf-2 && fb > fa+1e-6 {
				t := sim.Clamp((-entryHalf-fa)/(fb-fa), 0, 1)
				pt := ad.drivePath[i-1].Add(ad.drivePath[i].Sub(ad.drivePath[i-1]).Mul(t))
				entryLat = pt.Sub(origin).Dot(r)
				foundEntry = true
			}
		}
		if b.Dot(out) >= exitHalf-1 {
			seg := ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize()
			nextAligned := i+1 < len(ad.drivePath) && ad.drivePath[i+1].Sub(ad.drivePath[i]).Normalize().Dot(out) > .9
			if seg.Dot(out) > .9 || nextAligned {
				exitIdx = i
				break
			}
		}
	}
	if !foundEntry && ad.LastPathS < float64(len(ad.drivePath)) && len(ad.cumLen) == len(ad.drivePath) {
		entryLat = ad.pointAt(ad.LastPathS).Sub(origin).Dot(r)
		foundEntry = true
	}
	return entryLat, exitIdx, foundEntry && exitIdx >= 1
}

func (ad *AutopilotDriver) spliceCurrentLaneJunctionPath(
	f autopilotFrame,
	center, origin, u, r sim.Vec2,
	heading float64,
	turn routeTurn,
	halfLanes int,
	actualLat, distToStop float64,
	g sim.JunctionGeometry,
	exitIdx int,
) bool {
	out := u
	if turn == routeTurnLeft {
		out = sim.Vec2{X: u.Y, Y: -u.X}
	} else if turn == routeTurnRight {
		out = r
	}
	rOut := sim.Vec2{X: -out.Y, Y: out.X}
	entryHalf := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
	exitHalf := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY

	curLane := max(0, min(halfLanes-1, int(math.Floor(actualLat/f.laneWidth))))
	laneCenter := (float64(curLane) + .5) * f.laneWidth
	clampedActual := sim.Clamp(actualLat, f.bus.Width/2+4, float64(halfLanes)*f.laneWidth-f.bus.Width/2-4)
	if actualLat > float64(halfLanes)*f.laneWidth+4 && !requiredMergeLaneClear(f, heading, actualLat, clampedActual) {
		return false
	}
	inOffset := clampedActual
	if math.Abs(actualLat-laneCenter) <= f.laneWidth*0.2 || (distToStop >= 60 && requiredMergeLaneClear(f, heading, actualLat, laneCenter)) {
		inOffset = laneCenter
	}

	busAlong := f.bus.Pos.Sub(center).Dot(u)
	prefixEnd := min(ad.WaypointIdx, len(ad.drivePath)-1)
	for prefixEnd > 0 && ad.drivePath[prefixEnd-1].Sub(center).Dot(u) > busAlong-25 {
		prefixEnd--
	}

	points := make([]sim.Vec2, 0, prefixEnd+len(ad.drivePath)-exitIdx+48)
	points = append(points, ad.drivePath[:prefixEnd]...)
	pBehind := center.Add(u.Mul(math.Min(busAlong-15, -entryHalf-15))).Add(r.Mul(inOffset))
	if len(points) == 0 || points[len(points)-1].Sub(center).Dot(u) < pBehind.Sub(center).Dot(u)-2 {
		points = append(points, pBehind)
	}

	tail := exitIdx
	outOffset := ad.drivePath[exitIdx].Sub(center).Dot(rOut)
	if turn != routeTurnStraight {
		tail = turnExitTailIndex(ad.drivePath, tail, center, out, exitHalf+105)
		if ad.drivePath[tail].Sub(center).Dot(out) <= exitHalf+105 && tail+1 < len(ad.drivePath) {
			tail++
		}
		runout := 100.0
		if tail < len(ad.drivePath) {
			if maxOut := ad.drivePath[tail].Sub(center).Dot(out) - exitHalf - 10; maxOut < runout {
				runout = math.Max(10, maxOut)
			}
		}
		if turn == routeTurnLeft {
			outProbe := center.Add(out.Mul(exitHalf + 120))
			outCoord, outEW := center.Y, math.Abs(out.X) > .5
			if !outEW {
				outCoord = center.X
			}
			exitRoad, exitOK := f.world.RoadAt(outCoord, outEW, outProbe)
			outLanes := 0
			if exitOK {
				outLanes = exitRoad.LaneCount()
			}
			var curve []sim.Vec2
			if curLane >= 1 && halfLanes >= 2 && exitOK && outLanes >= 4 && !exitRoad.HasMedianBusLane() {
				curve = sim.CurbBusLeftTurnPath(center, u, out, inOffset, outOffset, g, halfLanes*2, outLanes)
			} else {
				curve = sim.WideLeftTurnPath(center, u, out, inOffset, outOffset, g)
			}
			if len(curve) > 0 {
				curve[len(curve)-1] = center.Add(out.Mul(exitHalf + runout)).Add(rOut.Mul(outOffset))
			}
			points = append(points, curve...)
		} else {
			radius := math.Max(60, math.Min(entryHalf-outOffset, exitHalf-inOffset))
			if maxR := -busAlong - outOffset - 5; maxR >= 60 && maxR < radius {
				radius = maxR
			}
			curve := sim.QuarterTurnPath(center, u, out, inOffset, outOffset, radius)
			if len(curve) > 0 && curve[0].Sub(center).Dot(u) > -entryHalf+2 {
				pStart := center.Sub(u.Mul(entryHalf)).Add(r.Mul(inOffset))
				if pStart.Sub(center).Dot(u) < curve[0].Sub(center).Dot(u)-2 && pStart.Sub(center).Dot(u) > pBehind.Sub(center).Dot(u)+2 {
					points = append(points, pStart)
				}
			}
			points = append(points, curve...)
			points = append(points, center.Add(out.Mul(exitHalf+runout)).Add(rOut.Mul(outOffset)))
		}
	} else {
		pEntry := center.Sub(u.Mul(entryHalf)).Add(r.Mul(inOffset))
		if pEntry.Sub(center).Dot(u) > pBehind.Sub(center).Dot(u)+2 {
			points = append(points, pEntry)
		}
		outProbe := center.Add(out.Mul(exitHalf + 120))
		outCoord, outEW := center.Y, math.Abs(out.X) > .5
		if !outEW {
			outCoord = center.X
		}
		outLanes := f.world.GetRoadLanes(outCoord, outEW, outProbe)
		maxOutOff := math.Max(0.5*f.laneWidth, float64(max(2, outLanes)/2)*f.laneWidth-0.5*f.laneWidth)
		crossOutOff := math.Min(inOffset, maxOutOff)
		if math.Abs(crossOutOff-inOffset) > 1 {
			const steps = 8
			for s := 1; s <= steps; s++ {
				w := float64(s) / steps
				smooth := w * w * (3 - 2*w)
				along := -entryHalf + (entryHalf+exitHalf)*w
				off := inOffset + (crossOutOff-inOffset)*smooth
				points = append(points, center.Add(out.Mul(along)).Add(rOut.Mul(off)))
			}
		} else {
			points = append(points, center.Add(out.Mul(exitHalf)).Add(rOut.Mul(crossOutOff)))
		}

		// Advance past any short exit points right on the junction mouth so the
		// transition back to the nominal lane stays strictly ahead along out.
		for tail < len(ad.drivePath)-1 && ad.drivePath[tail].Sub(center).Dot(out) <= exitHalf+105 &&
			ad.drivePath[tail+1].Sub(ad.drivePath[tail]).Normalize().Dot(out) > .95 {
			tail++
		}
		a, b := ad.drivePath[tail-1], ad.drivePath[tail]
		fa, fb := a.Sub(center).Dot(out), b.Sub(center).Dot(out)
		if fb > exitHalf+150 && fb > fa+1e-6 {
			runoutAlong := math.Min(fb-20, exitHalf+110)
			if runoutAlong > exitHalf+10 {
				points = append(points, center.Add(out.Mul(runoutAlong)).Add(rOut.Mul(crossOutOff)))
			}
			if fb > exitHalf+380 {
				rejoinAlong := math.Min(fb-20, exitHalf+420)
				t := sim.Clamp((rejoinAlong-fa)/(fb-fa), 0, 1)
				points = append(points, a.Add(b.Sub(a).Mul(t)))
			}
		} else if fb > exitHalf+25 {
			points = append(points, center.Add(out.Mul(math.Min(fb-10, exitHalf+105))).Add(rOut.Mul(crossOutOff)))
		}
	}

	points = append(points, ad.drivePath[tail:]...)
	ad.drivePath = dedupePath(points)
	ad.cumLen = make([]float64, len(ad.drivePath))
	for i := 1; i < len(ad.drivePath); i++ {
		ad.cumLen[i] = ad.cumLen[i-1] + ad.drivePath[i].Distance(ad.drivePath[i-1])
	}

	centerCopy := center
	ad.fallbackJunction = &centerCopy
	ad.blockedTurnMergeTimer = 0
	ad.approachMerge = nil
	ad.approachMergeJoined = false
	f.bus.LaneMergeRequest = nil
	ad.rightTurnWaiting = false
	ad.rightTurnJunction = nil
	if curLane == 0 {
		ad.curbBusRight = false
		ad.medianRightChoice = nil
	}
	ad.busTurnHoldDistance = 0
	ad.pathReturnHold = nil
	ad.busLaneJunction = nil
	ad.busLaneTurnChosen = false
	ad.ordinaryBusLeft = false
	ad.pathOrdinaryBusLeft = false
	ad.blockedLeftMerge = nil
	ad.blockedLeftMergeTime = 0
	ad.holdLaneOffset(0)
	ad.OvertakeState = OvertakeNone
	ad.pathSynced = false
	ad.trackPath(f.bus.Pos)
	return true
}
