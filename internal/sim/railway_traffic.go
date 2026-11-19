package sim

import "math"

const RailGateMargin = 34.

// LevelRailApproach gives the distance from the front bumper to each gate of
// the nearest level crossing ahead, while the bumper is still outside it.
func (w *World) LevelRailApproach(bumper Vec2, heading, lookahead float64) (entry, exit float64, found bool) {
	if w == nil || len(w.railways) == 0 {
		return 0, 0, false
	}
	sinH := math.Sin(heading)
	if math.Abs(sinH) < .8 {
		return 0, 0, false
	}
	direction := math.Copysign(1, sinH)
	entry = lookahead
	for _, r := range w.Railways() {
		gate := r.HalfWidth + RailGateMargin
		dist := (r.Y-bumper.Y)*direction - gate
		if dist < 0 || dist >= entry {
			continue
		}
		for _, c := range r.Crossings {
			if c.Level && math.Abs(c.Center.X-bumper.X) < c.RoadHalfWidth+ShoulderWidth {
				entry, exit, found = dist, dist+2*gate, true
				break
			}
		}
	}
	return entry, exit, found
}

func (w *World) RailSpeedLimit(tls *TrafficLightSystem, bumper Vec2, heading, speed, braking float64) float64 {
	if w == nil || tls == nil {
		return math.Inf(1)
	}
	limit := math.Inf(1)
	for _, r := range w.Railways() {
		limit = math.Min(limit, r.speedLimit(tls.RailElapsed, bumper, heading, speed, braking))
	}
	return limit
}

func (r *Railway) speedLimit(elapsed float64, bumper Vec2, heading, speed, braking float64) float64 {
	gate := r.HalfWidth + RailGateMargin
	lookAhead := speed*speed/(2*braking) + 90
	if math.Abs(bumper.Y-r.Y) > lookAhead+gate {
		return math.Inf(1)
	}
	direction := math.Sin(heading)
	if math.Abs(direction) < .8 {
		return math.Inf(1)
	}
	dist := (r.Y-bumper.Y)*math.Copysign(1, direction) - gate
	if dist <= 0 || dist >= lookAhead {
		return math.Inf(1)
	}
	for _, c := range r.Crossings {
		if c.Level && math.Abs(c.Center.X-bumper.X) < c.RoadHalfWidth+ShoulderWidth && c.Gate(elapsed).Red {
			return math.Sqrt(2 * braking * math.Max(0, dist-15))
		}
	}
	return math.Inf(1)
}

// RailGateCorrection enforces the physical barrier for all drivers. Vehicles
// already inside may clear; the train never yields to a vehicle left on the tracks.
func (w *World) RailGateCorrection(tls *TrafficLightSystem, before Vec2, body OBB) (shift float64, trainContact bool) {
	if w == nil || tls == nil {
		return 0, false
	}
	for _, r := range w.Railways() {
		if correction, contact := r.gateCorrection(tls.RailElapsed, before, body); correction != 0 || contact {
			return correction, contact
		}
	}
	return 0, false
}

func (r *Railway) gateCorrection(elapsed float64, before Vec2, body OBB) (float64, bool) {
	gate := r.HalfWidth + RailGateMargin
	delta := body.Center.Sub(before)
	if math.Abs(body.Center.Y-r.Y) > gate+body.HalfLen+body.HalfWid+math.Abs(delta.Y)+2 {
		return 0, false
	}
	sinH, cosH := math.Sincos(body.Heading)
	halfX := body.HalfLen*math.Abs(cosH) + body.HalfWid*math.Abs(sinH)
	halfY := body.HalfLen*math.Abs(sinH) + body.HalfWid*math.Abs(cosH)
	for _, c := range r.Crossings {
		if !c.Level || math.Abs(c.Center.X-body.Center.X) > c.RoadHalfWidth+ShoulderWidth+halfX {
			continue
		}
		state := c.Gate(elapsed)
		if !state.Red {
			continue
		}
		beforeY, afterY := before.Y-r.Y, body.Center.Y-r.Y
		switch {
		case delta.Y > 0 && beforeY+halfY <= -gate && afterY+halfY > -gate:
			return -gate - (afterY + halfY) - .1, false
		case delta.Y < 0 && beforeY-halfY >= gate && afterY-halfY < gate:
			return gate - (afterY - halfY) + .1, false
		case state.Occupied && math.Abs(afterY) < r.TrackSpacing+halfY+13:
			contact := false
			r.ForEachTrain(elapsed, func(train RailTrain) {
				contact = contact || (math.Abs(train.Pos.X-body.Center.X) <= train.Length()/2+halfX &&
					math.Abs(train.Pos.Y-body.Center.Y) <= halfY+13)
			})
			if contact {
				return math.Copysign(gate+halfY+2, afterY) - afterY, true
			}
		}
	}
	return 0, false
}

func (w *World) ProtectBusAtRail(tls *TrafficLightSystem, bus *Bus, before Vec2) bool {
	if w == nil || tls == nil {
		return false
	}
	near := false
	for _, r := range w.Railways() {
		near = near || math.Abs(bus.Pos.Y-r.Y) <= 340+math.Abs(bus.Pos.Y-before.Y)
	}
	if !near {
		return false
	}
	body := bus.BodyEnvelope()
	shift, hit := w.RailGateCorrection(tls, before.Add(body.Center.Sub(bus.Pos)), body)
	if shift != 0 {
		bus.Pos.Y += shift
		bus.Speed = 0
		bus.IsBraking = true
	}
	return hit
}

// RailIntersectionPullback returns the extra stop-line distance needed to hold
// an approaching vehicle before a level railway crossing when the space between
// the crossing and inter's stop line cannot fit vehicleLength.
func (w *World) RailIntersectionPullback(inter *TrafficIntersection, bumper Vec2, heading, laneWidth, vehicleLength float64) float64 {
	if w == nil || inter == nil {
		return 0
	}
	sinH, cosH := math.Sincos(heading)
	if math.Abs(sinH) < .8 || math.Abs(cosH) > .6 {
		return 0
	}
	dir := math.Copysign(1, sinH)
	stopLineDist := inter.GetStopLineDistForHeading(heading, laneWidth)
	sStopLine := inter.Center.Y*dir - stopLineDist
	sBumper := bumper.Y * dir
	pullback := 0.0
	for _, r := range w.Railways() {
		sRail := r.Y * dir
		gate := r.HalfWidth + RailGateMargin
		sEntry := sRail - gate
		sExit := sRail + gate
		if sRail >= sStopLine || sBumper > sEntry || (sStopLine-14.0)-vehicleLength >= sExit {
			continue
		}
		for _, c := range r.Crossings {
			if c.Level && math.Abs(c.Center.X-inter.Center.X) <= c.RoadHalfWidth+ShoulderWidth+laneWidth &&
				math.Abs(c.Center.X-bumper.X) <= c.RoadHalfWidth+ShoulderWidth+laneWidth {
				pullback = math.Max(pullback, sStopLine-sEntry)
				break
			}
		}
	}
	return pullback
}

// ClampStopBeforeRailCrossing pulls a forward stopping distance (measured from
// bumper along heading) back to the entry gate of any level railway crossing
// that bumper has not yet entered if stopping at stopDist-stopBuffer would
// leave any part of vehicleLength inside the railway crossing zone.
// If the vehicle is already occupying the crossing zone, it pushes the stopping
// distance forward so the vehicle never stops on the tracks.
func (w *World) ClampStopBeforeRailCrossing(bumper Vec2, heading, stopDist, vehicleLength, stopBuffer float64) (float64, bool) {
	if w == nil {
		return stopDist, false
	}
	sinH, cosH := math.Sincos(heading)
	if math.Abs(sinH) < .8 || math.Abs(cosH) > .6 {
		return stopDist, false
	}
	dir := math.Copysign(1, sinH)
	sBumper := bumper.Y * dir
	sStopBumper := sBumper + (stopDist - stopBuffer)
	sStopRear := sStopBumper - vehicleLength
	clamped := false
	for _, r := range w.Railways() {
		sRail := r.Y * dir
		gate := r.HalfWidth + RailGateMargin
		sEntry := sRail - gate
		sExit := sRail + gate

		for _, c := range r.Crossings {
			if !c.Level || math.Abs(c.Center.X-bumper.X) > c.RoadHalfWidth+ShoulderWidth+40 {
				continue
			}
			// Case 1: Bumper has not entered the level crossing gate yet.
			// Hold before the entry gate if stopping would leave vehicle on tracks.
			if sBumper <= sEntry {
				if sStopBumper > sEntry-stopBuffer && sStopRear < sExit {
					holdDist := sEntry - sBumper
					if holdDist < stopDist {
						stopDist = holdDist
						clamped = true
					}
				}
			} else if sBumper-vehicleLength < sExit {
				// Case 2: Vehicle is already inside the level crossing zone!
				// Never stop on the tracks: advance until the rear has cleared sExit.
				if sStopRear < sExit {
					clearDist := (sExit + vehicleLength + stopBuffer) - sBumper
					if clearDist > stopDist {
						stopDist = clearDist
						clamped = true
					}
				}
			}
			break
		}
	}
	return stopDist, clamped
}

// VehicleOnLevelCrossing reports whether any part of a vehicle with front
// bumper at bumper and total length vehicleLength occupies a level crossing.
func (w *World) VehicleOnLevelCrossing(bumper Vec2, heading, vehicleLength float64) bool {
	if w == nil {
		return false
	}
	sinH, cosH := math.Sincos(heading)
	if math.Abs(sinH) < .8 || math.Abs(cosH) > .6 {
		return false
	}
	dir := math.Copysign(1, sinH)
	sBumper := bumper.Y * dir
	sRear := sBumper - vehicleLength
	for _, r := range w.Railways() {
		sRail := r.Y * dir
		if sBumper <= sRail-r.HalfWidth || sRear >= sRail+r.HalfWidth+RailGateMargin {
			continue
		}
		for _, c := range r.Crossings {
			if c.Level && math.Abs(c.Center.X-bumper.X) <= c.RoadHalfWidth+ShoulderWidth+40 {
				return true
			}
		}
	}
	return false
}

// RailCrossingLookahead returns the extra forward obstacle scan range needed
// when approaching a level railway crossing so vehicles can see stopped traffic
// on the far side of the tracks before entering the crossing.
func (w *World) RailCrossingLookahead(bumper Vec2, heading, vehicleLength float64) float64 {
	if w == nil {
		return 0
	}
	sinH, cosH := math.Sincos(heading)
	if math.Abs(sinH) < .8 || math.Abs(cosH) > .6 {
		return 0
	}
	dir := math.Copysign(1, sinH)
	sBumper := bumper.Y * dir
	extra := 0.0
	for _, r := range w.Railways() {
		sRail := r.Y * dir
		sDeckStart := sRail - r.HalfWidth
		if sBumper > sDeckStart || sDeckStart-sBumper > 260 {
			continue
		}
		for _, c := range r.Crossings {
			if c.Level && math.Abs(c.Center.X-bumper.X) <= c.RoadHalfWidth+ShoulderWidth+40 {
				extra = math.Max(extra, 2*(r.HalfWidth+RailGateMargin)+vehicleLength+40)
				break
			}
		}
	}
	return extra
}
