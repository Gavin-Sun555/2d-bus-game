package sim

import "math"

// BeginSignalDemand starts a complete snapshot of the incoming traffic.
func (tls *TrafficLightSystem) BeginSignalDemand() {
	for _, inter := range tls.Intersections {
		inter.demandKnown = true
		inter.signalDemand = inter.carSignalDemand
	}
}

// RefreshCarSignalDemand samples civilian demand every 0.15 simulation seconds.
// Incoming traffic is observed well before it can reach the stop line.
func (tls *TrafficLightSystem) RefreshCarSignalDemand(dt float64, density int) bool {
	tls.carDemandRefresh -= dt
	if tls.carDemandRefresh > 0 && tls.carDemandDensity == density {
		return false
	}
	tls.carDemandRefresh, tls.carDemandDensity = .15, density
	for _, inter := range tls.Intersections {
		inter.carSignalDemand = 0
	}
	return true
}

// IncomingSignalApproach excludes vehicles that have already crossed the
// stop line. Include approaching traffic with time to reach the green.
func (inter *TrafficIntersection) IncomingSignalApproach(pos Vec2, heading, length, speed, laneWidth float64) bool {
	d := inter.Center.Sub(pos)
	reach := length/2 + 180 + math.Max(0, speed)*1.2
	ew, _, ns, _ := inter.GetApproachDimensions(laneWidth)
	limit := reach + math.Max(ew, ns)
	if math.Abs(d.X) > limit || math.Abs(d.Y) > limit {
		return false
	}
	idx := inter.GetApproachIndexForHeading(heading)
	if !inter.ApproachHasRoad[idx] {
		return false
	}
	u := UnitFromAngle(intersectionApproachHeading(heading))
	stopLine := ew
	if idx >= 2 {
		stopLine = ns
	}
	gap := d.Dot(u) - stopLine - length/2
	return gap >= -8 && gap <= reach-length/2 &&
		math.Abs(d.X*u.Y-d.Y*u.X) <= float64(inter.ApproachLanes[idx])*laneWidth/2+12
}

func (inter *TrafficIntersection) ObserveSignalDemand(heading float64, left bool) {
	inter.signalDemand |= inter.signalDemandBit(heading, left)
}

func (inter *TrafficIntersection) ObserveCarSignalDemand(heading float64, left bool) {
	inter.carSignalDemand |= inter.signalDemandBit(heading, left)
}

func (inter *TrafficIntersection) signalDemandBit(heading float64, left bool) uint8 {
	phase := PhaseEWGreen
	if inter.GetApproachIndexForHeading(heading) >= 2 {
		phase = PhaseNSGreen
	}
	if left && inter.ApproachHasDedicatedLanesForHeading(heading) {
		phase += 2
	}
	return 1 << (phase / 2)
}

func (inter *TrafficIntersection) phaseHasDemand(phase LightPhase) bool {
	return !inter.demandKnown || inter.signalDemand&(1<<(phase/2)) != 0
}

func (inter *TrafficIntersection) mixedPriorityRoads() bool {
	priority, ordinary := false, false
	for i, road := range inter.ApproachHasRoad {
		if road {
			priority = priority || inter.ApproachBusLane[i]
			ordinary = ordinary || !inter.ApproachBusLane[i]
		}
	}
	return priority && ordinary
}

func (inter *TrafficIntersection) ordinarySignalPhase(phase LightPhase) bool {
	if !inter.mixedPriorityRoads() {
		return false
	}
	first := 0
	if phase >= PhaseNSGreen {
		first = 2
	}
	return !inter.ApproachBusLane[first] && !inter.ApproachBusLane[first+1]
}

// With other traffic waiting, an empty ordinary green need only be displayed
// briefly before its normal yellow. With no demand, keep the normal cycle.
func (inter *TrafficIntersection) emptyOrdinaryGreen() bool {
	return inter.Phase%2 == 0 && inter.signalDemand != 0 && !inter.phaseHasDemand(inter.Phase) && inter.ordinarySignalPhase(inter.Phase)
}
