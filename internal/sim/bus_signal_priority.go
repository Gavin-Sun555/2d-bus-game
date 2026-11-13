package sim

import "math"

const BusPriorityMaxExtension = 10.0
const busPriorityMinGreen = 3.0
const emptySignalMinGreen = .5

// BusSignalRequest is also the saved FIFO entry. Served prevents an expired
// request from immediately acquiring another extension while still in range.
type BusSignalRequest struct {
	BusID, IntersectionID int
	Heading               float64
	Phase                 LightPhase
	Served                bool `json:",omitempty"`
}

// BusSignalObservation includes every live bus, even when it has no new
// request, so a granted request remains active until its trailer clears.
type BusSignalObservation struct {
	BusID   int
	Bus     *Bus
	Request *BusSignalRequest
}

func (tls *TrafficLightSystem) ObserveBusPriority(enabled bool, observations []BusSignalObservation, laneWidth float64, stationCrossings ...[]StationCrossing) {
	tls.BusPriorityEnabled = enabled
	if !enabled {
		tls.carDemandRefresh = 0
	}
	for _, inter := range tls.Intersections {
		inter.waitingRegularLeft, inter.waitingLeft = 0, 0
		if !enabled {
			inter.BusPriorityQueue = nil
			inter.PriorityResumePhase = nil
			inter.demandKnown = false
			continue
		}
		queue := inter.BusPriorityQueue[:0]
		for _, old := range inter.BusPriorityQueue {
			for _, ob := range observations {
				if ob.Bus == nil || ob.BusID != old.BusID {
					continue
				}
				if r := ob.Request; r != nil && r.IntersectionID == inter.ID {
					if r.Phase != old.Phase {
						old.Phase, old.Served = r.Phase, false
					}
					queue = append(queue, old)
				} else if inter.Phase == old.Phase && busOccupiesSignalBox(ob.Bus, inter, laneWidth) {
					queue = append(queue, old)
				}
				break
			}
		}
		inter.BusPriorityQueue = queue
		for _, ob := range observations {
			if ob.Bus == nil {
				continue
			}
			inter.observeWaitingLeft(ob.Bus, laneWidth)
			r := ob.Request
			if inter.demandKnown {
				inter.observeBusSignalDemand(ob.Bus, r, laneWidth)
			}
			if r == nil || r.IntersectionID != inter.ID {
				continue
			}
			idx := inter.GetApproachIndexForHeading(r.Heading)
			if !inter.ApproachBusLane[idx] || inter.ApproachLanes[idx] < 4 || !inter.validPriorityPhase(r.Phase) {
				continue
			}

			found := false
			for _, old := range inter.BusPriorityQueue {
				if old.BusID == ob.BusID {
					found = true
					break
				}
			}
			if !found {
				entry := *r
				entry.BusID, entry.Served = ob.BusID, false
				inter.BusPriorityQueue = append(inter.BusPriorityQueue, entry)
			}
		}
	}

	if len(stationCrossings) == 0 {
		return
	}
	crossings := stationCrossings[0]
	if tls.StationCrossingStates == nil {
		tls.StationCrossingStates = make(map[int]*StationCrossingSignalState)
	}
	if !enabled {
		for _, st := range tls.StationCrossingStates {
			st.Priority = false
		}
		return
	}
	for _, c := range crossings {
		if c.Kind != StationSignalZebra || !c.BusLane {
			continue
		}
		st, ok := tls.StationCrossingStates[c.ID]
		if !ok {
			normPhase := math.Mod(tls.StationCrossingElapsed+c.phaseOffset, stationSignalCycle)
			st = &StationCrossingSignalState{ID: c.ID}
			if normPhase < 24 {
				st.Phase = StationSignalGreen
				st.Timer = normPhase
			} else if normPhase < 28 {
				st.Phase = StationSignalYellow
				st.Timer = normPhase - 24
			} else {
				st.Phase = StationSignalRed
				st.Timer = normPhase - 28
			}
			tls.StationCrossingStates[c.ID] = st
		}
		hasBus := false
		for _, ob := range observations {
			b := ob.Bus
			if b == nil || b.Speed < 0 || b.DoorsOpen || b.PassengerServiceRemaining() > 0 {
				continue
			}
			if math.Abs(math.Cos(c.Heading-b.Heading)) < 0.85 {
				continue
			}
			forward := UnitFromAngle(intersectionApproachHeading(b.Heading))
			right := Vec2{X: -forward.Y, Y: forward.X}
			delta := c.Center.Sub(b.Pos)
			if math.Abs(delta.Dot(right)) > float64(c.Lanes)*laneWidth/2+ShoulderWidth {
				continue
			}
			dist := delta.Dot(forward)
			if dist > -b.Length/2-25 && dist < math.Min(900, 600+math.Abs(b.Speed)*1.5) {
				hasBus = true
				break
			}
		}
		st.Priority = hasBus
	}
}

// A lead bus at the line represents actual demand, even on an approach
// where buses cannot request signal priority.
func (inter *TrafficIntersection) observeWaitingLeft(bus *Bus, laneWidth float64) {
	c := bus.JunctionCorridor
	if c == nil || c.Entered || c.Remaining <= 0 || c.Center != inter.Center ||
		len(c.Bodies) < 2 || math.Abs(bus.Speed) > 5 ||
		!inter.ApproachHasDedicatedLanesForHeading(c.EntryHeading) {
		return
	}
	angle := NormalizeAngle(c.Bodies[len(c.Bodies)-1].Heading - c.EntryHeading)
	uTurn := math.Abs(angle) >= 2.8
	if !uTurn && angle >= -.35 {
		return
	}
	u := UnitFromAngle(c.EntryHeading)
	gap := inter.Center.Sub(bus.Pos).Dot(u) -
		inter.GetStopLineDistForHeading(c.EntryHeading, laneWidth) - bus.Length/2
	if gap < 0 || gap >= laneWidth/2 {
		return
	}
	phase := PhaseEWLeftGreen
	if math.Abs(u.Y) > .5 {
		phase = PhaseNSLeftGreen
	}
	mask := uint8(1 << (phase / 2))
	inter.waitingLeft |= mask
	if !uTurn {
		inter.waitingRegularLeft |= mask
	}
}

func (inter *TrafficIntersection) observeBusSignalDemand(bus *Bus, request *BusSignalRequest, laneWidth float64) {
	heading := bus.Heading
	c := bus.JunctionCorridor
	if c != nil && c.Remaining > 0 && c.Center == inter.Center {
		if c.Entered {
			return
		}
		heading = c.EntryHeading
	}
	if !inter.IncomingSignalApproach(bus.Pos, heading, bus.Length, bus.Speed, laneWidth) {
		return
	}
	if request != nil && request.IntersectionID == inter.ID && inter.validPriorityPhase(request.Phase) {
		inter.signalDemand |= 1 << (request.Phase / 2)
	} else if c != nil && c.Remaining > 0 && c.Center == inter.Center && len(c.Bodies) > 1 {
		angle := NormalizeAngle(c.Bodies[len(c.Bodies)-1].Heading - heading)
		inter.ObserveSignalDemand(heading, angle < -.35 || math.Abs(angle) >= 2.8)
	} else {
		// A manual bus with unknown intent is demand for either movement.
		inter.ObserveSignalDemand(heading, false)
		inter.ObserveSignalDemand(heading, true)
	}
}

func busOccupiesSignalBox(bus *Bus, inter *TrafficIntersection, laneWidth float64) bool {
	ew, _, ns, _ := inter.GetApproachDimensions(laneWidth)
	box := OBB{Center: inter.Center, HalfLen: ew, HalfWid: ns}
	hit, _, _ := bus.CollidesBody(box)
	return hit
}

func (inter *TrafficIntersection) validPriorityPhase(p LightPhase) bool {
	switch p {
	case PhaseEWGreen, PhaseNSGreen:
		return true
	case PhaseEWLeftGreen:
		return inter.IsMajor && inter.needsEWLeftPhase()
	case PhaseNSLeftGreen:
		return inter.IsMajor && inter.needsNSLeftPhase()
	}
	return false
}

// Retain service age for waiting traffic across save/load. An empty phase
// must discard old age, or an early cutoff can keep reselecting it forever.
func (inter *TrafficIntersection) updatePriorityPhaseWait(dt float64) {
	for index := range inter.PriorityPhaseWait {
		phase := LightPhase(index * 2)
		if !inter.validPriorityPhase(phase) || !inter.phaseHasDemand(phase) {
			inter.PriorityPhaseWait[index] = 0
			continue
		}
		duration := inter.GreenDuration
		if phase == PhaseEWLeftGreen || phase == PhaseNSLeftGreen {
			duration = inter.LeftGreenDuration
		}
		if inter.Phase == phase && (inter.Timer >= duration ||
			(inter.Timer >= busPriorityMinGreen && inter.PriorityPhaseWait[index] < inter.priorityServiceThresholdFor(phase))) {
			inter.PriorityPhaseWait[index] = 0
		} else {
			inter.PriorityPhaseWait[index] += math.Max(0, dt)
		}
	}
}

// Wide junctions need earlier protection because more of a short phase
// is spent reaching and clearing the turning area.
func (inter *TrafficIntersection) priorityServiceThreshold() float64 {
	cycle := inter.CycleDuration()
	for _, lanes := range inter.ApproachLanes {
		if lanes > 0 && lanes < 6 {
			return cycle
		}
	}
	return cycle / 2
}

func (inter *TrafficIntersection) priorityServiceThresholdFor(phase LightPhase) float64 {
	if inter.waitingLeft&(1<<(phase/2)) != 0 {
		return inter.CycleDuration() / 2
	}
	return inter.priorityServiceThreshold()
}

func (inter *TrafficIntersection) overduePriorityPhase() (LightPhase, bool) {
	var phase LightPhase
	longest := inter.CycleDuration()
	found := false
	for index, wait := range inter.PriorityPhaseWait {
		candidate := LightPhase(index * 2)
		if inter.validPriorityPhase(candidate) && inter.phaseHasDemand(candidate) && wait >= longest {
			phase, longest, found = candidate, wait, true
		}
	}
	return phase, found
}

func (inter *TrafficIntersection) firstBusPriorityRequest() *BusSignalRequest {
	for i := range inter.BusPriorityQueue {
		if !inter.BusPriorityQueue[i].Served {
			return &inter.BusPriorityQueue[i]
		}
	}
	return nil
}

func (inter *TrafficIntersection) busPriorityTarget(request *BusSignalRequest) (LightPhase, bool) {
	if phase, overdue := inter.overduePriorityPhase(); overdue {
		return phase, true
	}
	return request.Phase, false
}

func (inter *TrafficIntersection) priorityGreenEnd(duration float64, request *BusSignalRequest) float64 {
	if inter.PriorityResumePhase != nil {
		if request != nil && request.Phase == inter.Phase {
			return duration + BusPriorityMaxExtension
		}
		if !inter.phaseHasDemand(inter.Phase) {
			return math.Min(duration, emptySignalMinGreen)
		}
		return math.Min(duration, busPriorityMinGreen)
	}
	if request == nil {
		if inter.emptyOrdinaryGreen() {
			return math.Min(duration, emptySignalMinGreen)
		}
		return duration
	}
	if request.Phase != inter.Phase && !inter.phaseHasDemand(inter.Phase) {
		return math.Min(duration, emptySignalMinGreen)
	}
	target, _ := inter.busPriorityTarget(request)
	if target == inter.Phase {
		return duration + BusPriorityMaxExtension
	}
	if inter.PriorityPhaseWait[int(inter.Phase)/2] >= inter.priorityServiceThresholdFor(inter.Phase) {
		return duration
	}
	return math.Min(duration, busPriorityMinGreen)
}

// GreenRemaining includes the extension or early cutoff in force this tick.
func (inter *TrafficIntersection) GreenRemaining(busPriority bool) float64 {
	duration, _, valid := inter.phaseTransition()
	if !valid || inter.Phase%2 != 0 {
		return 0
	}
	if busPriority {
		duration = inter.priorityGreenEnd(duration, inter.firstBusPriorityRequest())
	}
	return math.Max(0, duration-inter.Timer)
}

// Transitions out of green always use the full yellow phase.
func (inter *TrafficIntersection) advanceBusPriority() bool {
	request := inter.firstBusPriorityRequest()
	if request == nil && inter.PriorityResumePhase == nil && !inter.emptyOrdinaryGreen() {
		return false
	}
	duration, next, valid := inter.phaseTransition()
	if !valid {
		return false
	}
	if inter.Phase%2 == 0 {
		if end := inter.priorityGreenEnd(duration, request); inter.Timer >= end {
			if end >= duration+BusPriorityMaxExtension {
				for i := range inter.BusPriorityQueue {
					if inter.BusPriorityQueue[i].Phase == inter.Phase {
						inter.BusPriorityQueue[i].Served = true
					}
				}
			}
			inter.Phase, inter.Timer = next, 0
		}
		return true
	}
	if inter.Timer < duration {
		return true
	}
	if inter.PriorityResumePhase != nil {
		target := *inter.PriorityResumePhase
		inter.PriorityResumePhase = nil
		if inter.validPriorityPhase(target) {
			inter.Phase, inter.Timer = target, 0
			return true
		}
	}
	if request == nil {
		return false
	}
	target, overdue := inter.busPriorityTarget(request)
	if !overdue && inter.phaseHasDemand(next) &&
		inter.PriorityPhaseWait[int(next)/2] >= inter.priorityServiceThresholdFor(next) {
		target = next
	}
	if !overdue && target == request.Phase && target != next && inter.mixedPriorityRoads() {
		inter.PriorityResumePhase = &next
	}
	inter.Phase, inter.Timer = target, 0
	return true
}
