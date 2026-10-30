package sim

import "math"

const BusPriorityMaxExtension = 10.0
const busPriorityMinGreen = 3.0

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

func (tls *TrafficLightSystem) ObserveBusPriority(enabled bool, observations []BusSignalObservation, laneWidth float64) {
	tls.BusPriorityEnabled = enabled
	for _, inter := range tls.Intersections {
		if !enabled {
			inter.BusPriorityQueue = nil
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
				} else if inter.GetSignalForHeading(old.Heading, old.Phase == PhaseEWLeftGreen || old.Phase == PhaseNSLeftGreen) == SignalGreen && busOccupiesSignalBox(ob.Bus, inter, laneWidth) {
					queue = append(queue, old)
				}
				break
			}
		}
		inter.BusPriorityQueue = queue
		for _, ob := range observations {
			r := ob.Request
			if ob.Bus == nil || r == nil || r.IntersectionID != inter.ID {
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
		return inter.IsMajor && inter.HasEW4Lane()
	case PhaseNSLeftGreen:
		return inter.IsMajor && inter.HasNS4Lane()
	}
	return false
}

// Return true when the priority controller has handled this tick. Transitions
// out of green always use its corresponding full yellow phase.
func (inter *TrafficIntersection) advanceBusPriority() bool {
	var head *BusSignalRequest
	for i := range inter.BusPriorityQueue {
		if !inter.BusPriorityQueue[i].Served {
			head = &inter.BusPriorityQueue[i]
			break
		}
	}
	if head == nil {
		return false
	}
	duration, next, valid := inter.phaseTransition()
	if !valid {
		return false
	}
	isGreen := inter.Phase == PhaseEWGreen || inter.Phase == PhaseNSGreen || inter.Phase == PhaseEWLeftGreen || inter.Phase == PhaseNSLeftGreen
	if !isGreen {
		if inter.Timer >= duration {
			inter.Phase, inter.Timer = head.Phase, 0
		}
		return true
	}
	if head.Phase != inter.Phase {
		if inter.Timer >= math.Min(busPriorityMinGreen, duration) {
			inter.Phase, inter.Timer = next, 0
		}
		return true
	}
	if inter.Timer >= duration+BusPriorityMaxExtension {
		for i := range inter.BusPriorityQueue {
			if inter.BusPriorityQueue[i].Phase == inter.Phase {
				inter.BusPriorityQueue[i].Served = true
			}
		}
		inter.Phase, inter.Timer = next, 0
	}
	return true
}
