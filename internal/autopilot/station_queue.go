package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

// QueueAtStation holds the bay alignment until this bus reaches its boarding
// berth. Overtaking and the previous stop's road path cannot split the queue.
func (ad *AutopilotDriver) QueueAtStation(stopID int) { ad.queueStopID = stopID }
func (ad *AutopilotDriver) stationQueueControls(bus *sim.Bus, stop *sim.BusStop, obstacles []VehicleObstacle) (stationDockInputs, bool) {
	if ad.queueStopID < 0 {
		return stationDockInputs{}, false
	}
	if ad.queueStopID != stop.ID {
		ad.queueStopID = -1
		return stationDockInputs{}, false
	}
	u := sim.UnitFromAngle(stop.Heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	targets := availableStationBerths(bus, stop, obstacles)
	targetPosition := stop.FrontBerthPosition()
	berthAvailable := len(targets) > 0
	if berthAvailable {
		targetPosition = targets[0]
	} else if stop.HasTwoBerths() {
		targetPosition = stop.Pos.Sub(u.Mul(stop.Length/2 + sim.StationBerthGap + bus.Length/2))
	}
	delta := targetPosition.Sub(bus.Pos)
	along, lateral := delta.Dot(u), delta.Dot(right)
	if stop.FrontBerthPosition().Sub(bus.Pos).Dot(u) < -bus.Length || math.Abs(lateral) > bus.Width+20 || math.Cos(bus.Heading-stop.Heading) < .85 {
		// The manual driver has left the queue; resume normal localization.
		ad.queueStopID = -1
		return stationDockInputs{}, false
	}
	ad.holdLaneOffset(0)
	ad.OvertakeState = OvertakeNone
	target := math.Min(70, math.Sqrt(2*85*math.Max(0, along-3)))
	for _, ob := range obstacles {
		d := ob.Pos.Sub(bus.Pos)
		yaw := ob.Heading - stop.Heading
		halfAlong := math.Abs(math.Cos(yaw))*ob.Length/2 + math.Abs(math.Sin(yaw))*ob.Width/2
		halfAcross := math.Abs(math.Sin(yaw))*ob.Length/2 + math.Abs(math.Cos(yaw))*ob.Width/2
		if d.Dot(u) <= 0 || math.Abs(d.Dot(right)) > bus.Width/2+halfAcross+8 {
			continue
		}
		gap := d.Dot(u) - bus.Length/2 - halfAlong - 24
		target = math.Min(target, math.Sqrt(2*135*math.Max(0, gap)))
	}
	if berthAvailable && atSelectedStationBerth(bus, targetPosition, stop.Heading) && math.Abs(bus.Speed) < 8 {
		// Do not begin boarding while another bus is still beside this berth.
		blocked := false
		for _, ob := range obstacles {
			d := ob.Pos.Sub(targetPosition)
			if math.Abs(d.Dot(u)) < (bus.Length+ob.Length)/2+12 && math.Abs(d.Dot(right)) < (bus.Width+ob.Width)/2+8 {
				blocked = true
				break
			}
		}
		if !blocked {
			ad.queueStopID = -1
			ad.beginDocking(stop)
			return stationDockInputs{brake: 1, toggleDoors: !bus.DoorsOpen && math.Abs(bus.Speed) < 8, isDocked: true}, true
		}
	}
	yaw := sim.NormalizeAngle(stop.Heading - bus.Heading)
	steer := sim.Clamp(yaw*2+lateral*.025, -.35, .35)
	throttle, brake := ad.longitudinalControls(bus.Speed, target)
	if bus.DoorsOpen {
		return stationDockInputs{brake: 1, toggleDoors: math.Abs(bus.Speed) < 8}, true
	}
	ad.LastTargetSpeed = target
	return stationDockInputs{throttle: throttle, brake: brake, steer: steer}, true
}
