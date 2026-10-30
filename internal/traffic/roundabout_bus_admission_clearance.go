package traffic

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Do not weaken admission while a long vehicle still circulates. A short bus
// may begin a separated join once the owner is moving outward through an exit,
// without waiting for its last axle to leave the complete roundabout.
func reservedOwnerLeaving(owner *sim.Bus, rb sim.Roundabout) bool {
	if owner == nil || owner.Speed <= 5 {
		return false
	}
	radial := owner.Pos.Sub(rb.Center)
	radius := radial.Length()
	return radius >= rb.Radius-owner.Width && radial.Dot(sim.UnitFromAngle(owner.Heading)) > radius*.5
}

// Short buses need a clear entrance, not exclusive possession of the entire
// ring. Keep the reservation for cars/other long entrants, but let the normal
// bus controller decide moving-traffic priority after this local clearance.
// Unknown/cross-lane entry geometry retains the conservative admission hold.
func shortBusClearsReservedEntry(bus, owner *sim.Bus, rb sim.Roundabout, forward sim.Vec2, laneWidth float64) bool {
	if bus == nil || owner == nil || bus.IsArticulated() {
		return false
	}
	own := sim.OBB{Center: bus.Pos, Heading: bus.Heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
	_, curve, ok := sim.RoundaboutEntryCorridor(rb, forward, laneWidth, own, true)
	if !ok {
		return false
	}
	others := owner.BodyParts()
	clear := func(body sim.OBB) bool {
		body.HalfLen += 10
		body.HalfWid += 8
		for _, part := range others {
			if part.HalfLen <= 0 {
				continue
			}
			part.HalfLen += 4
			part.HalfWid += 4
			if hit, _, _ := sim.CheckOBBCollision(body, part); hit {
				return false
			}
		}
		return true
	}
	if !clear(own) {
		return false
	}
	// Check the complete local join, including the rear section of the long
	// owner. Sampling density and padding cover the gaps between adjacent poses.
	for i := 0; i <= 48; i++ {
		p, t := sim.RoundaboutEntryPoint(curve, float64(i)/48)
		body := own
		body.Center = p
		body.Heading = math.Atan2(t.Y, t.X)
		if !clear(body) {
			return false
		}
	}
	return true
}
