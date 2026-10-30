package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

type roundaboutReservation struct {
	center  sim.Vec2
	owner   *sim.Bus
	entered bool
	holds   [1 + 6*MaxBusesPerLine]sim.RoundaboutAdmissionHold
}

// Long vehicles need the complete ring width while their trailer turns.
// Drain already-admitted cars before admitting one, then hold new arrivals
// until the last section is clear. Short buses may use a separated entrance
// once the owner begins leaving. All motion and collision rules still run.
func (tm *TrafficManager) updateRoundaboutReservations(player *sim.Bus, w *sim.World, count int, laneWidth float64) {
	var storage [1 + 6*MaxBusesPerLine]*sim.Bus
	buses := storage[:0]
	if player != nil {
		buses = append(buses, player)
	}
	if tm.AIBusMgr != nil {
		for _, ab := range tm.AIBusMgr.Buses {
			if ab != nil && ab.Bus != nil {
				buses = append(buses, ab.Bus)
			}
		}
	}
	for _, b := range buses {
		b.RoundaboutAdmissionHold = nil
	}
	if w == nil {
		tm.roundaboutReservations = nil
		return
	}
	if len(tm.roundaboutReservations) != len(w.Roundabouts) {
		tm.roundaboutReservations = make([]roundaboutReservation, len(w.Roundabouts))
	}
	for i, rb := range w.Roundabouts {
		r := &tm.roundaboutReservations[i]
		if r.center != rb.Center {
			*r = roundaboutReservation{center: rb.Center}
		}
		inside := func(b *sim.Bus) bool {
			for _, part := range b.BodyParts() {
				if part.HalfLen <= 0 {
					continue
				}
				u := sim.UnitFromAngle(part.Heading)
				right := sim.Vec2{X: -u.Y, Y: u.X}
				delta := rb.Center.Sub(part.Center)
				nearest := part.Center.Add(u.Mul(sim.Clamp(delta.Dot(u), -part.HalfLen, part.HalfLen))).Add(right.Mul(sim.Clamp(delta.Dot(right), -part.HalfWid, part.HalfWid)))
				if nearest.Distance(rb.Center) < rb.Radius+8 {
					return true
				}
			}
			return false
		}
		stillPresent := false
		for _, b := range buses {
			if b == r.owner {
				stillPresent = true
				break
			}
		}
		if !stillPresent {
			r.owner = nil
			r.entered = false
		}
		if r.owner != nil {
			if inside(r.owner) {
				r.entered = true
			}
			occupied := false
			for _, part := range r.owner.BodyParts() {
				if part.HalfLen > 0 && part.Center.Distance(rb.Center) < rb.Radius+math.Hypot(part.HalfLen, part.HalfWid)+20 {
					occupied = true
					break
				}
			}
			if r.entered && !occupied {
				r.owner = nil
				r.entered = false
			}
		}
		if r.owner != nil && !r.entered {
			intent := r.owner.RoundaboutEntry
			if intent == nil || intent.Remaining <= 0 || intent.Center != rb.Center || r.owner.Pos.Distance(rb.Center) > rb.Radius+450 {
				r.owner = nil
			} else {
				// A leader may publish its entry intent after the follower
				// claimed the ring. Never hold that physical leader for it.
				best := rb.Center.Sub(r.owner.Pos).Dot(intent.Forward)
				for _, b := range buses {
					entry := b.RoundaboutEntry
					if entry == nil || entry.Remaining <= 0 || entry.Center != rb.Center || entry.Forward.Dot(intent.Forward) < .99 {
						continue
					}
					along := rb.Center.Sub(b.Pos).Dot(entry.Forward)
					if along > 0 && roundaboutEntryPrecedes(b, r.owner, rb.Center, along, best) {
						r.owner = b
						best = along
					}
				}
			}
		}
		if r.owner == nil {
			longWaiting := false
			for _, b := range buses {
				if !b.IsArticulated() {
					continue
				}
				intent := b.RoundaboutEntry
				if inside(b) || (intent != nil && intent.Remaining > 0 && intent.Center == rb.Center && b.Pos.Distance(rb.Center) < rb.Radius+400) {
					longWaiting = true
					break
				}
			}
			if longWaiting {
				// Admit an already-circulating bus or the physical head of an
				// entrance queue. A short leader must not wait for a long bus
				// trapped behind it in that same lane.
				best := math.Inf(1)
				for _, b := range buses {
					if inside(b) {
						r.owner = b
						r.entered = true
						break
					}
					intent := b.RoundaboutEntry
					if intent == nil || intent.Remaining <= 0 || intent.Center != rb.Center || b.Pos.Distance(rb.Center) > rb.Radius+400 {
						continue
					}
					along := rb.Center.Sub(b.Pos).Dot(intent.Forward)
					if along > 0 && roundaboutEntryPrecedes(b, r.owner, rb.Center, along, best) {
						best = along
						r.owner = b
						r.entered = false
					}
				}
			}
		}
		if r.owner == nil {
			continue
		}
		drain := false
		for j, c := range tm.Cars {
			if j < count && c.InRoundabout && c.RoundaboutCenter == rb.Center {
				drain = true
				break
			}
		}
		for _, b := range buses {
			if b != r.owner && inside(b) {
				drain = true
			}
		}
		for j, b := range buses {
			if inside(b) || (b == r.owner && !drain) {
				continue
			}
			forward := sim.UnitFromAngle(b.Heading)
			if b.RoundaboutEntry != nil && b.RoundaboutEntry.Center == rb.Center {
				forward = b.RoundaboutEntry.Forward
			}
			if rb.Center.Sub(b.Pos).Dot(forward) <= 0 || b.Pos.Distance(rb.Center) > rb.Radius+450 {
				continue
			}
			if b != r.owner && r.entered && reservedOwnerLeaving(r.owner, rb) && shortBusClearsReservedEntry(b, r.owner, rb, forward, laneWidth) {
				continue
			}
			r.holds[j] = sim.RoundaboutAdmissionHold{Center: rb.Center, Forward: forward, StopDistance: rb.YieldLineDistance() + b.Length/2 + 8}
			b.RoundaboutAdmissionHold = &r.holds[j]
		}
	}
}

func (tm *TrafficManager) roundaboutReserved(center sim.Vec2) bool {
	for i := range tm.roundaboutReservations {
		r := &tm.roundaboutReservations[i]
		if r.center == center && r.owner != nil {
			return true
		}
	}
	return false
}

// Nearly abreast entrants share one articulated-vehicle reservation. Prefer
// the inner entrance, which does not have to cross the waiting inner bus's
// trajectory. A tiny nose lead in the outer lane must not lock that bus out.
func roundaboutEntryPrecedes(candidate, owner *sim.Bus, center sim.Vec2, along, best float64) bool {
	if owner != nil && owner.RoundaboutEntry != nil && candidate.RoundaboutEntry.Forward.Dot(owner.RoundaboutEntry.Forward) > .99 && math.Abs(along-best) < math.Min(candidate.Length, owner.Length)/2 {
		u := candidate.RoundaboutEntry.Forward
		r := sim.Vec2{X: -u.Y, Y: u.X}
		a, b := candidate.Pos.Sub(center).Dot(r), owner.Pos.Sub(center).Dot(r)
		if a > 0 && b > 0 && math.Abs(a-b) > (candidate.Width+owner.Width)/2 {
			return a < b
		}
	}
	return along < best
}
