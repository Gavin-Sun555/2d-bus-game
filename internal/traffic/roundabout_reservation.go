package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

type roundaboutReservation struct {
	center       sim.Vec2
	owner        *sim.Bus
	entered      bool
	lastApproach sim.Vec2
	// staging: the owner is still approaching. Cars and short buses keep
	// using the ring until it nears the line; long buses stay held.
	staging bool
	holds   [1 + MaxFleetBuses]sim.RoundaboutAdmissionHold
}

// Long vehicles need the complete ring width while their trailer turns.
// Drain already-admitted cars before admitting one, then hold new arrivals
// until the last section is clear. Short buses may use a separated entrance
// once the owner begins leaving. All motion and collision rules still run.
func (tm *TrafficManager) updateRoundaboutReservations(player *sim.Bus, w *sim.World, count int, laneWidth float64) {
	var storage [1 + MaxFleetBuses]*sim.Bus
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
			if !r.entered && inside(r.owner) {
				r.entered = true
				if intent := r.owner.RoundaboutEntry; intent != nil && intent.Center == rb.Center {
					r.lastApproach = intent.Forward
				}
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
				bestApproach := math.Inf(1)
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
					if along <= 0 {
						continue
					}
					// Rotate between approaches with waiting buses. Choosing
					// solely by distance favors the closest yield line forever
					// when another approach has a car ahead of its first bus.
					approach := roundaboutApproachOrder(r.lastApproach, intent.Forward)
					if approach < bestApproach || (approach == bestApproach && roundaboutEntryPrecedes(b, r.owner, rb.Center, along, best)) {
						best = along
						bestApproach = approach
						r.owner = b
						r.entered = false
					}
				}
			}
		}
		if r.owner == nil {
			r.staging = false
			continue
		}
		r.staging = !r.entered && !inside(r.owner) && reservationOwnerStaging(r.owner, rb)
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
			if r.staging && !b.IsArticulated() {
				continue
			}
			stop := rb.YieldLineDistance() + b.Length/2 + 8
			if b != r.owner && b.IsArticulated() && !r.entered {
				// Wait behind the owner's tail, not beside it: an owner
				// still angling into this lane sweeps its body across it.
				stop = math.Max(stop, ownerTailDistance(r.owner, rb, forward)+b.Length/2+20)
			}
			r.holds[j] = sim.RoundaboutAdmissionHold{Center: rb.Center, Forward: forward, StopDistance: stop}
			b.RoundaboutAdmissionHold = &r.holds[j]
		}
	}
}

// ownerTailDistance is how far the owner's rearmost body point lies from the
// roundabout center along an approach, or 0 when it is on another approach.
func ownerTailDistance(owner *sim.Bus, rb sim.Roundabout, forward sim.Vec2) float64 {
	entry := owner.RoundaboutEntry
	if entry == nil || entry.Center != rb.Center || entry.Forward.Dot(forward) < .99 {
		return 0
	}
	heading := math.Atan2(forward.Y, forward.X)
	tail := 0.
	for _, part := range owner.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		yaw := part.Heading - heading
		extent := part.HalfLen*math.Abs(math.Cos(yaw)) + part.HalfWid*math.Abs(math.Sin(yaw))
		tail = math.Max(tail, rb.Center.Sub(part.Center).Dot(forward)+extent)
	}
	return tail
}

func roundaboutApproachOrder(previous, next sim.Vec2) float64 {
	if previous == (sim.Vec2{}) {
		return 0
	}
	angle := math.Atan2(previous.X*next.Y-previous.Y*next.X, previous.Dot(next))
	quarterTurns := math.Round(angle / (math.Pi / 2))
	if quarterTurns <= 0 {
		quarterTurns += 4
	}
	return quarterTurns
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

// A reservation can't let its owner pass a car already queued in front of
// it. Let that car use the ordinary yield rules, then drain it from the ring
// before admitting the reserving bus.
func (tm *TrafficManager) roundaboutReservedForCar(rb sim.Roundabout, car *sim.TrafficCar, forward sim.Vec2, laneWidth float64) bool {
	for i := range tm.roundaboutReservations {
		r := &tm.roundaboutReservations[i]
		if r.center != rb.Center || r.owner == nil {
			continue
		}
		if r.staging {
			return false
		}
		intent := r.owner.RoundaboutEntry
		if r.entered || intent == nil || intent.Center != rb.Center || intent.Forward.Dot(forward) < .99 {
			return true
		}
		delta := car.Pos.Sub(r.owner.Pos)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		if ownerTurnedIntoCarLane(r.owner, car, forward, laneWidth) {
			return false
		}
		return math.Abs(delta.Dot(right)) >= laneWidth/2 || !roundaboutBusFollowing(car, r.owner)
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

// A stopped reservation owner angled into the neighbouring lane cannot pass a
// car queued ahead of it there, and that car would wait for the owner's
// reservation forever. Let it take its turn like a car ahead in the owner's
// own lane. An owner lying straight in its lane, simply waiting for the ring
// to drain, keeps holding the next lane so it cannot be starved.
func ownerTurnedIntoCarLane(owner *sim.Bus, car *sim.TrafficCar, forward sim.Vec2, laneWidth float64) bool {
	if math.Abs(owner.Speed) >= 1 {
		return false
	}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	delta := car.Pos.Sub(owner.Pos)
	lateral := delta.Dot(right)
	yaw := sim.NormalizeAngle(owner.Heading - math.Atan2(forward.Y, forward.X))
	return delta.Dot(forward) > 0 && math.Abs(lateral) >= laneWidth/2 && math.Abs(lateral) < 1.5*laneWidth && math.Abs(yaw) > .1 && yaw*lateral > 0
}

// reservationOwnerStaging reports whether the reserving bus is still far
// enough from its yield line to brake there comfortably. Holding every
// entrance from the moment a long bus is chosen, often 500 px out and queued,
// kept the ring closed several times longer than the bus spent in it.
// Close the entrances only when it arrives; the admission hold's braking
// curve still stops it at the line until the ring has drained.
func reservationOwnerStaging(owner *sim.Bus, rb sim.Roundabout) bool {
	forward := sim.UnitFromAngle(owner.Heading)
	if owner.RoundaboutEntry != nil && owner.RoundaboutEntry.Center == rb.Center {
		forward = owner.RoundaboutEntry.Forward
	}
	gap := rb.Center.Sub(owner.Pos).Dot(forward) - rb.YieldLineDistance() - owner.Length/2 - 8
	speed := math.Abs(owner.Speed)
	return gap > math.Max(90, speed*speed/(2*180)+60)
}
