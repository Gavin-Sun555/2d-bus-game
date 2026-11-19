package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

type roundaboutApproachSlot struct {
	forward    sim.Vec2
	owner      *sim.Bus
	entered    bool
	staging    bool
	stallTime  float64
	deferBuses [1 + MaxFleetBuses]*sim.Bus
	deferLeft  [1 + MaxFleetBuses]float64
}

type roundaboutReservation struct {
	center     sim.Vec2
	approaches [4]roundaboutApproachSlot
	holds      [1 + MaxFleetBuses]sim.RoundaboutAdmissionHold
}

func (a *roundaboutApproachSlot) tickDefers(dt float64) {
	for i := range a.deferLeft {
		if a.deferLeft[i] > 0 {
			a.deferLeft[i] -= dt
			if a.deferLeft[i] <= 0 {
				a.deferLeft[i] = 0
				a.deferBuses[i] = nil
			}
		}
	}
}

func (a *roundaboutApproachSlot) isDeferred(b *sim.Bus) bool {
	if b == nil {
		return false
	}
	for i, db := range a.deferBuses {
		if db == b && a.deferLeft[i] > 0 {
			return true
		}
	}
	return false
}

func (a *roundaboutApproachSlot) deferBus(b *sim.Bus, dur float64) {
	if b == nil || dur <= 0 {
		return
	}
	free := -1
	for i, db := range a.deferBuses {
		if db == b {
			a.deferLeft[i] = dur
			return
		}
		if (db == nil || a.deferLeft[i] <= 0) && free < 0 {
			free = i
		}
	}
	if free >= 0 {
		a.deferBuses[free] = b
		a.deferLeft[free] = dur
	}
}

func (r *roundaboutReservation) approachSlot(forward sim.Vec2) *roundaboutApproachSlot {
	empty := -1
	idle := -1
	for k := range r.approaches {
		slot := &r.approaches[k]
		if slot.forward == (sim.Vec2{}) {
			if empty < 0 {
				empty = k
			}
			continue
		}
		if slot.forward.Dot(forward) > 0.9 {
			return slot
		}
		if slot.owner == nil && !slot.hasDefers() && idle < 0 {
			idle = k
		}
	}
	target := empty
	if target < 0 {
		target = idle
	}
	if target < 0 {
		target = 0
	}
	r.approaches[target] = roundaboutApproachSlot{forward: forward}
	return &r.approaches[target]
}

func (a *roundaboutApproachSlot) hasDefers() bool {
	for _, left := range a.deferLeft {
		if left > 0 {
			return true
		}
	}
	return false
}

// Coordinate articulated buses per approach mouth rather than locking the entire
// roundabout globally. Multiple approaches enter and circulate concurrently using
// local yield/headway rules, while same-approach articulated buses serialize across
// the approach mouth only until the leader's trailer clears into the ring.
func (tm *TrafficManager) updateRoundaboutReservations(player *sim.Bus, w *sim.World, count int, laneWidth float64) {
	tm.updateRoundaboutReservationsAt(1.0/60.0, player, w, count, laneWidth)
}

func (tm *TrafficManager) updateRoundaboutReservationsAt(dt float64, player *sim.Bus, w *sim.World, count int, laneWidth float64) {
	if dt <= 0 {
		dt = 1.0 / 60.0
	}
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
		isPresent := func(target *sim.Bus) bool {
			for _, b := range buses {
				if b == target {
					return true
				}
			}
			return false
		}

		// Ensure each approach with a waiting articulated bus has a slot.
		for _, b := range buses {
			if !b.IsArticulated() {
				continue
			}
			intent := b.RoundaboutEntry
			if intent == nil || intent.Remaining <= 0 || intent.Center != rb.Center || b.Pos.Distance(rb.Center) >= rb.Radius+400 {
				continue
			}
			if rb.Center.Sub(b.Pos).Dot(intent.Forward) <= 0 {
				continue
			}
			r.approachSlot(intent.Forward)
		}

		for k := range r.approaches {
			app := &r.approaches[k]
			app.tickDefers(dt)
			if app.forward == (sim.Vec2{}) {
				continue
			}
			prevOwner := app.owner
			if app.owner != nil && !isPresent(app.owner) {
				app.owner = nil
				app.entered = false
			}
			if app.owner != nil {
				if !app.entered && inside(app.owner) {
					app.entered = true
				}
				if app.entered && !ownerStillAtMouth(app.owner, rb, app.forward, laneWidth) {
					app.owner = nil
					app.entered = false
				}
			}
			if app.owner != nil && !app.entered {
				intent := app.owner.RoundaboutEntry
				if intent == nil || intent.Remaining <= 0 || intent.Center != rb.Center || intent.Forward.Dot(app.forward) < 0.9 || app.owner.Pos.Distance(rb.Center) > rb.Radius+450 {
					app.owner = nil
				} else {
					best := rb.Center.Sub(app.owner.Pos).Dot(app.forward)
					for _, b := range buses {
						if app.isDeferred(b) {
							continue
						}
						entry := b.RoundaboutEntry
						if entry == nil || entry.Remaining <= 0 || entry.Center != rb.Center || entry.Forward.Dot(app.forward) < .99 {
							continue
						}
						along := rb.Center.Sub(b.Pos).Dot(entry.Forward)
						if along > 0 && roundaboutEntryPrecedes(b, app.owner, rb.Center, along, best) {
							app.owner = b
							best = along
						}
					}
				}
			}
			if app.owner == nil {
				longWaiting := false
				for _, b := range buses {
					if !b.IsArticulated() || app.isDeferred(b) {
						continue
					}
					intent := b.RoundaboutEntry
					if intent != nil && intent.Remaining > 0 && intent.Center == rb.Center && intent.Forward.Dot(app.forward) > .99 && b.Pos.Distance(rb.Center) < rb.Radius+400 && rb.Center.Sub(b.Pos).Dot(intent.Forward) > 0 {
						longWaiting = true
						break
					}
				}
				if longWaiting {
					best := math.Inf(1)
					for _, b := range buses {
						if app.isDeferred(b) {
							continue
						}
						intent := b.RoundaboutEntry
						if intent == nil || intent.Remaining <= 0 || intent.Center != rb.Center || intent.Forward.Dot(app.forward) < .99 || b.Pos.Distance(rb.Center) > rb.Radius+400 {
							continue
						}
						along := rb.Center.Sub(b.Pos).Dot(intent.Forward)
						if along <= 0 {
							continue
						}
						if roundaboutEntryPrecedes(b, app.owner, rb.Center, along, best) {
							best = along
							app.owner = b
							app.entered = inside(b)
						}
					}
				}
			}
			if app.owner == nil {
				app.staging = false
				app.stallTime = 0
				continue
			}
			app.staging = !app.entered && !inside(app.owner) && reservationOwnerStaging(app.owner, rb)
			if app.owner == prevOwner && !app.entered && !app.staging {
				if math.Abs(app.owner.Speed) < 2.0 {
					app.stallTime += dt
				} else {
					app.stallTime = 0
				}
				if app.stallTime >= 4.5 {
					app.deferBus(app.owner, 6.0)
					app.owner = nil
					app.entered = false
					app.staging = false
					app.stallTime = 0
					continue
				}
			} else {
				app.stallTime = 0
			}

			for j, b := range buses {
				if b == app.owner || inside(b) {
					continue
				}
				forward := sim.UnitFromAngle(b.Heading)
				if b.RoundaboutEntry != nil && b.RoundaboutEntry.Center == rb.Center {
					forward = b.RoundaboutEntry.Forward
				}
				if forward.Dot(app.forward) < 0.99 {
					continue
				}
				if rb.Center.Sub(b.Pos).Dot(forward) <= 0 || b.Pos.Distance(rb.Center) > rb.Radius+450 {
					continue
				}
				if app.staging && !b.IsArticulated() {
					continue
				}
				if !b.IsArticulated() && app.entered && shortBusClearsReservedEntry(b, app.owner, rb, forward, laneWidth) {
					continue
				}
				stop := rb.YieldLineDistance() + b.Length/2 + 8
				if b.IsArticulated() {
					// Keep trailing same-approach articulated buses behind the owner's
					// sweeping trailer until the trailer clears the approach mouth.
					stop = math.Max(stop, ownerTailDistanceOnApproach(app.owner, rb, forward)+b.Length/2+20)
				}
				r.holds[j] = sim.RoundaboutAdmissionHold{Center: rb.Center, Forward: forward, StopDistance: stop}
				b.RoundaboutAdmissionHold = &r.holds[j]
			}
		}
	}
}

// ownerStillAtMouth reports whether an already-entered owner still has any body
// section straddling its incoming approach mouth (rather than cleared into the ring).
func ownerStillAtMouth(owner *sim.Bus, rb sim.Roundabout, forward sim.Vec2, laneWidth float64) bool {
	if owner == nil {
		return false
	}
	heading := math.Atan2(forward.Y, forward.X)
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	for _, part := range owner.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		if part.Center.Distance(rb.Center) > rb.Radius+owner.OverallLength()+60 {
			continue
		}
		rel := part.Center.Sub(rb.Center)
		if rel.Dot(right) <= -laneWidth*0.5 {
			// Already on the outgoing half of the roadway.
			continue
		}
		yaw := part.Heading - heading
		extent := part.HalfLen*math.Abs(math.Cos(yaw)) + part.HalfWid*math.Abs(math.Sin(yaw))
		tail := rb.Center.Sub(part.Center).Dot(forward) + extent
		if tail > rb.Radius+16 {
			return true
		}
	}
	return false
}

// ownerTailDistanceOnApproach returns how far the owner's rearmost body point
// lies from the roundabout center along a given incoming direction.
func ownerTailDistanceOnApproach(owner *sim.Bus, rb sim.Roundabout, forward sim.Vec2) float64 {
	if owner == nil {
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

func (tm *TrafficManager) roundaboutReserved(center sim.Vec2) bool {
	for i := range tm.roundaboutReservations {
		r := &tm.roundaboutReservations[i]
		if r.center != center {
			continue
		}
		for k := range r.approaches {
			if r.approaches[k].owner != nil {
				return true
			}
		}
	}
	return false
}

// Restrict car reservation to the same approach as an entering articulated bus
// while its body is crossing that approach mouth. Other approaches remain free
// to enter concurrently under ordinary roundabout yield rules.
func (tm *TrafficManager) roundaboutReservedForCar(rb sim.Roundabout, car *sim.TrafficCar, forward sim.Vec2, laneWidth float64) bool {
	for i := range tm.roundaboutReservations {
		r := &tm.roundaboutReservations[i]
		if r.center != rb.Center {
			continue
		}
		for k := range r.approaches {
			app := &r.approaches[k]
			if app.owner == nil || !app.owner.IsArticulated() || app.staging {
				continue
			}
			appFwd := app.forward
			if appFwd == (sim.Vec2{}) {
				if intent := app.owner.RoundaboutEntry; intent != nil && intent.Center == rb.Center {
					appFwd = intent.Forward
				} else {
					appFwd = sim.UnitFromAngle(app.owner.Heading)
				}
			}
			if appFwd.Dot(forward) < .99 {
				continue
			}
			if app.entered {
				if ownerStillAtMouth(app.owner, rb, appFwd, laneWidth) {
					return true
				}
				continue
			}
			delta := car.Pos.Sub(app.owner.Pos)
			right := sim.Vec2{X: -forward.Y, Y: forward.X}
			if ownerTurnedIntoCarLane(app.owner, car, forward, laneWidth) {
				continue
			}
			if math.Abs(delta.Dot(right)) >= laneWidth/2 || !roundaboutBusFollowing(car, app.owner) {
				return true
			}
		}
	}
	return false
}

// Nearly abreast entrants share one articulated-vehicle reservation. Prefer
// the inner entrance, which does not have to cross the waiting inner bus's
// trajectory. A tiny nose lead in the outer lane must not lock that bus out.
func roundaboutEntryPrecedes(candidate, owner *sim.Bus, center sim.Vec2, along, best float64) bool {
	if owner != nil && owner.RoundaboutEntry != nil && candidate.RoundaboutEntry.Forward.Dot(owner.RoundaboutEntry.Forward) > .99 && math.Abs(along-best) < 12.0 {
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
