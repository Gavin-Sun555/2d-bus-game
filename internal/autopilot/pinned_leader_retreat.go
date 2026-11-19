package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A stuck articulated bus with its powered section turned toward the curb,
// on a road or just out of a junction turn, can only recover by backing its
// tail a few pixels. A follower stopped
// against that tail leaves it no room and both wait forever. After a short
// wait, give the leader that room with a proved straight retreat.
func (ad *AutopilotDriver) tryPinnedLeaderRetreat(f autopilotFrame, dt float64) bool {
	if f.world == nil || f.speedAbs > .1 || f.bus.DoorsOpen || ad.headonRecovery != nil || ad.State == AutopilotStateDocked {
		ad.pinnedLeaderWait = 0
		return false
	}
	road := approachRoadHeading(f.bus.Heading)
	pinned, side := false, false
	if f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.OverallLength()/2) {
		pinned = junctionPinnedLeader(f)
	} else {
		if math.Cos(f.bus.Heading-road) > .98 && math.Cos(f.bus.TailHeading()-road) > .98 {
			for _, ob := range f.obstacles {
				if gap, ok := sim.PinnedArticulatedRearGap(ob.Bus, f.bus.Pos, road, f.bus.Length/2, f.bus.Width/2, f.laneWidth*.5); ok && gap < 10 {
					pinned = true
					break
				}
			}
		}
		if !pinned && math.Abs(sim.NormalizeAngle(f.bus.Heading-road)) <= .8 && math.Cos(f.bus.Heading-f.bus.TailHeading()) > .96 {
			pinned = sideContactPinned(f, road)
			side = pinned
		}
	}
	// The more angled of two touching buses backs out first: reversing along
	// its own heading draws its nose away from the other. The straighter
	// one steps back only if that has not freed them.
	sideWait := 3.
	if side {
		if other := sideContactNeighbor(f, road); other != nil &&
			math.Abs(sim.NormalizeAngle(f.bus.Heading-road)) <= math.Abs(sim.NormalizeAngle(other.Heading-road)) {
			sideWait = 8
		}
	}
	if !pinned {
		ad.pinnedLeaderWait = 0
		return false
	}
	ad.pinnedLeaderWait += dt
	if ad.pinnedLeaderWait < sideWait {
		return false
	}
	distances := []float64{20, 14}
	if side {
		// Back far enough for the angled neighbor's nose to clear this
		// bus's side; shorter retreats are tried when traffic is close behind.
		distances = []float64{60, 40, 20, 14}
	}
	for _, distance := range distances {
		if ad.laneRetreatReverseClear(f, distance, 0) {
			ad.pinnedLeaderWait = 0
			if side {
				ad.sideYield = &sideYieldHold{neighbor: sideContactNeighbor(f, road), remaining: sideYieldSeconds}
				if ad.sideYield.neighbor != nil {
					ad.sideYield.anchor = ad.sideYield.neighbor.Pos
				}
			}
			ad.headonRecovery = &HeadonDeadlockRecovery{StartPos: f.bus.Pos, TargetDistance: distance, ParallelRetreat: true}
			return true
		}
	}
	return false
}

// Two buses turning out of a junction: the leader can overshoot its exit
// toward the curb with the follower's nose tucked against its trailer. Mid-turn
// the road axis cannot tell "behind" from "alongside", so test contact with
// the follower's own straight front section directly.
func junctionPinnedLeader(f autopilotFrame) bool {
	if math.Cos(f.bus.Heading-f.bus.TailHeading()) < .98 {
		return false
	}
	u := sim.UnitFromAngle(f.bus.Heading)
	front := sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length/2 + 8, HalfWid: f.bus.Width/2 + 6}
	for _, ob := range f.obstacles {
		leader := ob.Bus
		if leader == nil || !leader.IsArticulated() || math.Abs(leader.Speed) >= 1 || leader.Pos.Sub(f.bus.Pos).Dot(u) <= 0 {
			continue
		}
		for _, part := range leader.BodyParts() {
			if part.HalfLen <= 0 {
				continue
			}
			if hit, _, _ := sim.CheckOBBCollision(front, part); hit {
				return true
			}
		}
	}
	return false
}

// A straight bus stopped with its side touching another stopped bus that is
// angled across its lane (for example two long buses beside a platform): the
// angled bus cannot turn back without swinging through this one, and every
// forward move of either is blocked. The straight bus backs off to give it
// that room.
func sideContactPinned(f autopilotFrame, road float64) bool {
	return sideContactNeighbor(f, road) != nil
}

// sideYieldSeconds bounds how long a bus that backed away from an angled
// neighbor waits for it before resuming its own approach.
const sideYieldSeconds = 12.0

type sideYieldHold struct {
	neighbor  *sim.Bus
	anchor    sim.Vec2
	remaining float64
}

// holdingSideYield keeps a bus that backed away from an angled neighbor in
// place until that neighbor has moved off or the wait expires. Creeping
// forward again would only restore the contact.
func (ad *AutopilotDriver) holdingSideYield(dt float64) bool {
	h := ad.sideYield
	if h == nil {
		return false
	}
	h.remaining -= dt
	if h.remaining <= 0 || h.neighbor == nil || h.neighbor.Pos.Distance(h.anchor) > 20 || h.neighbor.DoorsOpen {
		ad.sideYield = nil
		return false
	}
	return true
}

func sideContactNeighbor(f autopilotFrame, road float64) *sim.Bus {
	var own [2]sim.OBB
	n := 0
	for _, part := range f.bus.BodyParts() {
		if part.HalfLen > 0 {
			part.HalfLen += 2
			part.HalfWid += 3
			own[n] = part
			n++
		}
	}
	for _, ob := range f.obstacles {
		other := ob.Bus
		if other == nil || math.Abs(other.Speed) >= 1 || other.DoorsOpen {
			continue
		}
		yaw := math.Abs(sim.NormalizeAngle(other.Heading - road))
		if math.Abs(sim.NormalizeAngle(other.Heading-f.bus.Heading)) < .1 || yaw > .8 {
			continue
		}
		for _, mine := range own[:n] {
			for _, part := range other.BodyParts() {
				if part.HalfLen <= 0 {
					continue
				}
				if hit, _, _ := sim.CheckOBBCollision(mine, part); hit {
					return other
				}
			}
		}
	}
	return nil
}
