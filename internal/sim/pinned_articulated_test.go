package sim

import (
	"math"
	"testing"
)

func TestPinnedArticulatedRearGap_BentTrailerAndCorridorClip(t *testing.T) {
	// Articulated leader pulling out of a curbside bay (turned left relative to
	// southbound heading pi/2), with its trailer also rotated toward the bay so
	// an outer rear corner extends past the follower's front bumper outside the
	// follower's lane corridor.
	leader := NewBus(11486.79, 4586.19, 2.1734)
	leader.SetModel(BusArticulated)
	leader.TrailerHeading = 2.6136
	leader.TrailerInitialized = true

	followerPos := Vec2{X: 11469.97, Y: 4498.22}
	gap, ok := PinnedArticulatedRearGap(leader, followerPos, math.Pi/2, 48, 18, 26)
	if !ok {
		t.Fatalf("expected pinned leader match for bent-trailer departure, got ok=false")
	}
	if gap < -48 || gap >= 10 {
		t.Fatalf("unexpected clipped gap %.2f, want in [-48, 10)", gap)
	}

	// Parallel articulated bus in an adjacent lane alongside the follower must
	// not trigger follower retreat.
	alongside := NewBus(11522.0, 4498.22, math.Pi/2+0.25)
	alongside.SetModel(BusArticulated)
	if g, ok := PinnedArticulatedRearGap(alongside, followerPos, math.Pi/2, 48, 18, 26); ok {
		t.Fatalf("parallel bus alongside follower should not trigger retreat (gap=%.2f)", g)
	}

	// Unbent articulated leader aligned with the road is normal car-following,
	// not a pinned curb/junction departure.
	straight := NewBus(11470.0, 4610.0, math.Pi/2)
	straight.SetModel(BusArticulated)
	if g, ok := PinnedArticulatedRearGap(straight, followerPos, math.Pi/2, 48, 18, 26); ok {
		t.Fatalf("straight leader should not match pinned retreat (gap=%.2f)", g)
	}
}
