package sim

import (
	"math"
	"testing"
)

func TestJunctionFollowingRequiresMatchingLaneAndApproach(t *testing.T) {
	points := LeftTurnPath(Vec2{}, 0, 52, 216)
	ours := &JunctionCorridor{Remaining: .25, EntryHeading: 0, Bodies: AppendPathBodies(nil, points, 96, 36)}
	theirs := &JunctionCorridor{Remaining: .25, EntryHeading: 0, Bodies: AppendPathBodies(nil, points, 48, 22)}
	body, ok := PredictJunctionBody(theirs, 180)
	if !ok {
		t.Fatal("no prediction")
	}
	if gap, ok := JunctionLeaderGap(ours, theirs, body, 96, 36); !ok || gap < 90 {
		t.Fatalf("same turning lane not recognized: gap%.1f ok%v", gap, ok)
	}
	if speed := JunctionFollowingSpeed(120, 85, 85, 130); speed < 85 {
		t.Fatal("unnecessary brake for moving leader")
	}
	if speed := JunctionFollowingSpeed(12, 0, 85, 130); speed != 0 {
		t.Fatal("stopped leader unprotected")
	}
	cases := []struct {
		name string
		edit func(*JunctionCorridor, *OBB)
	}{
		{"opposing approach", func(c *JunctionCorridor, _ *OBB) { c.EntryHeading = math.Pi }},
		{"different junction", func(c *JunctionCorridor, _ *OBB) { c.Center.X = 2000 }},
		{"opposite yaw", func(_ *JunctionCorridor, b *OBB) { b.Heading += math.Pi }},
		{"adjacent lane", func(c *JunctionCorridor, b *OBB) {
			for i := range c.Bodies {
				c.Bodies[i].Center.X += 52
			}
			b.Center.X += 52
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			c := *theirs
			c.Bodies = append([]OBB(nil), theirs.Bodies...)
			b := body
			tc.edit(&c, &b)
			if _, ok := JunctionLeaderGap(ours, &c, b, 96, 36); ok {
				t.Fatal("conflicting movement treated as leader")
			}
		})
	}
}

func TestPublishedJunctionGeometryMatchesLiveBodies(t *testing.T) {
	points := LeftTurnPath(Vec2{X: 1280, Y: -630}, .37, 52, 216)
	original := JunctionCorridor{Remaining: .25, EntryHeading: .37, Bodies: AppendPathBodies(nil, points, 96, 36)}
	cached := original
	cached.RebuildBounds()
	original.Min, original.Max = cached.Min, cached.Max
	for distance := -5.; distance <= 900; distance += 2.5 {
		got, ok := PredictJunctionBody(&cached, distance)
		want, liveOK := PredictJunctionBody(&original, distance)
		if got != want || ok != liveOK {
			t.Fatalf("prediction changed at distance %.1f", distance)
		}
		gap, ok := JunctionLeaderGap(&cached, &cached, got, 96, 36)
		wantGap, wantOK := JunctionLeaderGap(&original, &original, want, 96, 36)
		if gap != wantGap || ok != wantOK {
			t.Fatalf("headway changed at distance %.1f", distance)
		}
	}

	copied := cached
	copied.Bodies = append([]OBB(nil), cached.Bodies...)
	for i := range copied.Bodies {
		copied.Bodies[i].Center.X += float64(i) * .8
	}
	copied.RebuildBounds()
	liveCopy := JunctionCorridor{Remaining: copied.Remaining, Bodies: copied.Bodies, Min: copied.Min, Max: copied.Max}
	for i := 0; i < 2; i++ {
		got, ok := PredictJunctionBody(&copied, 140)
		want, liveOK := PredictJunctionBody(&liveCopy, 140)
		if got != want || ok != liveOK {
			t.Fatal("republishing copied bodies kept stale forecast geometry")
		}
		got, ok = PredictJunctionBody(&cached, 140)
		want, liveOK = PredictJunctionBody(&original, 140)
		if got != want || ok != liveOK {
			t.Fatal("republishing copied bodies mutated the original forecast")
		}
		if SeparatedJunctionCorridors(&cached, &copied) != SeparatedJunctionCorridors(&original, &liveCopy) {
			t.Fatal("reused corridor groups changed separation")
		}
	}
}
