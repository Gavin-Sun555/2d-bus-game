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
