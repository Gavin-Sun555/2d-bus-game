package sim

import (
	"fmt"
	"math"
	"testing"
)

func TestFleetBusMergeYieldReservesOnlySafeRearSlots(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, scenario := range []string{"stopped rear", "intermediate rear", "fast intermediate rear", "alongside intermediate", "rear safety margin", "offset rear body", "offset rear too close", "yawed bay exit rear", "yawed fast rear", "yawed alongside", "safe moving rear", "fast close rear", "alongside", "oncoming", "other lane", "already joined", "expired", "yawed merging leader", "continuing yawed leader", "turning bus"} {
			t.Run(fmt.Sprintf("h%.2f/%s", h, scenario), func(t *testing.T) {
				u := Vec2{X: math.Cos(h), Y: math.Sin(h)}
				r := Vec2{X: -u.Y, Y: u.X}
				merging := NewBus(0, 0, h)
				merging.Pos = r.Mul(8)
				merging.LaneMergeRequest = &BusLaneMergeRequest{Target: r.Mul(26), Heading: h, Remaining: .3, Is2Lane: true}
				follower := NewBus(0, 0, h)
				follower.Pos = r.Mul(26).Sub(u.Mul((merging.Length+follower.Length)/2 + 26))
				want := false
				switch scenario {
				case "stopped rear":
					want = true
				case "intermediate rear":
					merging.LaneMergeRequest.Target = r.Mul(130)
					want = true
				case "fast intermediate rear":
					merging.LaneMergeRequest.Target = r.Mul(130)
					follower.Speed = 150
				case "alongside intermediate":
					merging.LaneMergeRequest.Target = r.Mul(130)
					follower.Pos = r.Mul(26)
				case "yawed bay exit rear":
					follower.Heading += .205
					follower.Pos = follower.Pos.Sub(u.Mul(20)).Add(r.Mul(25))
					want = true
				case "yawed fast rear":
					follower.Heading += .205
					follower.Pos = follower.Pos.Sub(u.Mul(200))
					follower.Speed = 80
				case "yawed alongside":
					follower.Heading += .205
					follower.Pos = r.Mul(26)
				case "rear safety margin":
					follower.Pos = follower.Pos.Add(r.Mul(44))
					want = true
				case "offset rear body":
					follower.Pos = follower.Pos.Add(r.Mul(25))
					want = true
				case "offset rear too close":
					follower.Pos = follower.Pos.Add(r.Mul(25)).Add(u.Mul(20))
				case "safe moving rear":
					follower.Pos = follower.Pos.Sub(u.Mul(160))
					follower.Speed = 80
					want = true
				case "fast close rear":
					follower.Speed = 150
				case "alongside":
					follower.Pos = r.Mul(26)
				case "oncoming":
					follower.Heading += math.Pi
				case "other lane":
					follower.Pos = follower.Pos.Add(r.Mul(52))
				case "already joined":
					merging.LaneMergeRequest.Joined = true
				case "expired":
					merging.LaneMergeRequest.Remaining = 0
				case "continuing yawed leader":
					merging.Heading += .9
					follower.Pos = follower.Pos.Sub(u.Mul(50))
					want = true
				case "yawed merging leader":
					merging.Heading += .5
					want = true
				case "turning bus":
					merging.Heading += math.Pi / 3
				}
				limit, ok := BusMergeYieldLimit(follower, merging, 52, scenario == "continuing yawed leader")
				if ok != want {
					t.Fatalf("yield=%v want%v limit%.1f", ok, want, limit)
				}
				if scenario == "stopped rear" && limit != 0 {
					t.Fatal("follower would close granted gap")
				}
			})
		}
	}
}
