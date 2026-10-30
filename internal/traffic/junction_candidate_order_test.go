package traffic

import (
	"fmt"
	"math/rand"
	"slices"
	"testing"
)

func TestOrderedJunctionCandidatesMatchesSortCompact(t *testing.T) {
	rng := rand.New(rand.NewSource(20261027))
	for _, count := range []int{0, 1, 23, 24, 64, 144, 864, 1024, 1152, -1} {
		for _, n := range []int{0, 1, 23, 24, 25, 63, 64, 65, 128, 512, 2048} {
			for trial := 0; trial < 20; trial++ {
				ids := make([]int, n)
				for i := range ids {
					ids[i] = rng.Intn(max(1, count))
					if trial%4 == 0 {
						ids[i] = i % max(1, count)
					}
				}
				if n > 0 && trial%5 == 0 {
					ids[n/2] = -1
				}
				if n > 0 && trial%7 == 0 {
					ids[n-1] = count + 1
				}
				if n > 0 && trial == 19 {
					ids[0] = int(^uint(0) >> 1)
				}
				want := slices.Clone(ids)
				slices.Sort(want)
				want = slices.Compact(want)
				got := orderedJunctionCandidates(ids, count)
				if !slices.Equal(got, want) {
					t.Fatalf("count%d n%d trial%d got%v want%v", count, n, trial, got, want)
				}
			}
		}
	}
}

func BenchmarkJunctionCandidateOrder(b *testing.B) {
	for _, n := range []int{8, 24, 48, 96, 192, 384, 864} {
		rng := rand.New(rand.NewSource(int64(n)))
		input := make([]int, n)
		for i := range input {
			input[i] = rng.Intn(864)
		}
		for _, method := range []string{"sort", "bounded"} {
			b.Run(fmt.Sprintf("n%d/%s", n, method), func(b *testing.B) {
				ids := make([]int, n)
				b.ReportAllocs()
				b.ResetTimer()
				for b.Loop() {
					copy(ids, input)
					if method == "sort" {
						slices.Sort(ids)
						_ = slices.Compact(ids)
					} else {
						_ = orderedJunctionCandidates(ids, 864)
					}
				}
			})
		}
	}
}
