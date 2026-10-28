package sim

import (
	"math"
	"math/bits"
	"testing"
	"unsafe"
)

func predictionTestHash(b *Bus, p Vec2, h float64) uint64 {
	hash := uint64(0x9e3779b97f4a7c15)
	for _, v := range [8]float64{b.Pos.X, b.Pos.Y, b.Heading, b.TrailerHeading, b.Length, p.X, p.Y, h} {
		hash = bits.RotateLeft64(hash^math.Float64bits(v), 11) * 0x9e3779b97f4a7c15
	}
	if b.TrailerInitialized {
		hash ^= 0x517cc1b727220a95
	}
	return hash
}

// Use the bit that formerly collided in the 128-slot table. Capacity changes
// must preserve exact hits while ordinary same-slot replacement still works.
func TestPosePredictionExpandedCacheCollisionAndReplacement(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.SetModel(BusArticulated)
	b.PreparePosePredictions()
	var first, second, replace Vec2
	foundA, foundB, foundC := false, false, false
	for i := 1; i < 100000 && !(foundA && foundB && foundC); i++ {
		p := Vec2{X: float64(i) / 1000}
		switch predictionTestHash(b, p, 0) & 255 {
		case 0:
			if !foundA {
				first, foundA = p, true
			} else if !foundC {
				replace, foundC = p, true
			}
		case 128:
			if !foundB {
				second, foundB = p, true
			}
		}
	}
	if !(foundA && foundB && foundC) {
		t.Fatal("missing colliding keys")
	}
	assertPosePrediction(t, b, first, 0)
	assertPosePrediction(t, b, second, 0)
	h, m := b.PosePredictionStats()
	assertPosePrediction(t, b, first, 0)
	nh, nm := b.PosePredictionStats()
	if nh != h+1 || nm != m {
		t.Fatal("old-table collision still evicted a prediction")
	}
	assertPosePrediction(t, b, replace, 0)
	h, m = b.PosePredictionStats()
	assertPosePrediction(t, b, second, 0)
	nh, nm = b.PosePredictionStats()
	if nh != h+1 || nm != m {
		t.Fatal("replacement changed an unrelated slot")
	}
	assertPosePrediction(t, b, first, 0)
	_, nm = b.PosePredictionStats()
	if nm != m+1 {
		t.Fatal("same-slot entry was not replaced")
	}
	t.Logf("prediction cache bytes: %d", unsafe.Sizeof(*b.predictionCache))
	if len(b.predictionCache.entries) != 256 || unsafe.Sizeof(*b.predictionCache) > 32<<10 {
		t.Fatal("cache exceeded its fixed memory bound")
	}
}

func TestPosePredictionCacheRawBitKeys(t *testing.T) {
	b := NewBus(0, 0, 0)
	b.SetModel(BusArticulated)
	b.PreparePosePredictions()
	for _, v := range []uint64{0, 1 << 63, 0x7ff0000000000000, 0xfff0000000000000, 0x7ff8000000000001, 0x7ff8000000000002} {
		p := Vec2{X: math.Float64frombits(v)}
		entry, hit := b.predictionEntry(p, 0)
		if hit {
			t.Fatal("new bit pattern reused another key")
		}
		entry.pose = busPredictionPose{pos: p}
		entry.valid = true
		again, hit := b.predictionEntry(p, 0)
		if !hit || math.Float64bits(again.pose.pos.X) != v {
			t.Fatal("bitwise key was not retained")
		}
	}
}
