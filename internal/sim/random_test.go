package sim

import (
	"testing"
)

func TestSeedSimulationDeterminism(t *testing.T) {
	SeedSimulation(42)
	v1 := RandomFloat64()
	v2 := RandomIntn(100)
	v3 := RandomFloat64()

	// Re-seed with identical value
	SeedSimulation(42)
	r1 := RandomFloat64()
	r2 := RandomIntn(100)
	r3 := RandomFloat64()

	if v1 != r1 || v2 != r2 || v3 != r3 {
		t.Fatalf("SeedSimulation failed to produce deterministic random sequence: (%f,%d,%f) vs (%f,%d,%f)",
			v1, v2, v3, r1, r2, r3)
	}
}
