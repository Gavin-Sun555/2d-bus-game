package sim

import (
	"math/rand"
	"time"
)

// simRand is the central RNG for gameplay simulation (civilian vehicle speeds,
// passenger generation, alighting requests, and camera shake).
// It can be deterministically seeded via SeedSimulation for automated testing.
var simRand = rand.New(rand.NewSource(time.Now().UnixNano()))

// SeedSimulation re-seeds the simulation RNG for deterministic testing.
func SeedSimulation(seed int64) {
	simRand = rand.New(rand.NewSource(seed))
}

// RandomIntn samples from the simulation's shared, seedable random stream.
func RandomIntn(n int) int { return simRand.Intn(n) }

// RandomFloat64 samples from the simulation's shared, seedable random stream.
func RandomFloat64() float64 { return simRand.Float64() }
