package game

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
