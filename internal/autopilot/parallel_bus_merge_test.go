package autopilot

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestRightLaneReturnWaitsForEntireAdjacentBus(t *testing.T) {
	driver := NewAutopilotDriver()
	driver.drivePath = []sim.Vec2{{X: -300, Y: 182}, {X: 300, Y: 182}}
	driver.cumLen = []float64{0, 600}
	player, neighbor := sim.NewBus(0, 130, 0), sim.NewBus(75, 182, 0)
	player.SetModel(sim.BusArticulated)
	neighbor.SetModel(sim.BusArticulated)
	frame := autopilotFrame{bus: player, laneWidth: 52, s: 300, obstacles: []VehicleObstacle{ObserveBus(neighbor)}}
	if got := driver.holdOccupiedRightLane(frame, .6); got > 0 {
		t.Fatalf("steered into adjacent bus's trailer: %g", got)
	}
	neighbor.Pos.X = 250
	neighbor.ResetArticulation()
	frame.obstacles[0] = ObserveBus(neighbor)
	if got := driver.holdOccupiedRightLane(frame, .6); got != .6 {
		t.Fatalf("held after the entire lane gap cleared: %g", got)
	}
}

func TestParallelBusRetreatChecksRearTraffic(t *testing.T) {
	const laneWidth = 52
	world := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: -800}, P2: sim.Vec2{X: 800}, Lanes: 8}}}
	driver := NewAutopilotDriver()
	driver.FleetBus = true
	driver.drivePath = []sim.Vec2{{X: -400, Y: 182}, {X: 400, Y: 182}}
	driver.cumLen = []float64{0, 800}
	ai, angled := sim.NewBus(0, 182, 0), sim.NewBus(36.886, 141.012, .2748)
	ai.SetModel(sim.BusArticulated)
	angled.SetModel(sim.BusArticulated)
	angled.TrailerHeading = 0
	angled.TrailerInitialized = true
	frame := autopilotFrame{bus: ai, world: world, laneWidth: laneWidth, s: 400, obstacles: []VehicleObstacle{ObserveBus(angled)}}
	if distance := driver.parallelMergeRetreatDistance(frame); distance < 20 || !driver.laneRetreatReverseClear(frame, distance, 0) {
		t.Fatalf("expected a safe retreat in the open lane, requested %gpx", distance)
	}
	rear := sim.NewBus(-230, 182, 0)
	frame.obstacles = append(frame.obstacles, ObserveBus(rear))
	if driver.tryHeadonDeadlockRecovery(frame, 2) || driver.HeadonRecoveryActive() {
		t.Fatal("AI accepted a reverse maneuver into a rear vehicle")
	}
}
