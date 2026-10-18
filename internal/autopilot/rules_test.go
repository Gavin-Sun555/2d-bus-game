package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Lane changes create intentional lateral error. Keep recovery separate from
// ordinary following/braking so those errors do not freeze a passing bus.
func TestAutopilotRecoveryRespectsManeuversAndExistingSpeedCaps(t *testing.T) {
	tests := []struct {
		name                          string
		berserk, sharp                bool
		overtake                      int
		alpha, lateral, initial, want float64
	}{
		{name: "standard mild heading error", alpha: 0.3, initial: 150, want: 48},
		{name: "standard severe lateral error", lateral: 41, initial: 150, want: 38},
		{name: "berserk straight recovery", berserk: true, alpha: 0.8, initial: 445, want: 100},
		{name: "berserk corner recovery", berserk: true, sharp: true, alpha: 0.8, initial: 445, want: 75},
		{name: "intentional passing offset", overtake: OvertakePassing, lateral: 80, initial: 150, want: 150},
		{name: "lane change spinout", overtake: OvertakeChangingLane, alpha: 1.2, initial: 150, want: 70},
		{name: "preserve prior stop", alpha: 0.8, initial: 0, want: 0},
		{name: "preserve lower corner cap", berserk: true, alpha: 0.8, initial: 45, want: 45},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			driver := NewAutopilotDriver()
			driver.BerserkMode, driver.OvertakeState = tt.berserk, tt.overtake
			driver.drivePath = []sim.Vec2{{}, {X: 1000}}
			driver.cumLen = []float64{0, 1000}
			frame := autopilotFrame{bus: sim.NewBus(0, 0, 0), alpha: tt.alpha, lateralErr: tt.lateral}
			if got := driver.recoverySpeed(frame, tt.initial, tt.sharp); got != tt.want {
				t.Fatalf("recovery speed=%v, want %v", got, tt.want)
			}
		})
	}
}

// A waypoint before the intersection mouth must not select a turn. Also retain
// the distinction between no exit and an exit whose direction is unclassified:
// the lane planner uses it to decide whether fallback turn detection is needed.
func TestAutopilotIntersectionExitClassification(t *testing.T) {
	tests := []struct {
		name  string
		path  []sim.Vec2
		turn  routeTurn
		found bool
	}{
		{"approach jog is not an exit", []sim.Vec2{{X: -200, Y: -100}, {X: 50}}, routeTurnStraight, true},
		{"left exit", []sim.Vec2{{X: 20, Y: -100}}, routeTurnLeft, true},
		{"right exit", []sim.Vec2{{X: 20, Y: 100}}, routeTurnRight, true},
		{"no exit", []sim.Vec2{{X: -200}, {X: 10}}, routeTurnStraight, false},
		{"unclassified exit", []sim.Vec2{{X: -40, Y: 6}}, routeTurnStraight, true},
		{"later classified exit", []sim.Vec2{{X: -40, Y: 6}, {X: 20, Y: -100}}, routeTurnLeft, true},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			driver := NewAutopilotDriver()
			driver.drivePath = tt.path
			turn, found := driver.intersectionExitTurn(sim.Vec2{}, 1, 0, 50, 5)
			if turn != tt.turn || found != tt.found {
				t.Fatalf("turn=%v found=%v, want %v %v", turn, found, tt.turn, tt.found)
			}
		})
	}
}

func TestAutopilotDepartureStateBoundaries(t *testing.T) {
	driver := NewAutopilotDriver()
	driver.State = AutopilotStateDeparting
	previous := &sim.BusStop{Pos: sim.Vec2{}}
	bus := sim.NewBus(120, 0, 0)
	driver.updateDrivingState(bus, previous, 1000, 50)
	if driver.State != AutopilotStateDeparting {
		t.Fatal("must hold departure at the 120px boundary")
	}
	bus.Pos.X = math.Nextafter(120, math.Inf(1))
	driver.updateDrivingState(bus, previous, 1000, 50)
	if driver.State != AutopilotStateCruising {
		t.Fatal("must cruise after clearing the departure boundary")
	}
	driver.State = AutopilotStateDeparting
	driver.updateDrivingState(bus, previous, apApproachDist-1, 50)
	if driver.State != AutopilotStateApproaching {
		t.Fatal("approaching the next stop takes priority over departure")
	}
	driver.CanSkipTargetStop = true
	driver.updateDrivingState(bus, previous, 0, 50)
	if driver.State != AutopilotStateCruising {
		t.Fatal("a skipped stop must not trigger an approach")
	}
}
