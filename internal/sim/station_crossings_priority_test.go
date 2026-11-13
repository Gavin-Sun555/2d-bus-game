package sim

import (
	"testing"
)

func TestStationCrossingSignalPriorityOnBusLaneRoad(t *testing.T) {
	const lw = 52.0
	w := &World{LargeGrid: true}
	w.BuildMap(MapThemeMetropolis, lw)

	// Find a StationSignalZebra crossing on a road with bus lanes
	var target *StationCrossing
	for _, c := range w.StationCrossings() {
		if c.Kind == StationSignalZebra && c.BusLane {
			cp := c
			target = &cp
			break
		}
	}
	if target == nil {
		t.Fatal("no signal-controlled crossing on bus lane road in large grid Metropolis")
	}

	lights := NewTrafficLightSystem()
	forward := UnitFromAngle(target.Heading)
	right := Vec2{X: -forward.Y, Y: forward.X}

	// 1. Test Green Extension:
	// Advance until normal green is about to expire (Timer near 23.5s)
	// Create an approaching bus on the road corridor
	busPos := target.Center.Sub(forward.Mul(350)).Add(right.Mul(lw / 2))
	bus := &Bus{
		Pos:     busPos,
		Heading: target.Heading,
		Speed:   200,
		Length:  46,
		Width:   22,
	}
	obs := []BusSignalObservation{{BusID: 1, Bus: bus}}

	// Observe priority enabled with the world's crossings
	lights.ObserveBusPriority(true, obs, lw, w.StationCrossings())

	// Initialize crossing state to 23s into Green
	st := lights.StationCrossingStates[target.ID]
	if st == nil {
		t.Fatalf("expected crossing state for ID %d", target.ID)
	}
	st.Phase = StationSignalGreen
	st.Timer = 23.0
	st.Extension = 0

	// Advance 3 seconds: normal timer would be at 26s (Yellow), but priority bus holds Green!
	for i := 0; i < 30; i++ {
		lights.ObserveBusPriority(true, obs, lw, w.StationCrossings())
		lights.Update(0.1)
	}
	if lights.StationSignalPhase(*target) != StationSignalGreen {
		t.Fatalf("expected Green Extension, but got %v", lights.StationSignalPhase(*target))
	}
	if st.Extension <= 0 {
		t.Fatalf("expected Extension > 0, got %v", st.Extension)
	}

	// Move bus past the crossing so rear bumper clears:
	bus.Pos = target.Center.Add(forward.Mul(100))
	lights.ObserveBusPriority(true, obs, lw, w.StationCrossings())
	if st.Priority {
		t.Fatal("expected priority to release after bus cleared the crossing")
	}

	// Advance 1s, it should transition to Yellow because timer >= 24s and priority released
	lights.Update(0.1)
	if lights.StationSignalPhase(*target) != StationSignalYellow {
		t.Fatalf("expected transition to Yellow after bus cleared, got %v", lights.StationSignalPhase(*target))
	}

	// 2. Test Red Truncation / Priority Recall:
	// Set crossing to Red phase at Timer = 1.0s
	st.Phase = StationSignalRed
	st.Timer = 1.0
	st.Extension = 0
	st.Priority = false

	// Bus approaches from 400px before the crossing
	bus.Pos = target.Center.Sub(forward.Mul(400))
	lights.ObserveBusPriority(true, obs, lw, w.StationCrossings())

	// Step forward by 2.2s (Timer reaches 3.2s > min pedestrian time 3.0s)
	for i := 0; i < 22; i++ {
		lights.ObserveBusPriority(true, obs, lw, w.StationCrossings())
		lights.Update(0.1)
	}
	// Red light must be truncated and switched back to Green for approaching bus!
	if lights.StationSignalPhase(*target) != StationSignalGreen {
		t.Fatalf("expected Red truncation to Green for approaching bus, got %v", lights.StationSignalPhase(*target))
	}

	// 3. Doors Open Exclusion:
	// If bus is stopped with doors open, priority must NOT engage
	bus.DoorsOpen = true
	st.Phase = StationSignalRed
	st.Timer = 1.0
	for i := 0; i < 30; i++ {
		lights.ObserveBusPriority(true, obs, lw, w.StationCrossings())
		lights.Update(0.1)
	}
	if st.Priority {
		t.Fatal("expected no priority while bus doors are open")
	}
	// Light should remain Red (did not truncate at 3s)
	if lights.StationSignalPhase(*target) != StationSignalRed {
		t.Fatalf("expected Red light to remain while doors open, got %v", lights.StationSignalPhase(*target))
	}
}

func TestStationCrossingNoPriorityWithoutBusLane(t *testing.T) {
	const lw = 52.0
	target := StationCrossing{
		ID:          99,
		Center:      Vec2{X: 100, Y: 100},
		Heading:     0,
		Lanes:       4,
		Kind:        StationSignalZebra,
		BusLane:     false, // Ordinary road without bus lane!
		phaseOffset: 0,
	}

	lights := NewTrafficLightSystem()
	bus := &Bus{
		Pos:     Vec2{X: 0, Y: 100},
		Heading: 0,
		Speed:   150,
		Length:  46,
		Width:   22,
	}
	obs := []BusSignalObservation{{BusID: 1, Bus: bus}}

	// Observe priority
	lights.ObserveBusPriority(true, obs, lw, []StationCrossing{target})

	// Crossing without bus lane must NOT have priority state created
	if lights.StationCrossingStates[target.ID] != nil {
		t.Fatal("crossing on ordinary road without bus lane should not receive priority state")
	}
}
