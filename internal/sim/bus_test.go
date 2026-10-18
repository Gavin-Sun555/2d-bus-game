package sim

import (
	"testing"
)

func TestBusManualTransmissionAndGears(t *testing.T) {
	// Gear strings and names
	gears := []Gear{GearReverse, GearNeutral, GearDrive, Gear1, Gear2, Gear3, Gear4, Gear5}
	expectedStrings := []string{"R", "N", "D", "1", "2", "3", "4", "5"}

	for i, g := range gears {
		if g.String() != expectedStrings[i] {
			t.Errorf("Gear %d String() = %s, expected %s", i, g.String(), expectedStrings[i])
		}
		if g.Name() == "" {
			t.Errorf("Gear %d Name() should not be empty", i)
		}
	}

	// Default unknown gear
	var unknownGear Gear = 99
	if unknownGear.String() != "N" {
		t.Errorf("Unknown gear should default to N string")
	}

	// TransmissionAuto Shifting
	bus := NewBus(0, 0, 0)
	bus.TransmissionMode = TransmissionAuto
	bus.Gear = GearNeutral

	if !bus.ShiftUp() || bus.Gear != GearDrive {
		t.Errorf("Auto ShiftUp from N should shift to D, got %s", bus.Gear)
	}
	if bus.ShiftUp() {
		t.Errorf("Auto ShiftUp from D should return false")
	}
	if !bus.ShiftDown() || bus.Gear != GearNeutral {
		t.Errorf("Auto ShiftDown from D should shift to N, got %s", bus.Gear)
	}
	if !bus.ShiftDown() || bus.Gear != GearReverse {
		t.Errorf("Auto ShiftDown from N should shift to R, got %s", bus.Gear)
	}
	if bus.ShiftDown() {
		t.Errorf("Auto ShiftDown from R should return false")
	}

	// TransmissionManual Shifting progression: R -> N -> 1 -> 2 -> 3 -> 4 -> 5
	bus.TransmissionMode = TransmissionManual
	bus.Gear = GearReverse
	expectedManualUp := []Gear{GearNeutral, Gear1, Gear2, Gear3, Gear4, Gear5}
	for _, expected := range expectedManualUp {
		if !bus.ShiftUp() || bus.Gear != expected {
			t.Errorf("Manual ShiftUp expected %s, got %s", expected, bus.Gear)
		}
	}
	if bus.ShiftUp() {
		t.Errorf("Manual ShiftUp at 5th gear should return false")
	}

	// Manual ShiftDown: 5 -> 4 -> 3 -> 2 -> 1 -> N -> R
	expectedManualDown := []Gear{Gear4, Gear3, Gear2, Gear1, GearNeutral, GearReverse}
	for _, expected := range expectedManualDown {
		if !bus.ShiftDown() || bus.Gear != expected {
			t.Errorf("Manual ShiftDown expected %s, got %s", expected, bus.Gear)
		}
	}
	if bus.ShiftDown() {
		t.Errorf("Manual ShiftDown at R should return false")
	}

	// Gear limits
	for _, g := range gears {
		bus.Gear = g
		maxSpd, accel := bus.GetGearLimits()
		if g == GearNeutral {
			if maxSpd != 0 || accel != 0 {
				t.Errorf("Gear N limits should be 0: maxSpd=%.1f, accel=%.1f", maxSpd, accel)
			}
		} else {
			if maxSpd <= 0 || accel <= 0 {
				t.Errorf("Gear %s limits invalid: maxSpd=%.1f, accel=%.1f", g, maxSpd, accel)
			}
		}
	}
}

func TestBusKinematics(t *testing.T) {
	bus := NewBus(0, 0, 0)

	// Test acceleration forward (throttle=1.0, brake=0.0)
	bus.Update(0.5, 1.0, 0.0, 0.0, false, false)
	if bus.Speed <= 0 {
		t.Fatalf("Expected bus speed to increase with throttle, got %f", bus.Speed)
	}

	// Test steering turns heading
	bus.Update(0.5, 1.0, 0.0, 1.0, false, false)
	if bus.Heading == 0 {
		t.Fatalf("Expected bus heading to change when steering while moving, got %f", bus.Heading)
	}

	// Test door safety lock (cannot move while doors are open)
	bus.Speed = 0
	bus.Update(0.1, 0, 0, 0, false, true) // toggle door open
	if !bus.DoorsOpen {
		t.Fatalf("Expected doors to open when stopped")
	}

	// Attempt throttle while doors open
	bus.Update(0.5, 1.0, 0, 0, false, false)
	if bus.Speed != 0 {
		t.Fatalf("Bus should not move while doors are open, got speed %f", bus.Speed)
	}
}

func TestBusShoulderDecelerationAndCrash(t *testing.T) {
	busOnRoad := NewBus(0, 0, 0)
	busOnRoad.Speed = 200
	busOnRoad.OnShoulder = false
	busOnRoad.Update(0.1, 0, 0, 0, false, false) // 0.1s natural friction

	busOnShoulder := NewBus(0, 0, 0)
	busOnShoulder.Speed = 200
	busOnShoulder.OnShoulder = true
	busOnShoulder.Update(0.1, 0, 0, 0, false, false) // 0.1s shoulder drag

	if busOnShoulder.Speed >= busOnRoad.Speed {
		t.Fatalf("Expected bus on shoulder to decelerate faster than on road: shoulder=%f, road=%f",
			busOnShoulder.Speed, busOnRoad.Speed)
	}

	// Test Crash / Durability
	if busOnShoulder.Durability != 100.0 {
		t.Fatalf("Expected fresh bus durability to be 100")
	}
	busOnShoulder.Durability -= 20.0
	if busOnShoulder.Durability != 80.0 {
		t.Fatalf("Expected durability 80, got %f", busOnShoulder.Durability)
	}
}

func TestResolveBusCarSeparation_PushesApart(t *testing.T) {
	bus := NewBus(200.0, 100.0, 0.0)
	bus.Speed = 0.0

	// Car forced inside the rear half of the bus
	car := &TrafficCar{
		Pos:     Vec2{X: 180.0, Y: 100.0},
		Heading: 0.0,
		Speed:   50.0,
		Length:  48.0,
		Width:   22.0,
	}

	busOBB := OBB{Center: bus.Pos, HalfLen: bus.Length / 2.0, HalfWid: bus.Width / 2.0, Heading: bus.Heading}
	carOBB := OBB{Center: car.Pos, HalfLen: car.Length / 2.0, HalfWid: car.Width / 2.0, Heading: car.Heading}
	collidesBefore, _, depthBefore := CheckOBBCollision(busOBB, carOBB)
	if !collidesBefore || depthBefore <= 0 {
		t.Fatalf("Test setup error: expected initial collision")
	}

	separated := ResolveBusCarSeparation(bus, car)
	if !separated {
		t.Fatalf("Expected ResolveBusCarSeparation to return true")
	}

	// Check post-separation collision state
	busOBB.Center = bus.Pos
	carOBB.Center = car.Pos
	collidesAfter, _, depthAfter := CheckOBBCollision(busOBB, carOBB)
	if collidesAfter && depthAfter > 0.001 {
		t.Fatalf("Expected zero overlap after separation, but collides=%v with depth=%.2f", collidesAfter, depthAfter)
	}
	if car.Speed != 0 {
		t.Fatalf("Expected car speed to be killed to 0 upon rear impact, got: %.2f", car.Speed)
	}
}

func TestResolveBusBusSeparation_PushesApart(t *testing.T) {
	bus1 := NewBus(200.0, 100.0, 0.0)
	bus1.Speed = 0.0
	bus2 := NewBus(230.0, 100.0, 0.0)
	bus2.Speed = 30.0

	obb1 := OBB{Center: bus1.Pos, HalfLen: bus1.Length / 2.0, HalfWid: bus1.Width / 2.0, Heading: bus1.Heading}
	obb2 := OBB{Center: bus2.Pos, HalfLen: bus2.Length / 2.0, HalfWid: bus2.Width / 2.0, Heading: bus2.Heading}
	collidesBefore, _, _ := CheckOBBCollision(obb1, obb2)
	if !collidesBefore {
		t.Fatalf("Test setup error: expected initial collision")
	}

	separated := ResolveBusBusSeparation(bus1, bus2)
	if !separated {
		t.Fatalf("Expected ResolveBusBusSeparation to return true")
	}

	obb1.Center = bus1.Pos
	obb2.Center = bus2.Pos
	collidesAfter, _, depthAfter := CheckOBBCollision(obb1, obb2)
	if collidesAfter && depthAfter > 0.001 {
		t.Fatalf("Expected zero overlap after bus-bus separation, got depth=%.2f", depthAfter)
	}
}

func TestResolveCarCarSeparation_PushesApart(t *testing.T) {
	car1 := &TrafficCar{Pos: Vec2{X: 100.0, Y: 100.0}, Heading: 0.0, Speed: 0.0, Length: 48.0, Width: 22.0}
	car2 := &TrafficCar{Pos: Vec2{X: 120.0, Y: 100.0}, Heading: 0.0, Speed: 40.0, Length: 48.0, Width: 22.0}

	obb1 := OBB{Center: car1.Pos, HalfLen: car1.Length / 2.0, HalfWid: car1.Width / 2.0, Heading: car1.Heading}
	obb2 := OBB{Center: car2.Pos, HalfLen: car2.Length / 2.0, HalfWid: car2.Width / 2.0, Heading: car2.Heading}
	collidesBefore, _, _ := CheckOBBCollision(obb1, obb2)
	if !collidesBefore {
		t.Fatalf("Test setup error: expected initial collision")
	}

	separated := ResolveCarCarSeparation(car1, car2)
	if !separated {
		t.Fatalf("Expected ResolveCarCarSeparation to return true")
	}

	obb1.Center = car1.Pos
	obb2.Center = car2.Pos
	collidesAfter, _, depthAfter := CheckOBBCollision(obb1, obb2)
	if collidesAfter && depthAfter > 0.001 {
		t.Fatalf("Expected zero overlap after car-car separation, got depth=%.2f", depthAfter)
	}
}
