package ui

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSettingsAndCustomLaneWidth(t *testing.T) {
	settings := NewSettings()
	if !settings.TrafficFinesEnabled {
		t.Fatalf("Expected traffic fines enabled by default")
	}

	// Test lane width is fixed to 52.0 (宽敞市区 Comfortable Wide) as default
	if settings.GetLaneWidth() != 52.0 {
		t.Fatalf("Expected default fixed lane width to be 52.0, got %f", settings.GetLaneWidth())
	}

	// Test World dynamic lane width update
	world := sim.NewWorld(52.0)
	if len(world.Stops) != 12 {
		t.Fatalf("Expected 12 bus stops in expanded world, got %d", len(world.Stops))
	}

	initialStopPos := world.Stops[0].Pos
	world.UpdateLaneWidth(78.0)
	newStopPos := world.Stops[0].Pos

	if initialStopPos == newStopPos {
		t.Fatalf("Expected bus stop bay to adapt dynamically when lane width changes")
	}
}

func TestGearboxTransmissionAndBrakeSeparation(t *testing.T) {
	bus := sim.NewBus(0, 0, 0)

	// 1. Initial State: Default Gear is Drive (D)
	if bus.Gear != sim.GearDrive {
		t.Fatalf("Expected default gear to be GearDrive (D), got %v", bus.Gear)
	}

	// 2. S is strictly Brake: Foot brake decelerates vehicle and holds at 0, NEVER reverses!
	bus.Speed = 150.0
	// Apply heavy brake for 2 seconds (throttle=0, brake=1.0)
	for i := 0; i < 120; i++ {
		bus.Update(0.016, 0.0, 1.0, 0.0, false, false)
	}
	if bus.Speed != 0.0 {
		t.Fatalf("Expected foot brake to bring bus speed down to exactly 0, got %f", bus.Speed)
	}
	if !bus.IsBraking {
		t.Fatalf("Expected IsBraking to be true while holding brake pedal")
	}

	// Continue pressing brake for another 1 second while stopped: MUST stay at 0, NOT reverse!
	for i := 0; i < 60; i++ {
		bus.Update(0.016, 0.0, 1.0, 0.0, false, false)
	}
	if bus.Speed < 0 {
		t.Fatalf("Brake pedal (S) must NEVER cause vehicle to reverse! Speed is %f", bus.Speed)
	}
	if bus.Speed != 0 {
		t.Fatalf("Brake pedal while stopped must keep vehicle at 0, got %f", bus.Speed)
	}

	// 3. In Neutral (N), accelerator throttle produces no driving torque
	bus.ShiftTo(sim.GearNeutral)
	if bus.Gear != sim.GearNeutral {
		t.Fatalf("Expected gear to be GearNeutral (N)")
	}
	for i := 0; i < 60; i++ {
		bus.Update(0.016, 1.0, 0.0, 0.0, false, false) // throttle in N
	}
	if bus.Speed > 0.01 {
		t.Fatalf("In Neutral (N), throttle must NOT accelerate bus, got speed %f", bus.Speed)
	}

	// 4. Reversing requires shifting to Reverse (R):
	// Shift to GearReverse
	if !bus.ShiftTo(sim.GearReverse) {
		t.Fatalf("Expected successful shift to GearReverse from standstill")
	}
	if bus.Gear != sim.GearReverse {
		t.Fatalf("Expected current gear to be GearReverse (R)")
	}

	// Accelerate in Reverse with throttle (W / Up): speed becomes negative
	for i := 0; i < 60; i++ {
		bus.Update(0.016, 1.0, 0.0, 0.0, false, false)
	}
	if bus.Speed >= 0 {
		t.Fatalf("In Reverse (R), throttle must accelerate bus backward (negative speed), got %f", bus.Speed)
	}
	if !bus.IsReversing {
		t.Fatalf("Expected IsReversing to be true while in Reverse gear")
	}

	// While moving in reverse, applying brake (S / Down) decelerates toward 0 and stops at 0!
	for i := 0; i < 120; i++ {
		bus.Update(0.016, 0.0, 1.0, 0.0, false, false)
	}
	if bus.Speed != 0.0 {
		t.Fatalf("Expected foot brake to stop reversing bus squarely at 0, got %f", bus.Speed)
	}

	// 5. Gear Shifting Progression (Auto mode: R <-> N <-> D)
	bus.ShiftTo(sim.GearReverse)
	if !bus.ShiftUp() || bus.Gear != sim.GearNeutral {
		t.Fatalf("Expected ShiftUp from R to go to N, got %v", bus.Gear)
	}
	if !bus.ShiftUp() || bus.Gear != sim.GearDrive {
		t.Fatalf("Expected ShiftUp from N to go to D, got %v", bus.Gear)
	}
	if bus.ShiftUp() {
		t.Fatalf("ShiftUp in Auto at D should return false (already at top auto gear)")
	}

	if !bus.ShiftDown() || bus.Gear != sim.GearNeutral {
		t.Fatalf("Expected ShiftDown from D to go to N, got %v", bus.Gear)
	}
	if !bus.ShiftDown() || bus.Gear != sim.GearReverse {
		t.Fatalf("Expected ShiftDown from N to go to R, got %v", bus.Gear)
	}
	if bus.ShiftDown() {
		t.Fatalf("ShiftDown at R should return false (already in reverse)")
	}

	// 6. High-speed reverse interlock protection:
	bus.ShiftTo(sim.GearDrive)
	bus.Speed = 120.0 // Moving forward fast
	if bus.ShiftTo(sim.GearReverse) {
		t.Fatalf("High-speed forward motion must prevent shifting directly into Reverse (transmission protection)!")
	}
	if bus.Gear != sim.GearDrive {
		t.Fatalf("Gear must remain in Drive when reverse shift is locked out")
	}

	// 7. Manual transmission mode readiness (预留手动挡功能测试):
	bus.Speed = 0
	bus.TransmissionMode = sim.TransmissionManual
	bus.ShiftTo(sim.GearNeutral)

	// Step up through manual gears: N -> 1 -> 2 -> 3 -> 4 -> 5
	expectedManualGears := []sim.Gear{sim.Gear1, sim.Gear2, sim.Gear3, sim.Gear4, sim.Gear5}
	for _, expGear := range expectedManualGears {
		if !bus.ShiftUp() || bus.Gear != expGear {
			t.Fatalf("Expected ShiftUp in manual mode to reach %v, got %v", expGear, bus.Gear)
		}
		maxSpd, accelRatio := bus.GetGearLimits()
		if maxSpd <= 0 || accelRatio <= 0 {
			t.Fatalf("Manual gear %v must have positive maxSpeed and acceleration factor", expGear)
		}
	}

	// 8. HUD interactive gear click test
	hud := NewHUD()
	// Simulate HUD Draw layout setup
	hud.GearRBounds = sim.Rect{X: 134, Y: 600, W: 24, H: 20}
	hud.GearNBounds = sim.Rect{X: 162, Y: 600, W: 24, H: 20}
	hud.GearDBounds = sim.Rect{X: 190, Y: 600, W: 24, H: 20}

	bus.TransmissionMode = sim.TransmissionAuto
	bus.Speed = 0
	hud.HandleGearClick(sim.Vec2{X: 140, Y: 610}, bus)
	if bus.Gear != sim.GearReverse {
		t.Fatalf("Expected clicking R badge to shift bus into Reverse, got %v", bus.Gear)
	}

	hud.HandleGearClick(sim.Vec2{X: 170, Y: 610}, bus)
	if bus.Gear != sim.GearNeutral {
		t.Fatalf("Expected clicking N badge to shift bus into Neutral, got %v", bus.Gear)
	}

	hud.HandleGearClick(sim.Vec2{X: 195, Y: 610}, bus)
	if bus.Gear != sim.GearDrive {
		t.Fatalf("Expected clicking D badge to shift bus into Drive, got %v", bus.Gear)
	}
}

func TestCameraPerspectivesAndRotation(t *testing.T) {
	cam := sim.NewCamera(1280, 720)

	// 1. Initial Mode Verification (North-Up)
	if cam.Mode != sim.CameraModeNorthUp {
		t.Fatalf("Expected default camera mode to be CameraModeNorthUp, got %v", cam.Mode)
	}
	if cam.Mode.String() != "固定正北 (标准)" {
		t.Fatalf("Unexpected North-Up string: %s", cam.Mode.String())
	}

	bus := sim.NewBus(0, 0, 0)
	bus.Heading = 0 // Facing East in world space
	bus.Speed = 50.0

	// In North-Up mode, TargetRotation must be 0
	cam.FollowBus(bus)
	if cam.TargetRotation != 0 {
		t.Fatalf("Expected North-Up TargetRotation to be 0, got %f", cam.TargetRotation)
	}

	// 2. Toggle to Heading-Up Mode
	mode1 := cam.ToggleMode()
	if mode1 != sim.CameraModeHeadingUp || cam.Mode != sim.CameraModeHeadingUp {
		t.Fatalf("Expected camera mode to toggle to CameraModeHeadingUp, got %v", mode1)
	}
	if cam.Mode.String() != "车头朝上 (跟随)" {
		t.Fatalf("Unexpected Heading-Up string: %s", cam.Mode.String())
	}

	// In Heading-Up mode, TargetRotation = Heading + Pi/2
	cam.FollowBus(bus)
	expectedRot := bus.Heading + math.Pi/2
	if math.Abs(cam.TargetRotation-expectedRot) > 1e-6 {
		t.Fatalf("Expected Heading-Up TargetRotation to be %f, got %f", expectedRot, cam.TargetRotation)
	}

	// Forward lookahead verification
	expectedForwardOffset := 120.0 + 0.35*bus.Speed
	distFromBus := cam.TargetPos.Distance(bus.Pos)
	if math.Abs(distFromBus-expectedForwardOffset) > 1e-4 {
		t.Fatalf("Expected forward offset %f, got %f", expectedForwardOffset, distFromBus)
	}

	// Heading-Up Rotation Transform Math: Bus heading on screen facing straight UP
	cam.Pos = bus.Pos
	cam.Rotation = cam.TargetRotation // TargetRotation = Pi/2 for heading 0 (East)

	// Point 100px ahead in the bus's forward direction (East: dx=100, dy=0)
	aheadWorld := bus.Pos.Add(sim.Vec2{X: 100, Y: 0})
	aheadScreen := cam.WorldToScreen(aheadWorld)
	busScreen := cam.WorldToScreen(bus.Pos)

	// In screen coordinates:
	// Since bus heading is East (world +X), with camera rotation Pi/2,
	// the ahead position on screen MUST be directly ABOVE busScreen (smaller Y, same X)!
	if math.Abs(aheadScreen.X-busScreen.X) > 1e-4 {
		t.Fatalf("Ahead point on screen should have same X as bus, got dx=%f", aheadScreen.X-busScreen.X)
	}
	if aheadScreen.Y >= busScreen.Y {
		t.Fatalf("Ahead point on screen should be ABOVE bus (aheadScreen.Y < busScreen.Y), got aheadY=%f, busY=%f", aheadScreen.Y, busScreen.Y)
	}

	// 3. Toggle to South-Up Mode
	mode2 := cam.ToggleMode()
	if mode2 != sim.CameraModeSouthUp || cam.Mode != sim.CameraModeSouthUp {
		t.Fatalf("Expected camera mode to toggle to CameraModeSouthUp, got %v", mode2)
	}
	if cam.Mode.String() != "固定正南 (倒置)" {
		t.Fatalf("Unexpected South-Up string: %s", cam.Mode.String())
	}

	// In South-Up mode, TargetRotation must be Pi
	cam.FollowBus(bus)
	if math.Abs(cam.TargetRotation-math.Pi) > 1e-6 {
		t.Fatalf("Expected South-Up TargetRotation to be Pi, got %f", cam.TargetRotation)
	}

	// South-Up Rotation Transform Math: South (+Y in world) points UP on screen (smaller Y)
	cam.Pos = sim.Vec2{X: 0, Y: 0}
	cam.Rotation = math.Pi
	southWorld := sim.Vec2{X: 0, Y: 100} // Point 100px to the South
	southScreen := cam.WorldToScreen(southWorld)
	centerScreen := cam.WorldToScreen(cam.Pos)

	if southScreen.Y >= centerScreen.Y {
		t.Fatalf("In South-Up mode, South must point UP on screen! got southY=%f, centerY=%f", southScreen.Y, centerScreen.Y)
	}
	if math.Abs(southScreen.X-centerScreen.X) > 1e-4 {
		t.Fatalf("In South-Up mode, purely South point should have same X as center, got dx=%f", southScreen.X-centerScreen.X)
	}

	// 4. Toggle from South-Up back to North-Up Mode
	mode3 := cam.ToggleMode()
	if mode3 != sim.CameraModeNorthUp || cam.Mode != sim.CameraModeNorthUp {
		t.Fatalf("Expected camera mode to cycle back to CameraModeNorthUp, got %v", mode3)
	}

	// 5. Test Invertibility of WorldToScreen and ScreenToWorld with non-zero rotation
	cam.Rotation = 1.234 // Arbitrary non-cardinal angle
	testWorld := sim.Vec2{X: 350.5, Y: -220.8}
	screenPt := cam.WorldToScreen(testWorld)
	recoveredWorld := cam.ScreenToWorld(screenPt)

	if testWorld.Distance(recoveredWorld) > 1e-4 {
		t.Fatalf("ScreenToWorld did not invert WorldToScreen accurately! Original: %v, Recovered: %v", testWorld, recoveredWorld)
	}

	// 6. Test HUD Perspective button click cycles through all 3 modes
	hud := NewHUD()
	hud.CameraBtnBounds = sim.Rect{X: 900, Y: 12, W: 175, H: 32}

	// Currently NorthUp -> click -> HeadingUp
	t1, n1 := hud.HandleCameraBtnClick(sim.Vec2{X: 950, Y: 25}, cam)
	if !t1 || cam.Mode != sim.CameraModeHeadingUp || n1 == "" {
		t.Fatalf("Expected HUD click to switch to Heading-Up, got mode=%v, n=%s", cam.Mode, n1)
	}

	// HeadingUp -> click -> SouthUp
	t2, n2 := hud.HandleCameraBtnClick(sim.Vec2{X: 950, Y: 25}, cam)
	if !t2 || cam.Mode != sim.CameraModeSouthUp || n2 == "" {
		t.Fatalf("Expected HUD click to switch to South-Up, got mode=%v, n=%s", cam.Mode, n2)
	}

	// SouthUp -> click -> NorthUp
	t3, n3 := hud.HandleCameraBtnClick(sim.Vec2{X: 950, Y: 25}, cam)
	if !t3 || cam.Mode != sim.CameraModeNorthUp || n3 == "" {
		t.Fatalf("Expected HUD click to switch back to North-Up, got mode=%v, n=%s", cam.Mode, n3)
	}

	// 7. Test Settings synchronization and 3-way toggle
	settings := NewSettings()
	if settings.CameraViewMode != sim.CameraModeNorthUp {
		t.Fatalf("Expected settings default camera view mode to be CameraModeNorthUp")
	}
	settings.cameraToggleBounds = sim.Rect{X: 320, Y: 190, W: 190, H: 32}
	settings.IsOpen = true

	// Simulate clicks in settings
	settings.CameraViewMode = sim.CameraModeHeadingUp
	cam.SetMode(settings.CameraViewMode)
	if cam.Mode != sim.CameraModeHeadingUp {
		t.Fatalf("Expected camera mode to update to Heading-Up from settings")
	}
	settings.CameraViewMode = sim.CameraModeSouthUp
	cam.SetMode(settings.CameraViewMode)
	if cam.Mode != sim.CameraModeSouthUp {
		t.Fatalf("Expected camera mode to update to South-Up from settings")
	}

	// 8. Test smooth rotation update towards South-Up (pi) and returning to North-Up (0)
	cam.Rotation = 0
	cam.TargetRotation = math.Pi
	cam.Update(0.1)
	if cam.Rotation == 0 {
		t.Fatalf("Camera rotation should smoothly interpolate towards TargetRotation Pi")
	}

	for i := 0; i < 200; i++ {
		cam.Update(0.1)
	}
	if math.Abs(cam.Rotation-math.Pi) > 1e-3 {
		t.Fatalf("Camera rotation should reach Pi in South-Up mode, got %f", cam.Rotation)
	}

	// Switching to NorthUp mode smoothly returns rotation to 0
	cam.SetMode(sim.CameraModeNorthUp)
	cam.FollowBus(bus)
	for i := 0; i < 200; i++ {
		cam.Update(0.1)
	}
	if math.Abs(cam.Rotation) > 1e-3 {
		t.Fatalf("Camera rotation should return to 0 in North-Up mode, got %f", cam.Rotation)
	}
}
