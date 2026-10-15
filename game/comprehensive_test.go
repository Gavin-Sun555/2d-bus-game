package game

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

// -----------------------------------------------------------------------------
// 1. Traffic Light System: Signal Info, Decision & Phases
// -----------------------------------------------------------------------------

func TestTrafficLightApproachingSignalInfoAndStopDecision(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)
	if tls == nil || len(tls.Intersections) == 0 {
		t.Fatalf("Metropolis map must have traffic intersections")
	}

	inter := tls.Intersections[0]
	// Position a vehicle approaching from the west heading East (heading = 0)
	stopLineDist := 52.0*1.6 + 18.0
	// Outer through lane is at Y = inter.Center.Y + 52.0 * 1.5
	throughApproachPos := Vec2{X: inter.Center.X - stopLineDist - 60.0, Y: inter.Center.Y + 52.0*1.5}
	// Inner left-turn lane is at Y = inter.Center.Y + 52.0 * 0.5
	leftApproachPos := Vec2{X: inter.Center.X - stopLineDist - 60.0, Y: inter.Center.Y + 52.0*0.5}

	// 1. Signal Info Query
	hasInter, thruSig, leftSig, isMajor, remSec, distToLine := tls.GetApproachingSignalInfo(throughApproachPos, 0, 52.0)
	if !hasInter {
		t.Fatalf("Expected intersection detection for approachPos %v heading East towards %v", throughApproachPos, inter.Center)
	}
	if distToLine <= 0 || distToLine > 100.0 {
		t.Errorf("Unexpected distToLine: %.2f", distToLine)
	}
	if remSec < 0 {
		t.Errorf("Remaining seconds should be non-negative, got %.2f", remSec)
	}
	_ = thruSig
	_ = leftSig
	_ = isMajor

	// Distant position should not detect approaching intersection
	farPos := Vec2{X: inter.Center.X - 1000.0, Y: inter.Center.Y}
	hasFar, _, _, _, _, _ := tls.GetApproachingSignalInfo(farPos, 0, 52.0)
	if hasFar {
		t.Errorf("Distant position %v should not detect approaching intersection", farPos)
	}

	// 2. Stop Decision under Green vs Red phases
	// In PhaseEWGreen: Through traffic gets Green, Left-turn gets Red
	inter.Phase = PhaseEWGreen
	inter.Timer = 1.0
	inter.PlayerInside = false
	inter.FinedThisPassage = false

	stopGreen, _ := tls.CheckStopForVehicle(throughApproachPos, 0, 120.0, 52.0)
	if stopGreen {
		t.Errorf("Vehicle should not stop under green phase")
	}

	// Dedicated left-turn lane under PhaseEWGreen gets Red signal
	stopLeftDuringThruGreen, _ := tls.CheckStopForVehicle(leftApproachPos, 0, 120.0, 52.0)
	if !stopLeftDuringThruGreen {
		t.Errorf("Vehicle in dedicated left lane should stop under red left arrow")
	}

	// Dedicated left-turn lane under PhaseEWLeftGreen gets Green signal
	inter.Phase = PhaseEWLeftGreen
	inter.Timer = 1.0
	stopLeftGreen, _ := tls.CheckStopForVehicle(leftApproachPos, 0, 120.0, 52.0)
	if stopLeftGreen {
		t.Errorf("Vehicle in dedicated left lane should not stop under left green arrow")
	}

	// Set Phase to North-South Green (East-West is Red)
	inter.Phase = PhaseNSGreen
	inter.Timer = 1.0
	stopRed, distRed := tls.CheckStopForVehicle(throughApproachPos, 0, 120.0, 52.0)
	if !stopRed {
		t.Errorf("Vehicle should stop under red phase")
	}
	if distRed <= 0 || distRed > 120.0 {
		t.Errorf("Expected positive distance to stop line under red, got %.2f", distRed)
	}

	// Wrapper method ShouldVehicleStop
	if !tls.ShouldVehicleStop(throughApproachPos, 0, 120.0) {
		t.Errorf("ShouldVehicleStop should return true under red phase")
	}
}

// -----------------------------------------------------------------------------
// 2. Road Surveillance Camera & Wrong-Way Violation Detection
// -----------------------------------------------------------------------------

func TestRoadCameraViolationsComprehensive(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	tls := NewTrafficLightSystem()
	tls.BuildForWorld(w)

	// Ensure there is at least one road camera on Metropolis
	if len(tls.RoadCameras) == 0 {
		t.Fatalf("Metropolis map should have road cameras")
	}

	cam := tls.RoadCameras[0]
	cam.Cooldown = 0

	bus := NewBus(cam.Pos.X, cam.Pos.Y, 0)
	bus.Speed = 40.0 // Driving speed

	if cam.IsEW {
		// South side of centerline (Y = cam.RoadCoord + 30): legal flow is East (heading = 0)
		bus.Pos = Vec2{X: cam.Pos.X, Y: cam.RoadCoord + 30.0}
		bus.Heading = 0 // Eastbound (legal)
		violated, _ := tls.CheckRoadCameraViolation(bus)
		if violated {
			t.Errorf("Lawful driving on right-hand side should not trigger road camera violation")
		}

		// Driving Westbound on the South side is wrong-way (逆行)!
		bus.Heading = math.Pi // Westbound (wrong-way on south side!)
		violated, notice := tls.CheckRoadCameraViolation(bus)
		if !violated {
			t.Errorf("Wrong-way driving past camera should trigger violation")
		}
		if notice == "" {
			t.Errorf("Expected violation notice message")
		}
		if cam.Cooldown <= 0 {
			t.Errorf("Camera cooldown should be set after violation")
		}

		// Subsequent check during cooldown should not fine again
		violatedCooldown, _ := tls.CheckRoadCameraViolation(bus)
		if violatedCooldown {
			t.Errorf("Camera on cooldown should not issue duplicate violation")
		}
	}

	// Bus stopped should not be fined as wrong-way driving
	cam.Cooldown = 0
	bus.Speed = 0
	bus.Heading = math.Pi
	violatedStopped, _ := tls.CheckRoadCameraViolation(bus)
	if violatedStopped {
		t.Errorf("Stopped vehicle should not be fined by road camera")
	}
}

func TestIsBusFacingWrongWayOnRoadways(t *testing.T) {
	// Bus in intersection is exempt
	interBus := NewBus(0, 0, math.Pi)
	if IsBusFacingWrongWay(interBus) {
		t.Errorf("Bus inside intersection should not be considered wrong-way")
	}

	// Bus on East-West road Y = 0 (away from intersection):
	// South side (Y = 50): legal flow East (heading 0), wrong-way West (heading Pi)
	legalBus := NewBus(600, 50, 0)
	legalBus.Speed = 30
	if IsBusFacingWrongWay(legalBus) {
		t.Errorf("Eastbound bus on south side should not be wrong-way")
	}

	wrongBus := NewBus(600, 50, math.Pi)
	wrongBus.Speed = 30
	if !IsBusFacingWrongWay(wrongBus) {
		t.Errorf("Westbound bus on south side must be detected as wrong-way")
	}

	// Reversing bus: heading is Pi but moving backwards (Speed < -1): effective direction is East!
	reversingBus := NewBus(600, 50, math.Pi)
	reversingBus.Speed = -20
	if IsBusFacingWrongWay(reversingBus) {
		t.Errorf("Bus reversing eastward on south side should not be wrong-way")
	}

	// Nil bus safety
	if IsBusFacingWrongWay(nil) {
		t.Errorf("Nil bus should return false")
	}
}

// -----------------------------------------------------------------------------
// 3. Transmission, Shifting & Gearbox Kinematics
// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------
// 4. Camera Viewport Culling & Visibility Math
// -----------------------------------------------------------------------------

func TestCameraViewportCulling(t *testing.T) {
	cam := NewCamera(1280, 720)
	cam.Pos = Vec2{X: 0, Y: 0}
	cam.Zoom = 1.0
	cam.Rotation = 0

	// 1. Box in camera view center
	if !cam.IsBoxVisible(-100, -100, 100, 100) {
		t.Errorf("Box at center should be visible")
	}

	// 2. Box far outside viewport
	if cam.IsBoxVisible(3000, 3000, 3100, 3100) {
		t.Errorf("Distant box should not be visible")
	}

	// 3. Point visibility
	if !cam.IsPointVisible(Vec2{X: 100, Y: 100}, 25.0) {
		t.Errorf("Point inside viewport should be visible")
	}
	if cam.IsPointVisible(Vec2{X: 4000, Y: 4000}, 25.0) {
		t.Errorf("Distant point should not be visible")
	}

	// 4. Rotated camera (45 degrees)
	cam.Rotation = math.Pi / 4
	if !cam.IsBoxVisible(-50, -50, 50, 50) {
		t.Errorf("Center box under rotation should be visible")
	}
	if cam.IsBoxVisible(5000, 5000, 5100, 5100) {
		t.Errorf("Distant box under rotation should not be visible")
	}

	// 5. ScreenToWorld and WorldToScreen round-trip consistency
	cam.Rotation = 0
	cam.Zoom = 1.25
	cam.Pos = Vec2{X: 300, Y: -200}
	worldPt := Vec2{X: 450, Y: -120}
	screenPt := cam.WorldToScreen(worldPt)
	recoveredWorld := cam.ScreenToWorld(screenPt)
	if recoveredWorld.Distance(worldPt) > 0.01 {
		t.Errorf("Coordinate round-trip mismatch: got %v, expected %v", recoveredWorld, worldPt)
	}
}

// -----------------------------------------------------------------------------
// 5. Bus Stop Passenger Generation & Containment
// -----------------------------------------------------------------------------

func TestBusStopPassengerArrivalsAndContainment(t *testing.T) {
	stop := NewBusStop(1, "TestStop", Vec2{X: 200, Y: 100}, 0, 4, RGBA(0, 0, 0, 0), 52.0, 5)
	stop.SpawnInterval = 0.5 // Fast arrival cycle
	stop.SpawnTimer = 0.0
	stop.WaitingPassengers = 3
	stop.MaxWaiting = 15

	// Advance less than spawn interval
	stop.Update(0.2)
	if stop.WaitingPassengers != 3 {
		t.Errorf("Passengers should not spawn before interval elapses")
	}

	// Cross spawn interval
	stop.Update(0.4)
	if stop.WaitingPassengers <= 3 {
		t.Errorf("Passengers should have arrived after spawn interval elapsed, got %d", stop.WaitingPassengers)
	}
	if stop.SpawnTimer != 0 {
		t.Errorf("Spawn timer should reset to 0 after spawning, got %.2f", stop.SpawnTimer)
	}

	// Bus Containment
	bus := NewBus(stop.Pos.X, stop.Pos.Y, stop.Heading)
	if !stop.ContainsBus(bus) {
		t.Errorf("Bus aligned at stop position should be contained inside bay")
	}

	farBus := NewBus(stop.Pos.X+400.0, stop.Pos.Y, stop.Heading)
	if stop.ContainsBus(farBus) {
		t.Errorf("Distant bus should not be contained inside bay")
	}
}

// -----------------------------------------------------------------------------
// 6. HUD Click Interactions & Toast Capacity
// -----------------------------------------------------------------------------

func TestHUDInteractionHandlers(t *testing.T) {
	hud := NewHUD()
	cam := NewCamera(1280, 720)
	bus := NewBus(0, 0, 0)
	bus.Gear = GearNeutral

	// 1. Menu Button Click
	hud.MenuBtnBounds = Rect{X: 10, Y: 10, W: 60, H: 30}
	if !hud.HandleMenuBtnClick(Vec2{X: 25, Y: 25}) {
		t.Errorf("HandleMenuBtnClick should return true when clicking inside bounds")
	}
	if !hud.ReturnToMenuRequested {
		t.Errorf("ReturnToMenuRequested should be true")
	}
	if hud.HandleMenuBtnClick(Vec2{X: 500, Y: 500}) {
		t.Errorf("HandleMenuBtnClick should return false when clicking outside")
	}

	// 2. Camera Toggle Button Click
	hud.CameraBtnBounds = Rect{X: 80, Y: 10, W: 60, H: 30}
	toggled, notice := hud.HandleCameraBtnClick(Vec2{X: 95, Y: 25}, cam)
	if !toggled || notice == "" {
		t.Errorf("HandleCameraBtnClick should toggle camera mode and return notice")
	}

	// 3. Gear Badges Click
	hud.GearDBounds = Rect{X: 200, Y: 600, W: 40, H: 40}
	shifted, gearNotice := hud.HandleGearClick(Vec2{X: 220, Y: 620}, bus)
	if !shifted || bus.Gear != GearDrive || gearNotice == "" {
		t.Errorf("HandleGearClick on D badge should shift bus to Drive, got %s", bus.Gear)
	}

	hud.GearNBounds = Rect{X: 250, Y: 600, W: 40, H: 40}
	shiftedN, _ := hud.HandleGearClick(Vec2{X: 270, Y: 620}, bus)
	if !shiftedN || bus.Gear != GearNeutral {
		t.Errorf("HandleGearClick on N badge should shift bus to Neutral, got %s", bus.Gear)
	}

	// 4. Toast Cap: max 4 toasts
	for i := 0; i < 10; i++ {
		hud.AddToast("Test Toast", true)
	}
	if len(hud.Toasts) > 4 {
		t.Errorf("HUD toast list exceeded maximum display cap of 4, got %d", len(hud.Toasts))
	}
}

// -----------------------------------------------------------------------------
// 7. Settings Menu Toggles & Custom Lane Width
// -----------------------------------------------------------------------------

func TestSettingsInteractionAndToggles(t *testing.T) {
	s := NewSettings()

	// Toggle Open/Close
	if s.IsOpen {
		t.Errorf("Settings should be closed by default")
	}
	s.Toggle()
	if !s.IsOpen {
		t.Errorf("Settings should be open after Toggle")
	}
	s.Toggle()
	if s.IsOpen {
		t.Errorf("Settings should be closed after second Toggle")
	}

	// Lane width is fixed to 52.0 (宽敞市区 Comfortable Wide) as default
	if s.GetLaneWidth() != 52.0 {
		t.Errorf("Expected fixed lane width of 52.0 (宽敞市区), got %.1f", s.GetLaneWidth())
	}
}

// -----------------------------------------------------------------------------
// 8. AI Bus Manager Fleet, Obstacles & Active Counts
// -----------------------------------------------------------------------------

func TestAIBusManagerObstaclesAndFleetCounts(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	abm := NewAIBusManager()

	abm.BuildFleet(w, 3, 52.0)
	if abm.TotalActiveBuses() != 3 {
		t.Errorf("Expected 3 active buses in fleet, got %d", abm.TotalActiveBuses())
	}

	// All obstacles
	allObs := abm.GetObstacles(-1)
	if len(allObs) != 3 {
		t.Errorf("Expected 3 obstacles, got %d", len(allObs))
	}

	// Exclude one bus
	firstID := abm.Buses[0].ID
	filteredObs := abm.GetObstacles(firstID)
	if len(filteredObs) != 2 {
		t.Errorf("Expected 2 obstacles excluding bus #%d, got %d", firstID, len(filteredObs))
	}

	// Set count to 0 (clears fleet)
	abm.SetLineCount(0, w, 52.0)
	if abm.TotalActiveBuses() != 0 {
		t.Errorf("Fleet count should be 0, got %d", abm.TotalActiveBuses())
	}
}

// -----------------------------------------------------------------------------
// 9. Custom Map JSON Serialization & Validation
// -----------------------------------------------------------------------------

func TestCustomMapSerializationRoundtripAndValidation(t *testing.T) {
	data := GetDefaultCustomMapData()
	if err := ValidateCustomMapData(data); err != nil {
		t.Fatalf("Default custom map data validation failed: %v", err)
	}

	// Validation constraints
	badData := *data
	badData.Name = ""
	if err := ValidateCustomMapData(&badData); err == nil {
		t.Errorf("Expected validation error for empty map name")
	}

	badRoads := *data
	badRoads.Roads = []CustomRoadData{{P1: [2]float64{0, 0}, P2: [2]float64{100, 0}, Lanes: 2}}
	if err := ValidateCustomMapData(&badRoads); err == nil {
		t.Errorf("Expected validation error for < 2 roads")
	}

	// colorToHex
	hexOpaque := colorToHex(RGBA(255, 128, 0, 255))
	if hexOpaque != "#FF8000" {
		t.Errorf("Expected #FF8000, got %s", hexOpaque)
	}
	hexAlpha := colorToHex(RGBA(255, 128, 0, 180))
	if hexAlpha != "#FF8000B4" {
		t.Errorf("Expected #FF8000B4, got %s", hexAlpha)
	}

	// Save and Load roundtrip via temporary directory
	tempDir := t.TempDir()
	tempPath := filepath.Join(tempDir, "test_custom_map.json")

	if err := SaveCustomMapToFile(tempPath, data); err != nil {
		t.Fatalf("SaveCustomMapToFile failed: %v", err)
	}

	loaded, err := LoadCustomMapFromFile(tempPath)
	if err != nil {
		t.Fatalf("LoadCustomMapFromFile failed: %v", err)
	}

	if loaded.Name != data.Name {
		t.Errorf("Loaded map name mismatch: got %s, expected %s", loaded.Name, data.Name)
	}
	if len(loaded.Roads) != len(data.Roads) {
		t.Errorf("Loaded roads count mismatch: got %d, expected %d", len(loaded.Roads), len(data.Roads))
	}

	// EnsureDefaultCustomMapFile on non-existing path creates the file
	ensurePath := filepath.Join(tempDir, "ensure_map.json")
	if err := EnsureDefaultCustomMapFile(ensurePath); err != nil {
		t.Fatalf("EnsureDefaultCustomMapFile failed: %v", err)
	}
	if _, err := os.Stat(ensurePath); os.IsNotExist(err) {
		t.Errorf("EnsureDefaultCustomMapFile failed to write file")
	}
}

// -----------------------------------------------------------------------------
// 10. Deterministic Simulation RNG via SeedSimulation
// -----------------------------------------------------------------------------

func TestSeedSimulationDeterminism(t *testing.T) {
	SeedSimulation(42)
	v1 := simRand.Float64()
	v2 := simRand.Intn(100)
	v3 := simRand.Float64()

	// Re-seed with identical value
	SeedSimulation(42)
	r1 := simRand.Float64()
	r2 := simRand.Intn(100)
	r3 := simRand.Float64()

	if v1 != r1 || v2 != r2 || v3 != r3 {
		t.Fatalf("SeedSimulation failed to produce deterministic random sequence: (%f,%d,%f) vs (%f,%d,%f)",
			v1, v2, v3, r1, r2, r3)
	}
}

// -----------------------------------------------------------------------------
// 11. Route Manipulation & Target Stop Resolution
// -----------------------------------------------------------------------------

func TestRouteManipulationAndTargetStop(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)
	route := NewRoute()

	if route.GetCurrentTargetStopID() != -1 {
		t.Errorf("Empty route target stop should be -1")
	}

	route.AddStop(0)
	route.AddStop(1)
	route.AddStop(2)
	// Immediate duplicate stop should be ignored
	route.AddStop(2)
	if len(route.StopIDs) != 3 {
		t.Errorf("Expected 3 stops after duplicate prevention, got %d", len(route.StopIDs))
	}

	if route.GetCurrentTargetStopID() != 0 {
		t.Errorf("Target stop should be 0, got %d", route.GetCurrentTargetStopID())
	}
	route.AdvanceToNextStop()
	if route.GetCurrentTargetStopID() != 1 {
		t.Errorf("Target stop should be 1, got %d", route.GetCurrentTargetStopID())
	}

	// Cached description
	desc := route.GetRouteDescription(w)
	if desc == "" {
		t.Errorf("Route description should not be empty")
	}
	// Calling again should return cached description
	cachedDesc := route.GetRouteDescription(w)
	if cachedDesc != desc {
		t.Errorf("Cached description mismatch")
	}

	route.Clear()
	if len(route.StopIDs) != 0 || route.GetCurrentTargetStopID() != -1 {
		t.Errorf("Route should be clear")
	}
}

// -----------------------------------------------------------------------------
// 12. Autopilot State String & DrivePath Accessor
// -----------------------------------------------------------------------------

func TestAutopilotStateAndDrivePathAccessors(t *testing.T) {
	states := []AutopilotState{
		AutopilotStateCruising,
		AutopilotStateApproaching,
		AutopilotStateDocked,
		AutopilotStateDeparting,
		AutopilotState(99),
	}

	for _, st := range states {
		if st.String() == "" {
			t.Errorf("AutopilotState %d String() should not be empty", st)
		}
	}

	driver := NewAutopilotDriver()
	if driver.DrivePath() != nil {
		t.Errorf("Initial DrivePath should be nil")
	}
}

// -----------------------------------------------------------------------------
// 13. World Entities, Roundabout & Bridge Helpers
// -----------------------------------------------------------------------------

func TestWorldEntitiesAndRoundaboutMethods(t *testing.T) {
	w := NewWorld(52.0, MapThemeMetropolis)

	// Roundabout OuterRadius
	if len(w.Roundabouts) > 0 {
		rb := w.Roundabouts[0]
		if rb.OuterRadius() != rb.Radius {
			t.Errorf("OuterRadius() = %.1f, expected %.1f", rb.OuterRadius(), rb.Radius)
		}
	}

	// Water bodies
	waters := w.WaterBodies()
	if len(waters) == 0 {
		t.Errorf("Metropolis should have water bodies")
	}

	// Bridge styles
	b1 := Bridge{IsArch: true}
	if b1.Style() != BridgeStyleStoneArch {
		t.Errorf("Expected BridgeStyleStoneArch, got %v", b1.Style())
	}
	b2 := Bridge{IsCable: true}
	if b2.Style() != BridgeStyleSuspension {
		t.Errorf("Expected BridgeStyleSuspension, got %v", b2.Style())
	}
	b3 := Bridge{IsOverpass: true}
	if b3.Style() != BridgeStyleOverpass {
		t.Errorf("Expected BridgeStyleOverpass, got %v", b3.Style())
	}
	b4 := Bridge{}
	if b4.Style() != BridgeStyleStandard {
		t.Errorf("Expected BridgeStyleStandard, got %v", b4.Style())
	}

	// MapTheme Name and DisplayName
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeRiverValley,
		MapThemeEmeraldLake,
		MapThemeTwinCities,
		MapThemeCustom,
	}

	for _, th := range themes {
		if th.Name() == "" || th.DisplayName() == "" {
			t.Errorf("Theme %d names should not be empty", th)
		}
	}
}

// -----------------------------------------------------------------------------
// 14. Autopilot Multi-Map Left-Turn & U-Turn Lane Discipline
// -----------------------------------------------------------------------------

func TestAutopilotMultiMapLeftTurnLaneDiscipline(t *testing.T) {
	themes := []MapTheme{
		MapThemeMetropolis,
		MapThemeCoastalBay,
		MapThemeTwinCities,
	}

	for _, th := range themes {
		t.Run(th.DisplayName(), func(t *testing.T) {
			lw := 52.0
			w := NewWorld(lw, th)
			route := GetStandardRouteForMap(th, w)
			if len(route.StopIDs) < 2 {
				t.Fatalf("Route must have >= 2 stops")
			}

			// Validate all consecutive stops along the route maintain proper lane offsets
			for i := 0; i < len(route.StopIDs)-1; i++ {
				s1 := w.Stops[route.StopIDs[i]]
				s2 := w.Stops[route.StopIDs[i+1]]
				path := computeRoadPathBetweenStops(w, s1, s2, lw)
				if len(path) == 0 {
					continue
				}

				// Verify all path waypoints outside roundabouts stay safely away from the center divider
				for _, pt := range path {
					// Roundabouts use circular curves rather than straight-lane offsets
					inRoundabout := false
					for _, rb := range w.Roundabouts {
						if pt.Distance(rb.Center) <= rb.Radius+25.0 {
							inRoundabout = true
							break
						}
					}
					if inRoundabout {
						continue
					}

					// Check against East-West 4-lane avenues
					for _, r := range w.Roads {
						if math.Abs(r.P1.Y-r.P2.Y) < 1.0 && (r.Lanes == 4 || r.Type == RoadAvenue4Lane) {
							if pt.X >= math.Min(r.P1.X, r.P2.X)-10 && pt.X <= math.Max(r.P1.X, r.P2.X)+10 {
								dy := math.Abs(pt.Y - r.P1.Y)
								// Waypoint must be inside valid travel corridors (either Lane 0 ~ 0.5*lw or Lane 1 ~ 1.5*lw)
								// and NOT on top of yellow centerline (|dy| < 8px)
								if dy < 10.0 {
									t.Errorf("[%s] Route waypoint %v is driving on the yellow centerline of %s",
										th.DisplayName(), pt, r.Name)
								}
							}
						}
					}
				}
			}
		})
	}
}
