package sim

import (
	"math"
	"testing"
)

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
