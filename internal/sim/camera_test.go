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

func TestWideDrivingViewDoublesWorldCoverage(t *testing.T) {
	for _, rotation := range []float64{0, math.Pi / 2, math.Pi} {
		cam := NewCamera(1280, 720)
		cam.Pos = Vec2{X: 350, Y: -200}
		cam.TargetPos = cam.Pos
		cam.Rotation = rotation
		before := cam.ScreenToWorld(Vec2{}).Sub(cam.Pos)
		if cam.ToggleDrivingView() != 2 {
			t.Fatal("wide view did not enable")
		}
		after := cam.ScreenToWorld(Vec2{}).Sub(cam.Pos)
		if after.Distance(before.Mul(2)) > .001 || cam.Zoom != .5 || cam.TargetZoom != .5 {
			t.Fatal("world coverage did not double")
		}
		world := cam.ScreenToWorld(Vec2{X: 100, Y: 200})
		if cam.WorldToScreen(world).Distance(Vec2{X: 100, Y: 200}) > .001 {
			t.Fatal("wide view coordinate transform mismatch")
		}
		cam.Zoom, cam.TargetZoom = .08, .08 // Planning zoom must not overwrite the driving choice.
		cam.RestoreDrivingZoom()
		if cam.Zoom != .5 {
			t.Fatal("planning discarded wide view")
		}
		if cam.ToggleDrivingView() != 4 || cam.Zoom != .25 || cam.TargetZoom != .25 {
			t.Fatal("second toggle did not select 4× view")
		}
		if cam.ScreenToWorld(Vec2{}).Sub(cam.Pos).Distance(before.Mul(4)) > .001 {
			t.Fatal("4× world coverage mismatch")
		}
		world = cam.ScreenToWorld(Vec2{X: 100, Y: 200})
		if cam.WorldToScreen(world).Distance(Vec2{X: 100, Y: 200}) > .001 {
			t.Fatal("4× view coordinate transform mismatch")
		}
		if cam.ToggleDrivingView() != 1 || cam.Zoom != 1 || cam.TargetZoom != 1 {
			t.Fatal("third toggle did not restore default")
		}
		if cam.Pos != cam.TargetPos || cam.Rotation != rotation || cam.ScreenWidth != 1280 || cam.ScreenHeight != 720 {
			t.Fatal("toggle changed framing or window size")
		}
	}
}
