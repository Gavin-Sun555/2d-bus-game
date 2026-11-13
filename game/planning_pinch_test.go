package game

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/platforminput"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestPlanningModeTrackpadPinchZoom(t *testing.T) {
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)

	// Pinch events recorded while in driving mode must not carry over when entering planning mode.
	platforminput.RecordPinch(0.5)
	g.TogglePlanMode()
	if !g.IsPlanMode {
		t.Fatalf("expected IsPlanMode=true")
	}
	initialZoom := g.Camera.Zoom

	g.updatePlanningMode(1.0 / 60.0)
	if g.Camera.Zoom != initialZoom {
		t.Fatalf("stale driving pinch delta affected planning zoom: got %.4f, want %.4f", g.Camera.Zoom, initialZoom)
	}

	// 1. Trackpad two-finger up/down scrolling must NOT zoom the planning map.
	platforminput.RecordScroll(2.5, true)
	g.updatePlanningMode(1.0 / 60.0)
	if g.Camera.Zoom != initialZoom {
		t.Fatalf("trackpad two-finger scroll should not zoom planning map: got %.4f, want %.4f", g.Camera.Zoom, initialZoom)
	}

	// 2. Physical mouse scroll wheel up/down MUST still zoom the planning map.
	platforminput.RecordScroll(2.0, false)
	g.updatePlanningMode(1.0 / 60.0)
	wheelZoomedIn := g.Camera.Zoom
	if wheelZoomedIn <= initialZoom {
		t.Fatalf("expected mouse wheel scroll up to zoom in above %.4f, got %.4f", initialZoom, wheelZoomedIn)
	}
	platforminput.RecordScroll(-2.0, false)
	g.updatePlanningMode(1.0 / 60.0)
	if g.Camera.Zoom >= wheelZoomedIn {
		t.Fatalf("expected mouse wheel scroll down to zoom out below %.4f, got %.4f", wheelZoomedIn, g.Camera.Zoom)
	}

	// 3. Spreading two fingers apart (pinch out > 0) zooms in (magnifies map).
	baseZoom := g.Camera.Zoom
	platforminput.RecordPinch(0.25)
	g.updatePlanningMode(1.0 / 60.0)
	zoomedIn := g.Camera.Zoom
	if zoomedIn <= baseZoom || g.Camera.TargetZoom != zoomedIn {
		t.Fatalf("expected pinch out to increase zoom above %.4f, got Zoom=%.4f TargetZoom=%.4f",
			baseZoom, zoomedIn, g.Camera.TargetZoom)
	}

	// 4. Pinching two fingers together (pinch in < 0) zooms out (shrinks map).
	platforminput.RecordPinch(-0.15)
	g.updatePlanningMode(1.0 / 60.0)
	zoomedOut := g.Camera.Zoom
	if zoomedOut >= zoomedIn || g.Camera.TargetZoom != zoomedOut {
		t.Fatalf("expected pinch in to decrease zoom below %.4f, got Zoom=%.4f TargetZoom=%.4f",
			zoomedIn, zoomedOut, g.Camera.TargetZoom)
	}

	// 5. Extreme pinch out clamps at maxZoom (1.4).
	platforminput.RecordPinch(10.0)
	g.updatePlanningMode(1.0 / 60.0)
	if g.Camera.Zoom != 1.4 {
		t.Fatalf("expected pinch zoom max clamp 1.4, got %.4f", g.Camera.Zoom)
	}

	// 6. Extreme pinch in clamps at minZoom (0.04 for standard grid, 0.01 for large grid).
	wantMin := 0.04
	if g.World != nil && g.World.LargeGrid {
		wantMin = 0.01
	}
	platforminput.RecordPinch(-20.0)
	g.updatePlanningMode(1.0 / 60.0)
	if g.Camera.Zoom != wantMin {
		t.Fatalf("expected pinch zoom min clamp %.4f, got %.4f", wantMin, g.Camera.Zoom)
	}
}
