package sim

import (
	"math"
	"testing"
)

func TestMetropolisLargeRoadNetworkScaling(t *testing.T) {
	lw := 52.0

	// 1. Standard Metropolis World (1x)
	wStd := NewWorld(lw, MapThemeMetropolis)
	if wStd.LargeGrid {
		t.Fatalf("Expected standard world LargeGrid=false")
	}
	if wStd.GridScale() != MapGridScale {
		t.Fatalf("Expected standard GridScale=%.1f, got %.1f", MapGridScale, wStd.GridScale())
	}

	// 2. Large Road Network Metropolis World (4x)
	wLarge := &World{
		Theme:     MapThemeMetropolis,
		LargeGrid: true,
	}
	wLarge.BuildMap(MapThemeMetropolis, lw)

	if !wLarge.LargeGrid {
		t.Fatalf("Expected large world LargeGrid=true")
	}
	expectedScale := MapGridScale * 4.0
	if wLarge.GridScale() != expectedScale {
		t.Fatalf("Expected large GridScale=%.1f, got %.1f", expectedScale, wLarge.GridScale())
	}

	// 3. Verify Footprint: exactly 4x
	stdMinX, stdMaxX, stdMinY, stdMaxY := getBounds(wStd.Roads)
	largeMinX, largeMaxX, largeMinY, largeMaxY := getBounds(wLarge.Roads)

	stdW := stdMaxX - stdMinX
	stdH := stdMaxY - stdMinY
	largeW := largeMaxX - largeMinX
	largeH := largeMaxY - largeMinY

	if math.Abs(largeW-stdW*4.0) > 1.0 {
		t.Fatalf("Expected large width to be 4x standard: std=%.1f, large=%.1f", stdW, largeW)
	}
	if math.Abs(largeH-stdH*4.0) > 1.0 {
		t.Fatalf("Expected large height to be 4x standard: std=%.1f, large=%.1f", stdH, largeH)
	}

	// 4. Verify Intersection Spacing: exactly 4x
	// Along Central Boulevard (Y=0), intersections are at X = -5200..5200 in base units
	// Standard: 1300 * 3.0 = 3900 px spacing
	// Large: 1300 * 12.0 = 15600 px spacing (4x)
	var stdCentralX, largeCentralX []float64
	for _, it := range wStd.Intersections {
		if math.Abs(it.Y) < 1.0 {
			stdCentralX = append(stdCentralX, it.X)
		}
	}
	for _, it := range wLarge.Intersections {
		if math.Abs(it.Y) < 1.0 {
			largeCentralX = append(largeCentralX, it.X)
		}
	}

	if len(stdCentralX) != len(largeCentralX) {
		t.Fatalf("Intersection count mismatch on central avenue: std=%d, large=%d", len(stdCentralX), len(largeCentralX))
	}

	// 5. Verify Stops: block layout topology is preserved
	if len(wLarge.Stops) != len(wStd.Stops) {
		t.Fatalf("Stop count mismatch: std=%d, large=%d", len(wStd.Stops), len(wLarge.Stops))
	}

	// Verify each stop's road center coordinate is scaled by 4x
	for i := range wStd.Stops {
		stdStop := wStd.Stops[i]
		largeStop := wLarge.Stops[i]
		if math.Abs(largeStop.RoadCenter.X-stdStop.RoadCenter.X*4.0) > 2.0 ||
			math.Abs(largeStop.RoadCenter.Y-stdStop.RoadCenter.Y*4.0) > 2.0 {
			t.Fatalf("Stop %d position mismatch: std=%v, large=%v", i, stdStop.RoadCenter, largeStop.RoadCenter)
		}
	}

	// 6. Verify Traffic Light Cameras
	tlsStd := NewTrafficLightSystem()
	tlsStd.BuildForWorld(wStd)

	tlsLarge := NewTrafficLightSystem()
	tlsLarge.BuildForWorld(wLarge)

	if len(tlsLarge.RoadCameras) != len(tlsStd.RoadCameras) {
		t.Fatalf("Camera count mismatch: std=%d, large=%d", len(tlsStd.RoadCameras), len(tlsLarge.RoadCameras))
	}

	for i := range tlsStd.RoadCameras {
		cStd := tlsStd.RoadCameras[i]
		cLarge := tlsLarge.RoadCameras[i]
		if math.Abs(cLarge.Pos.X-cStd.Pos.X*4.0) > 2.0 || math.Abs(cLarge.Pos.Y-cStd.Pos.Y*4.0) > 2.0 {
			t.Fatalf("Camera %d position mismatch: std=%v, large=%v", i, cStd.Pos, cLarge.Pos)
		}
		if math.Abs(cLarge.RoadCoord-cStd.RoadCoord*4.0) > 2.0 {
			t.Fatalf("Camera %d RoadCoord mismatch: std=%.1f, large=%.1f", i, cStd.RoadCoord, cLarge.RoadCoord)
		}
	}
}

func getBounds(roads []RoadSegment) (minX, maxX, minY, maxY float64) {
	minX, minY = math.Inf(1), math.Inf(1)
	maxX, maxY = math.Inf(-1), math.Inf(-1)
	for _, r := range roads {
		for _, p := range []Vec2{r.P1, r.P2} {
			minX = math.Min(minX, p.X)
			maxX = math.Max(maxX, p.X)
			minY = math.Min(minY, p.Y)
			maxY = math.Max(maxY, p.Y)
		}
	}
	return
}
