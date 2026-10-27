package ui

import (
	"strings"
	"testing"
	"unicode/utf8"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
)

func TestHUDTopLayoutNoOverlap(t *testing.T) {
	for _, w := range []float64{1280, 1440, 1707, 2560} {
		l := newHUDTopLayout(w)
		rects := []sim.Rect{l.mode, l.cash, l.pax, l.maps, l.camera, l.settings, l.stop, l.metrics, l.bell}
		for i, a := range rects {
			if a.X < 0 || a.Y < 0 || a.X+a.W > w || a.Y+a.H > hudTopHeight || a.W <= 20 {
				t.Fatalf("invalid HUD rectangle at width %v: %+v", w, a)
			}
			for _, b := range rects[i+1:] {
				if a.X < b.X+b.W && b.X < a.X+a.W && a.Y < b.Y+b.H && b.Y < a.Y+a.H {
					t.Fatalf("overlap at width %v: %+v %+v", w, a, b)
				}
			}
		}
		for _, s := range []string{"下一站：" + strings.Repeat("街区202 · 东行站 ", 20), "Next: " + strings.Repeat("Central Business District ", 20)} {
			got := fitHUDText(s, 13, l.stop.W-20)
			measured, _ := textui.MeasureText(got, 13)
			if !utf8.ValidString(got) || !strings.HasSuffix(got, "…") || measured > l.stop.W-20 {
				t.Fatalf("invalid truncated name: %q", got)
			}
		}
		for _, s := range []string{"距离 123456m · 候车 9999人", "123456m away · Waiting 9999"} {
			if got := fitHUDText(s, 12, l.metrics.W-20); got != s {
				t.Fatalf("distance was truncated: %q", got)
			}
		}
	}
	h := NewHUD()
	if !h.HandleModeBtnClick(sim.Vec2{X: 100, Y: 25}) || h.HandleModeBtnClick(sim.Vec2{X: 250, Y: 25}) {
		t.Fatal("mode click bounds mismatch")
	}
	if !h.IsInsideTopBar(sim.Vec2{X: 500, Y: 80}) || h.IsInsideTopBar(sim.Vec2{X: 500, Y: 100}) {
		t.Fatal("overlay click exclusion mismatch")
	}
}

func TestHUDTextFitEmptyAndExact(t *testing.T) {
	for _, s := range []string{"", "下一站", "Next: Main Street"} {
		w, _ := textui.MeasureText(s, 13)
		if got := fitHUDText(s, 13, w); got != s {
			t.Fatalf("changed fitting text %q to %q", s, got)
		}
	}
	if fitHUDText("长站名", 13, 0) != "" {
		t.Fatal("text escaped zero width")
	}
}
