package ui

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func TestFareControlsAndOperationsPanelLayout(t *testing.T) {
	f := sim.DefaultFarePolicy()
	for i := 0; i < 30; i++ {
		AdjustFare(&f, 1, 1)
		AdjustFare(&f, 6, -1)
	}
	if f.Base != 8 || f.Cap != 9 {
		t.Fatal("cap must stay strictly above base")
	}
	for field := 0; field < 7; field++ {
		for i := 0; i < 150; i++ {
			AdjustFare(&f, field, -1)
		}
	}
	if f.Fixed != 0 || f.Base != 0 || f.Cap != 1 || f.StopsPerYuan != 1 || f.PerKM != 0 {
		t.Fatal("fare minimum bounds incorrect")
	}
	w := sim.NewWorld(52, sim.MapThemeMetropolis)
	r := sim.GetStandardRouteForMap(w.Theme, w)
	o := sim.NewTransitOperations(w, r, 52, sim.DefaultFarePolicy())
	for id := 0; id <= 8; id++ {
		v := o.Vehicle(id)
		v.Route, v.LineName = r, r.Name
		v.Revenue = 123 + id
		v.Boarded = 50
		v.Alighted = 34
		v.CompletedLaps = 1
		v.CompletedKM = 5
		v.CompletedSeconds = 900
		v.KM = 5.5
		o.Revenue += v.Revenue
	}
	original := locale.CurrentLanguage
	defer func() { locale.CurrentLanguage = original }()
	for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
		locale.CurrentLanguage = lang
		for tab := 0; tab < 4; tab++ {
			p := NewOperationsPanel()
			p.IsOpen = true
			p.Tab = tab
			screen := ebiten.NewImage(1280, 720)
			f.Mode = sim.FareByDistance
			p.Draw(screen, f, o, w, r)
			if p.closeBounds.X+p.closeBounds.W > 1280 || p.tabBounds[2].Y+p.tabBounds[2].H > 720 {
				t.Fatal("panel controls outside viewport")
			}
			if tab == 0 {
				for _, field := range []int{1, 4, 5, 6} {
					if p.fieldBounds[field][1].W == 0 {
						t.Fatal("missing fare field")
					}
				}
			}
		}
	}
}

// A long station list must not determine scrolling on the vehicle statistics tab.
func TestVehiclesTabUsesVehicleRows(t *testing.T) {
	w := sim.NewWorld(52)
	r := sim.NewRoute()
	for i := 0; i < 30; i++ {
		r.AddStop(i)
	}
	o := sim.NewTransitOperations(w, r, 52, sim.DefaultFarePolicy())
	o.Vehicle(0)
	p := NewOperationsPanel()
	p.IsOpen = true
	p.Tab, p.Scroll = 1, 999
	screen := ebiten.NewImage(1280, 720)
	p.Draw(screen, sim.DefaultFarePolicy(), o, w)
	if p.Scroll == 0 {
		t.Fatal("station tab should have multiple pages")
	}
	p.Tab, p.Scroll = 2, 999
	p.Draw(screen, sim.DefaultFarePolicy(), o, w)
	if p.Scroll != 0 {
		t.Fatal("vehicle tab rendered station rows")
	}
	for _, tab := range p.tabBounds {
		if tab.Contains(sim.Vec2{X: p.vehicleBounds.X, Y: p.vehicleBounds.Y}) {
			t.Fatal("vehicle selector overlaps tabs")
		}
	}
}
