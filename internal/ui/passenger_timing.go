package ui

import (
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (s *Settings) ModelPassengerRates(model sim.BusModel) sim.PassengerRates {
	if s.PassengerRates == nil || !model.Valid() {
		return sim.DefaultPassengerRates(model)
	}
	return s.PassengerRates[model].ForModel(model)
}
func (s *Settings) CyclePassengerRate(model sim.BusModel, door int) {
	if !model.Valid() || door < 0 || door > 1 {
		return
	}
	var rates [sim.BusModelCount]sim.PassengerRates
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		rates[m] = s.ModelPassengerRates(m)
	}
	current := rates[model].Boarding
	if door == 1 {
		current = rates[model].Alighting
	}
	for i, v := range sim.PassengerTimeOptions {
		if v == current {
			next := sim.PassengerTimeOptions[(i+1)%len(sim.PassengerTimeOptions)]
			if door == 0 {
				rates[model].Boarding = next
			} else {
				rates[model].Alighting = next
			}
			break
		}
	}
	s.PassengerRates = &rates
}
func (s *Settings) drawPassengerTiming(screen *ebiten.Image) {
	w, h := float64(screen.Bounds().Dx()), float64(screen.Bounds().Dy())
	x, y := (w-640)/2, (h-350)/2
	vector.DrawFilledRect(screen, 0, 0, float32(w), float32(h), sim.RGBA(0, 0, 0, 180), false)
	vector.DrawFilledRect(screen, float32(x), float32(y), 640, 350, sim.RGBA(15, 23, 42, 255), true)
	textui.DrawText(screen, locale.T("各车型上下客用时（秒 / 人）", "Passenger service time (seconds / person)"), x+24, y+22, 16, sim.RGBA(255, 255, 255, 255))
	textui.DrawText(screen, locale.T("车型", "Bus model"), x+24, y+65, 13, sim.RGBA(148, 163, 184, 255))
	textui.DrawText(screen, locale.T("上车 · 点击切换", "Boarding · click to cycle"), x+205, y+65, 13, sim.RGBA(148, 163, 184, 255))
	textui.DrawText(screen, locale.T("下车 · 点击切换", "Alighting · click to cycle"), x+415, y+65, 13, sim.RGBA(148, 163, 184, 255))
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		row := y + 92 + float64(m)*44
		textui.DrawText(screen, locale.T(m.Name(), m.EnglishName()), x+24, row+8, 14, sim.RGBA(226, 232, 240, 255))
		r := s.ModelPassengerRates(m)
		for door, value := range []float64{r.Boarding, r.Alighting} {
			b := sim.Rect{X: x + 205 + float64(door)*210, Y: row, W: 186, H: 30}
			s.passengerRateBounds[m][door] = b
			operationsButton(screen, b, fmt.Sprintf("%g s", value), false)
		}
	}
	textui.DrawText(screen, "0.25 / 0.5 / 0.75 / 1 / 1.5 / 2 / 3 s", x+24, y+237, 13, sim.RGBA(56, 189, 248, 255))
	textui.DrawText(screen, locale.T("下次靠站生效；多车按各自速度持续上客，直到满载或站台无人。", "Applies at next stop. Each bus boards at its own rate until full or queue empty."), x+24, y+266, 11, sim.RGBA(148, 163, 184, 255))
	s.passengerTimingClose = sim.Rect{X: x + 24, Y: y + 302, W: 592, H: 30}
	operationsButton(screen, s.passengerTimingClose, locale.T("返回设置 [ESC]", "Back to settings [ESC]"), true)
}
