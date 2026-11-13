package ui

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

func (s *Settings) ModelPassengerOverload(model sim.BusModel) int {
	if !model.Valid() {
		return 0
	}
	if s.PassengerOverloadPercents != nil {
		return min(50, max(0, (*s.PassengerOverloadPercents)[model]))
	}
	if s.PassengerCapacities != nil {
		// Recover the old integer allowance despite flooring its passenger count,
		// then round to the nearest new ten-percent step (25% becomes 30%).
		extra := max(0, (*s.PassengerCapacities)[model]-model.Capacity())
		percent := (extra*100 + model.Capacity() - 1) / model.Capacity()
		return min(50, (percent+5)/10*10)
	}
	return 0
}
func (s *Settings) ModelPassengerCapacity(model sim.BusModel) int {
	return model.CapacityWithOverload(s.ModelPassengerOverload(model))
}

func (s *Settings) NormalizePassengerOverloads() {
	if s.PassengerCapacities == nil {
		return
	}
	var percentages [sim.BusModelCount]int
	for model := sim.BusStandard; model < sim.BusModelCount; model++ {
		percentages[model] = s.ModelPassengerOverload(model)
	}
	s.PassengerOverloadPercents = &percentages
	s.PassengerCapacities = nil
}

func (s *Settings) AdjustPassengerOverload(model sim.BusModel, steps int) {
	if !model.Valid() {
		return
	}
	var percentages [sim.BusModelCount]int
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		percentages[m] = s.ModelPassengerOverload(m)
	}
	percentages[model] = min(50, max(0, percentages[model]+steps*10))
	s.PassengerOverloadPercents = &percentages
	s.PassengerCapacities = nil
}

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
	textui.DrawText(screen, locale.T("各车型超载比例与上下客用时", "Bus overload allowance and service times"), x+24, y+22, 16, sim.RGBA(255, 255, 255, 255))
	textui.DrawText(screen, locale.T("车型", "Bus model"), x+24, y+65, 13, sim.RGBA(148, 163, 184, 255))
	textui.DrawText(screen, locale.T("超载比例", "Overload allowance"), x+170, y+65, 13, sim.RGBA(148, 163, 184, 255))
	textui.DrawText(screen, locale.T("上车（秒 / 人）", "Boarding (s/person)"), x+365, y+65, 13, sim.RGBA(148, 163, 184, 255))
	textui.DrawText(screen, locale.T("下车（秒 / 人）", "Alighting (s/person)"), x+495, y+65, 13, sim.RGBA(148, 163, 184, 255))
	for m := sim.BusStandard; m < sim.BusModelCount; m++ {
		row := y + 92 + float64(m)*44
		textui.DrawText(screen, locale.T(m.Name(), m.EnglishName()), x+24, row+8, 14, sim.RGBA(226, 232, 240, 255))
		for direction, label := range [2]string{"-", "+"} {
			b := sim.Rect{X: x + 170 + float64(direction)*128, Y: row, W: 30, H: 30}
			s.passengerCapacityBounds[m][direction] = b
			operationsButton(screen, b, label, false)
		}
		value := fmt.Sprintf("%d %%", s.ModelPassengerOverload(m))
		width, _ := textui.MeasureText(value, 14)
		textui.DrawText(screen, value, x+249-width/2, row+8, 14, sim.RGBA(226, 232, 240, 255))
		r := s.ModelPassengerRates(m)
		for door, value := range []float64{r.Boarding, r.Alighting} {
			b := sim.Rect{X: x + 365 + float64(door)*130, Y: row, W: 110, H: 30}
			s.passengerRateBounds[m][door] = b
			operationsButton(screen, b, fmt.Sprintf("%g s", value), false)
		}
	}
	textui.DrawText(screen, locale.T("超载比例为核定载客量的 0～50%，每档 10%；立即生效，已上车乘客保留。", "Allowance: 0–50% of rated capacity, in 10% steps. Applies now; onboard riders stay."), x+24, y+237, 12, sim.RGBA(56, 189, 248, 255))
	textui.DrawText(screen, locale.T("点击秒数切换上下客用时，下次靠站生效。", "Click a service time to cycle it. Applies at the next stop."), x+24, y+266, 11, sim.RGBA(148, 163, 184, 255))
	s.passengerTimingClose = sim.Rect{X: x + 24, Y: y + 302, W: 592, H: 30}
	operationsButton(screen, s.passengerTimingClose, locale.T("返回设置 [ESC]", "Back to settings [ESC]"), true)
}
