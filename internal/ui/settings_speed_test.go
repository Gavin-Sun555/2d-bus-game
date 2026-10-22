package ui

import (
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"testing"
)

func TestAISpeedSettingsLimitsAndLayout(t *testing.T) {
	s := NewSettings()
	if s.AICarMinSpeed != 30 || s.AICarMaxSpeed != 60 || s.AIBusMinSpeed != 30 || s.AIBusMaxSpeed != 50 {
		t.Fatal("wrong default AI speed ranges")
	}
	for i := 0; i < 20; i++ {
		s.AdjustAISpeed(1, 10)
		s.AdjustAISpeed(3, 10)
	}
	if s.AICarMaxSpeed != 100 || s.AIBusMaxSpeed != 80 {
		t.Fatal("wrong upper bounds")
	}
	for i := 0; i < 20; i++ {
		s.AdjustAISpeed(0, 10)
		s.AdjustAISpeed(2, 10)
	}
	if s.AICarMinSpeed != 100 || s.AIBusMinSpeed != 80 {
		t.Fatal("minimum not clamped to maximum")
	}
	for i := 0; i < 20; i++ {
		s.AdjustAISpeed(1, -10)
		s.AdjustAISpeed(3, -10)
	}
	if s.AICarMaxSpeed != 100 || s.AIBusMaxSpeed != 80 {
		t.Fatal("maximum fell below minimum")
	}
	for i := 0; i < 20; i++ {
		s.AdjustAISpeed(0, -10)
		s.AdjustAISpeed(2, -10)
		s.AdjustAISpeed(1, -10)
		s.AdjustAISpeed(3, -10)
	}
	if s.AICarMinSpeed != 30 || s.AICarMaxSpeed != 30 || s.AIBusMinSpeed != 30 || s.AIBusMaxSpeed != 30 {
		t.Fatal("wrong lower limits")
	}
	s.IsOpen = true
	s.Draw(ebiten.NewImage(1280, 720))
	for _, buttons := range s.speedRangeBounds {
		for _, b := range buttons {
			if b.X < 0 || b.Y < 0 || b.X+b.W > 1280 || b.Y+b.H > s.closeButtonBounds.Y {
				t.Fatalf("speed control outside settings card: %+v", b)
			}
		}
	}
}
