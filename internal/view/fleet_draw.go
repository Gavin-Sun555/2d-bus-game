package view

import (
	"fmt"
	"image/color"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// Draw renders all AI route buses with customized transit liveries and roof badges
func DrawFleet(abm *traffic.AIBusManager, screen *ebiten.Image, cam *sim.Camera) {
	if cam == nil || len(abm.Buses) == 0 {
		return
	}

	for _, ab := range abm.Buses {
		if !cam.IsPointVisible(ab.Bus.Pos, 140) {
			continue
		}

		drawBusBody(screen, cam, ab.Bus.Pos, ab.Bus.Heading, ab.Bus.Length, ab.Bus.Width, ab.Color, true)
		drawBusWheels(ab.Bus, screen, cam)
		drawBusRoof(ab.Bus, screen, cam)
		drawBusLights(ab.Bus, screen, cam)
		drawBusDoors(ab.Bus, screen, cam)

		// Overhead glowing LED Route Badge
		if cam.Zoom >= 0.28 {
			sPos := cam.WorldToScreen(ab.Bus.Pos)
			badgeW := 105.0
			badgeH := 20.0
			badgeX := sPos.X - badgeW/2
			badgeY := sPos.Y - 32.0*cam.Zoom - badgeH

			vector.DrawFilledRect(screen, float32(badgeX), float32(badgeY), float32(badgeW), float32(badgeH),
				sim.RGBA(15, 23, 42, 230), true)
			vector.StrokeRect(screen, float32(badgeX), float32(badgeY), float32(badgeW), float32(badgeH),
				1.2, ab.Color, true)

			badgeText := fmt.Sprintf("[1号线] AI#%d 👥%d/%d", ab.ID, ab.Bus.PassengerCount, ab.Bus.Capacity)
			textui.DrawText(screen, badgeText, badgeX+6, badgeY+4, 10, color.White)
		}
	}
}
