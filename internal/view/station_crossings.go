package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/hajimehoshi/ebiten/v2"
)

func drawStationCrossingGround(w *sim.World, screen *ebiten.Image, cam *sim.Camera, lw float64) {
	if cam.Zoom < .12 {
		return
	}
	for _, crossing := range w.StationCrossings() {
		half := float64(crossing.Lanes) * lw / 2
		landing := half + sim.ShoulderWidth + 11
		if !cam.IsPointVisible(crossing.Center, landing+48) {
			continue
		}
		forward := sim.UnitFromAngle(crossing.Heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		box := func(pos sim.Vec2, depth, width float64, c color.RGBA) {
			drawRotatedBox(screen, cam, pos, forward.X, forward.Y, depth, width, c)
		}
		switch crossing.Kind {
		case sim.StationFootbridge:
			box(crossing.Center.Add(forward.Mul(8)), 22, landing*2, sim.RGBA(8, 15, 26, 105))
		case sim.StationUnderpass:
			box(crossing.Center, 20, landing*2, sim.RGBA(25, 44, 63, 95))
			if cam.Zoom >= .28 {
				for lateral := -half + 10; lateral < half-8; lateral += 18 {
					for _, sign := range [...]float64{-1, 1} {
						p := crossing.Center.Add(forward.Mul(sign * 9)).Add(right.Mul(lateral))
						box(p, 2, 10, sim.RGBA(109, 178, 208, 145))
					}
				}
			}
			for _, side := range [...]float64{-1, 1} {
				p := crossing.Center.Add(right.Mul(side * landing))
				box(p, 39, 20, rgb(139, 161, 178))
				box(p, 31, 14, rgb(25, 41, 57))
				if cam.Zoom >= .28 {
					for tread := -11.; tread <= 12; tread += 4.5 {
						box(p.Add(forward.Mul(tread)), 1.5, 13, rgb(166, 192, 203))
					}
				}
			}
		case sim.StationZebra, sim.StationSignalZebra:
			if cam.Zoom < .22 {
				box(crossing.Center, 14, 2*half, sim.RGBA(232, 241, 245, 180))
			} else {
				count := max(1, int(2*half/14))
				step := 2 * half / float64(count)
				for i := 0; i < count; i++ {
					p := crossing.Center.Add(right.Mul(-half + step*(float64(i)+.5)))
					box(p, 31, step*.57, rgb(232, 241, 245))
				}
			}
			if crossing.Kind == sim.StationSignalZebra {
				for _, side := range [...]float64{-1, 1} {
					stop := crossing.Center.Add(forward.Mul(-side * sim.StationCrossingStopOffset)).Add(right.Mul(side * half / 2))
					box(stop, 4, half, rgb(232, 241, 245))
				}
			}
		}
	}
}

// DrawElevatedStationCrossings draws the decks and signals after vehicle
// bodies. Ground markings and bridge shadows are drawn with the street.
func DrawElevatedStationCrossings(w *sim.World, tls *sim.TrafficLightSystem, screen *ebiten.Image, cam *sim.Camera, lw float64, plan bool) {
	if cam.Zoom < .12 {
		return
	}
	darkness := w.Clock.Darkness()
	if plan {
		darkness = math.Min(.28, darkness)
	}
	shade := func(c color.RGBA) color.RGBA { return tint(c, rgb(5, 12, 28), darkness) }
	for _, crossing := range w.StationCrossings() {
		if crossing.Kind != sim.StationFootbridge && crossing.Kind != sim.StationSignalZebra {
			continue
		}
		half := float64(crossing.Lanes) * lw / 2
		landing := half + sim.ShoulderWidth + 11
		if !cam.IsPointVisible(crossing.Center, landing+58) {
			continue
		}
		forward := sim.UnitFromAngle(crossing.Heading)
		right := sim.Vec2{X: -forward.Y, Y: forward.X}
		box := func(pos sim.Vec2, depth, width float64, c color.RGBA) {
			drawRotatedBox(screen, cam, pos, forward.X, forward.Y, depth, width, c)
		}
		if crossing.Kind == sim.StationFootbridge {
			box(crossing.Center, 22, landing*2, shade(rgb(65, 92, 110)))
			box(crossing.Center, 15, landing*2-4, shade(rgb(132, 191, 208)))
			box(crossing.Center, 7, landing*2-8, shade(rgb(203, 222, 224)))
			for _, side := range [...]float64{-1, 1} {
				p := crossing.Center.Add(right.Mul(side * landing))
				box(p, 43, 23, shade(rgb(69, 93, 111)))
				box(p, 34, 17, shade(rgb(142, 170, 182)))
				if cam.Zoom >= .28 {
					for step := -12.; step <= 12; step += 5 {
						box(p.Add(forward.Mul(step)), 1.6, 16, shade(rgb(221, 231, 235)))
					}
				}
			}
			continue
		}
		phase := tls.StationSignalPhase(crossing)
		for _, side := range [...]float64{-1, 1} {
			line := crossing.Center.Add(forward.Mul(-side * sim.StationCrossingStopOffset))
			base := line.Add(right.Mul(side * (half + 17)))
			head := line.Add(right.Mul(side * (half - 10)))
			worldLine(screen, cam, base, head, 2.5, shade(rgb(155, 170, 182)))
			box(head, 10, 24, shade(rgb(26, 38, 48)))
			for i, lamp := range [...]struct {
				phase sim.StationCrossingSignal
				col   color.RGBA
			}{{sim.StationSignalRed, rgb(255, 57, 65)}, {sim.StationSignalYellow, rgb(255, 192, 66)}, {sim.StationSignalGreen, rgb(61, 234, 116)}} {
				col := shade(rgb(65, 77, 84))
				if lamp.phase == phase {
					col = lamp.col
				}
				pos := head.Add(right.Mul(float64(i-1) * 7 * side))
				worldCircle(screen, cam, pos, 2.9, col)
			}
		}
	}
}
