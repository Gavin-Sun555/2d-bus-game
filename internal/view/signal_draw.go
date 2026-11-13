package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

// DrawIntersectionStopLines paints road markings before vehicle bodies.
func DrawIntersectionStopLines(tls *sim.TrafficLightSystem, screen *ebiten.Image, cam *sim.Camera, laneWidth float64) {
	for _, inter := range tls.Intersections {
		if intersectionSignalsVisible(inter, cam, laneWidth) {
			drawIntersectionStopLines(screen, cam, inter, laneWidth)
		}
	}
}

func intersectionSignalsVisible(inter *sim.TrafficIntersection, cam *sim.Camera, laneWidth float64) bool {
	ewStop, _, nsStop, _ := inter.GetApproachDimensions(laneWidth)
	halfRoad := laneWidth + 12
	for _, lanes := range inter.ApproachLanes {
		halfRoad = math.Max(halfRoad, float64(lanes)*laneWidth/2+12)
	}
	radius := math.Hypot(math.Max(ewStop, nsStop), halfRoad) + 40/cam.Zoom
	return cam.IsPointVisible(inter.Center, radius)
}

// DrawTrafficLights renders elevated fixtures after vehicle bodies.
func DrawTrafficLights(tls *sim.TrafficLightSystem, screen *ebiten.Image, cam *sim.Camera, laneWidth float64, railways []*sim.Railway) {
	for _, inter := range tls.Intersections {
		if !intersectionSignalsVisible(inter, cam, laneWidth) {
			continue
		}

		ewThru, ewLeft, nsThru, nsLeft := inter.GetDetailedSignals()
		_, ewCross, _, nsCross := inter.GetApproachDimensions(laneWidth)
		northCross, southCross := viaductSignalOffsets(railways, inter.Center, nsCross)

		// 2. Draw traffic lights directly mounted at each connected approach zebra crossing:
		// West Approach (0, Eastbound traffic):
		if inter.ApproachHasRoad[0] || !inter.HasConfiguredRoads() {
			lanes := inter.ApproachLanes[0]
			if lanes == 0 {
				if inter.ApproachIs4Lane[0] || inter.IsMajor {
					lanes = 4
				} else {
					lanes = 2
				}
			}
			drawApproachSignal(screen, cam, inter.Center, 0, ewThru, ewLeft, lanes, laneWidth, ewCross, inter.ApproachMedianBusLane[0])
		}

		// East Approach (1, Westbound traffic):
		if inter.ApproachHasRoad[1] || !inter.HasConfiguredRoads() {
			lanes := inter.ApproachLanes[1]
			if lanes == 0 {
				if inter.ApproachIs4Lane[1] || inter.IsMajor {
					lanes = 4
				} else {
					lanes = 2
				}
			}
			drawApproachSignal(screen, cam, inter.Center, 1, ewThru, ewLeft, lanes, laneWidth, ewCross, inter.ApproachMedianBusLane[1])
		}

		// North Approach (2, Southbound traffic):
		if inter.ApproachHasRoad[2] || !inter.HasConfiguredRoads() {
			lanes := inter.ApproachLanes[2]
			if lanes == 0 {
				if inter.ApproachIs4Lane[2] || inter.IsMajor {
					lanes = 4
				} else {
					lanes = 2
				}
			}
			drawApproachSignal(screen, cam, inter.Center, 2, nsThru, nsLeft, lanes, laneWidth, northCross, inter.ApproachMedianBusLane[2])
		}

		// South Approach (3, Northbound traffic):
		if inter.ApproachHasRoad[3] || !inter.HasConfiguredRoads() {
			lanes := inter.ApproachLanes[3]
			if lanes == 0 {
				if inter.ApproachIs4Lane[3] || inter.IsMajor {
					lanes = 4
				} else {
					lanes = 2
				}
			}
			drawApproachSignal(screen, cam, inter.Center, 3, nsThru, nsLeft, lanes, laneWidth, southCross, inter.ApproachMedianBusLane[3])
		}
		signalOffsets := [4]float64{ewCross, ewCross, northCross, southCross}
		for approach, reserved := range inter.ApproachBusLane {
			lanes := inter.ApproachLanes[approach]
			if !reserved && lanes < 6 {
				continue
			}
			heading := [4]float64{0, math.Pi, math.Pi / 2, -math.Pi / 2}[approach]
			fwd := sim.UnitFromAngle(heading)
			right := sim.Vec2{X: -fwd.Y, Y: fwd.X}
			thru, left := inter.GetSignalForApproach(heading)
			base := inter.Center.Sub(fwd.Mul(signalOffsets[approach]))
			if inter.ApproachMedianBusLane[approach] {
				drawMedianBRTSignals(screen, cam, base.Add(right.Mul(.5*laneWidth)), fwd, right, laneWidth,
					[3]sim.LightSignal{left, thru, inter.GetMedianBusRightSignal(heading)})
			}
			curb := float64(lanes/2) - 1
			drawSignalHead(screen, cam, base.Add(right.Mul((curb+.25)*laneWidth)), left, signalLeft, approach < 2)
			drawSignalHead(screen, cam, base.Add(right.Mul((curb+.75)*laneWidth)), thru, signalThrough, approach < 2)
		}
	}

	// 3. Draw straight road surveillance camera gantries
	DrawRoadCameras(tls, screen, cam, laneWidth)
}

// Mount road signal heads clear of the bridge where it covers the crosswalk.
// The stop line and traffic phases still belong to the original junction.
func viaductSignalOffsets(railways []*sim.Railway, center sim.Vec2, offset float64) (north, south float64) {
	north, south = offset, offset
	for _, railway := range railways {
		for _, crossing := range railway.Crossings {
			if crossing.Level || math.Abs(crossing.Center.X-center.X) >= 1 {
				continue
			}
			top := railway.Y - railway.HalfWidth - 14
			bottom := railway.Y + railway.HalfWidth + 32
			if y := center.Y - north; y >= top && y <= bottom {
				north = center.Y - top + 10
			}
			if y := center.Y + south; y >= top && y <= bottom {
				south = bottom - center.Y + 10
			}
			break
		}
	}
	return north, south
}

// One aspect per movement keeps the three arrows legible as a single row.
// Positions and arrow directions follow the approach, including rotated cameras.
func drawMedianBRTSignals(screen *ebiten.Image, cam *sim.Camera, center, forward, right sim.Vec2, lw float64, signals [3]sim.LightSignal) {
	if cam.Zoom < .22 || !cam.IsPointVisible(center, lw) {
		return
	}
	drawRotatedBox(screen, cam, center, forward.X, forward.Y, .36*lw, .98*lw, rgb(44, 63, 77))
	stroke := math.Max(.95/cam.Zoom, .032*lw)
	for i, signal := range signals {
		pos := center.Add(right.Mul(float64(i-1) * .32 * lw))
		drawRotatedBox(screen, cam, pos, forward.X, forward.Y, .31*lw, .285*lw, rgb(12, 21, 32))
		col := rgb(255, 74, 83)
		if signal == sim.SignalGreen {
			col = rgb(72, 244, 139)
		} else if signal == sim.SignalYellow {
			col = rgb(255, 203, 71)
		}
		start := pos.Sub(forward.Mul(.105 * lw))
		dir := forward
		if i != 1 {
			dir = right.Mul(float64(i - 1))
			worldLine(screen, cam, start, pos, stroke, col)
			start = pos
		}
		tip := pos.Add(dir.Mul(.095 * lw))
		back := tip.Sub(dir.Mul(.065 * lw))
		side := sim.Vec2{X: -dir.Y, Y: dir.X}.Mul(.047 * lw)
		worldLine(screen, cam, start, tip, stroke, col)
		worldLine(screen, cam, tip, back.Add(side), stroke, col)
		worldLine(screen, cam, tip, back.Sub(side), stroke, col)
	}
}

// DrawRoadCameras renders surveillance gantries, cameras, pulsing LEDs, and violation flash effects
func DrawRoadCameras(tls *sim.TrafficLightSystem, screen *ebiten.Image, cam *sim.Camera, laneWidth float64) {
	for _, rc := range tls.RoadCameras {
		roadHalfWidth := laneWidth*2.0 + 12.0
		var camOffsets [4]float64
		numOffsets := 2
		camOffsets[0] = -laneWidth
		camOffsets[1] = laneWidth
		is6 := (rc.Lanes >= 6) || (rc.IsEW && math.Abs(rc.RoadCoord-(-1200*sim.MapGridScale)) < 15.0)
		if is6 {
			roadHalfWidth = laneWidth*float64(max(6, rc.Lanes))/2 + 12.0
			camOffsets[0] = -laneWidth * 2.0
			camOffsets[1] = -laneWidth * 1.0
			camOffsets[2] = laneWidth * 1.0
			camOffsets[3] = laneWidth * 2.0
			numOffsets = 4
		}

		sc := cam.WorldToScreen(rc.Pos)
		// View culling
		sw := float64(screen.Bounds().Dx())
		sh := float64(screen.Bounds().Dy())
		if sc.X < -150 || sc.X > sw+150 || sc.Y < -150 || sc.Y > sh+150 {
			continue
		}

		// Draw Camera Flash Flare effect when capturing a violation
		if rc.FlashTimer > 0 {
			alpha := float32(math.Min(1.0, rc.FlashTimer/0.4))
			filledCircle(screen, float32(sc.X), float32(sc.Y), float32(65*cam.Zoom),
				sim.RGBA(255, 255, 255, uint8(alpha*160)), true)
			filledCircle(screen, float32(sc.X), float32(sc.Y), float32(35*cam.Zoom),
				sim.RGBA(56, 189, 248, uint8(alpha*220)), true)
		}

		if rc.IsEW {
			// East-West road: Gantry spans across Y from (Pos.X, RoadCoord - roadHalfWidth) to (Pos.X, RoadCoord + roadHalfWidth)
			pNorth := cam.WorldToScreen(sim.Vec2{X: rc.Pos.X, Y: rc.RoadCoord - roadHalfWidth})
			pSouth := cam.WorldToScreen(sim.Vec2{X: rc.Pos.X, Y: rc.RoadCoord + roadHalfWidth})

			// Gantry main overhead beam
			beamThick := float32(math.Max(2.5, 4.0*cam.Zoom))
			strokeLine(screen, float32(pNorth.X), float32(pNorth.Y), float32(pSouth.X), float32(pSouth.Y),
				beamThick, sim.RGBA(71, 85, 105, 240), true)
			strokeLine(screen, float32(pNorth.X+1.5*cam.Zoom), float32(pNorth.Y), float32(pSouth.X+1.5*cam.Zoom), float32(pSouth.Y),
				float32(math.Max(1.0, 1.5*cam.Zoom)), sim.RGBA(148, 163, 184, 200), true)

			// Support footings with hazard stripes
			footW := float32(math.Max(4, 7*cam.Zoom))
			footH := float32(math.Max(5, 9*cam.Zoom))
			filledRect(screen, float32(pNorth.X)-footW/2, float32(pNorth.Y)-footH/2, footW, footH, sim.RGBA(234, 179, 8, 255), true)
			filledRect(screen, float32(pSouth.X)-footW/2, float32(pSouth.Y)-footH/2, footW, footH, sim.RGBA(234, 179, 8, 255), true)

			// Camera boxes mounted across the beam (South lane cam, North lane cam)
			for _, off := range camOffsets[:numOffsets] {
				cp := cam.WorldToScreen(sim.Vec2{X: rc.Pos.X, Y: rc.RoadCoord + off})
				boxW := float32(math.Max(5, 9*cam.Zoom))
				boxH := float32(math.Max(4, 7*cam.Zoom))

				// Camera housing body
				filledRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, sim.RGBA(15, 23, 42, 255), true)
				strokeRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, 1.0, sim.RGBA(148, 163, 184, 230), true)

				// Dark camera lens
				filledCircle(screen, float32(cp.X), float32(cp.Y), float32(math.Max(1.5, 2.5*cam.Zoom)), sim.RGBA(2, 6, 23, 255), true)
				// Pulsing cyan surveillance LED
				filledCircle(screen, float32(cp.X)+boxW*0.3, float32(cp.Y)-boxH*0.25, float32(math.Max(1.0, 1.8*cam.Zoom)), sim.RGBA(56, 189, 248, 240), true)
			}

			// Road asphalt marking: "📸 电子监控"
			if cam.Zoom >= 0.75 {
				textui.DrawText(screen, "📸 电子监控", sc.X-36*cam.Zoom, sc.Y-18*cam.Zoom, 10, sim.RGBA(255, 255, 255, 180))
			}
		} else {
			// North-South road: Gantry spans across X from (RoadCoord - roadHalfWidth, Pos.Y) to (RoadCoord + roadHalfWidth, Pos.Y)
			pWest := cam.WorldToScreen(sim.Vec2{X: rc.RoadCoord - roadHalfWidth, Y: rc.Pos.Y})
			pEast := cam.WorldToScreen(sim.Vec2{X: rc.RoadCoord + roadHalfWidth, Y: rc.Pos.Y})

			beamThick := float32(math.Max(2.5, 4.0*cam.Zoom))
			strokeLine(screen, float32(pWest.X), float32(pWest.Y), float32(pEast.X), float32(pEast.Y),
				beamThick, sim.RGBA(71, 85, 105, 240), true)
			strokeLine(screen, float32(pWest.X), float32(pWest.Y+1.5*cam.Zoom), float32(pEast.X), float32(pEast.Y+1.5*cam.Zoom),
				float32(math.Max(1.0, 1.5*cam.Zoom)), sim.RGBA(148, 163, 184, 200), true)

			footW := float32(math.Max(5, 9*cam.Zoom))
			footH := float32(math.Max(4, 7*cam.Zoom))
			filledRect(screen, float32(pWest.X)-footW/2, float32(pWest.Y)-footH/2, footW, footH, sim.RGBA(234, 179, 8, 255), true)
			filledRect(screen, float32(pEast.X)-footW/2, float32(pEast.Y)-footH/2, footW, footH, sim.RGBA(234, 179, 8, 255), true)

			offsetsNS := [2]float64{-laneWidth * 1.0, laneWidth * 1.0}
			for _, off := range offsetsNS {
				cp := cam.WorldToScreen(sim.Vec2{X: rc.RoadCoord + off, Y: rc.Pos.Y})
				boxW := float32(math.Max(4, 7*cam.Zoom))
				boxH := float32(math.Max(5, 9*cam.Zoom))

				filledRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, sim.RGBA(15, 23, 42, 255), true)
				strokeRect(screen, float32(cp.X)-boxW/2, float32(cp.Y)-boxH/2, boxW, boxH, 1.0, sim.RGBA(148, 163, 184, 230), true)

				filledCircle(screen, float32(cp.X), float32(cp.Y), float32(math.Max(1.5, 2.5*cam.Zoom)), sim.RGBA(2, 6, 23, 255), true)
				filledCircle(screen, float32(cp.X)-boxW*0.25, float32(cp.Y)+boxH*0.3, float32(math.Max(1.0, 1.8*cam.Zoom)), sim.RGBA(56, 189, 248, 240), true)
			}

			if cam.Zoom >= 0.75 {
				textui.DrawText(screen, "📸 电子监控", sc.X-36*cam.Zoom, sc.Y-18*cam.Zoom, 10, sim.RGBA(255, 255, 255, 180))
			}
		}
	}
}

func drawIntersectionStopLines(screen *ebiten.Image, cam *sim.Camera, inter *sim.TrafficIntersection, laneWidth float64) {
	c := inter.Center
	ewStopLineDist, _, nsStopLineDist, _ := inter.GetApproachDimensions(laneWidth)
	stopLineColor := sim.RGBA(248, 250, 252, 235) // Crisp solid white line
	lineWidth := float32(3.5 * cam.Zoom)

	// West approach stop line (0, heading East, incoming lane on +Y)
	if inter.ApproachHasRoad[0] || !inter.HasConfiguredRoads() {
		roadHalf := laneWidth*1.0 + 6.0
		if inter.ApproachLanes[0] >= 6 {
			roadHalf = laneWidth*float64(inter.ApproachLanes[0])/2 + 12.0
		} else if inter.ApproachIs4Lane[0] || (!inter.HasConfiguredRoads() && inter.IsMajor) {
			roadHalf = laneWidth*2.0 + 12.0
		}
		w1 := cam.WorldToScreen(sim.Vec2{X: c.X - ewStopLineDist, Y: c.Y + 3})
		w2 := cam.WorldToScreen(sim.Vec2{X: c.X - ewStopLineDist, Y: c.Y + roadHalf})
		strokeLine(screen, float32(w1.X), float32(w1.Y), float32(w2.X), float32(w2.Y), lineWidth, stopLineColor, true)
	}

	// East approach stop line (1, heading West, incoming lane on -Y)
	if inter.ApproachHasRoad[1] || !inter.HasConfiguredRoads() {
		roadHalf := laneWidth*1.0 + 6.0
		if inter.ApproachLanes[1] >= 6 {
			roadHalf = laneWidth*float64(inter.ApproachLanes[1])/2 + 12.0
		} else if inter.ApproachIs4Lane[1] || (!inter.HasConfiguredRoads() && inter.IsMajor) {
			roadHalf = laneWidth*2.0 + 12.0
		}
		e1 := cam.WorldToScreen(sim.Vec2{X: c.X + ewStopLineDist, Y: c.Y - 3})
		e2 := cam.WorldToScreen(sim.Vec2{X: c.X + ewStopLineDist, Y: c.Y - roadHalf})
		strokeLine(screen, float32(e1.X), float32(e1.Y), float32(e2.X), float32(e2.Y), lineWidth, stopLineColor, true)
	}

	// North approach stop line (2, heading South, incoming lane on -X)
	if inter.ApproachHasRoad[2] || !inter.HasConfiguredRoads() {
		roadHalf := laneWidth*1.0 + 6.0
		if inter.ApproachLanes[2] >= 6 {
			roadHalf = laneWidth*float64(inter.ApproachLanes[2])/2 + 12.0
		} else if inter.ApproachIs4Lane[2] || (!inter.HasConfiguredRoads() && inter.IsMajor) {
			roadHalf = laneWidth*2.0 + 12.0
		}
		n1 := cam.WorldToScreen(sim.Vec2{X: c.X - 3, Y: c.Y - nsStopLineDist})
		n2 := cam.WorldToScreen(sim.Vec2{X: c.X - roadHalf, Y: c.Y - nsStopLineDist})
		strokeLine(screen, float32(n1.X), float32(n1.Y), float32(n2.X), float32(n2.Y), lineWidth, stopLineColor, true)
	}

	// South approach stop line (3, heading North, incoming lane on +X)
	if inter.ApproachHasRoad[3] || !inter.HasConfiguredRoads() {
		roadHalf := laneWidth*1.0 + 6.0
		if inter.ApproachLanes[3] >= 6 {
			roadHalf = laneWidth*float64(inter.ApproachLanes[3])/2 + 12.0
		} else if inter.ApproachIs4Lane[3] || (!inter.HasConfiguredRoads() && inter.IsMajor) {
			roadHalf = laneWidth*2.0 + 12.0
		}
		s1 := cam.WorldToScreen(sim.Vec2{X: c.X + 3, Y: c.Y + nsStopLineDist})
		s2 := cam.WorldToScreen(sim.Vec2{X: c.X + roadHalf, Y: c.Y + nsStopLineDist})
		strokeLine(screen, float32(s1.X), float32(s1.Y), float32(s2.X), float32(s2.Y), lineWidth, stopLineColor, true)
	}
}

func drawApproachSignal(screen *ebiten.Image, cam *sim.Camera, c sim.Vec2, approach int, thruSig, leftSig sim.LightSignal, approachLanes int, laneWidth float64, crosswalkDist float64, medianBRT bool) {
	scale := cam.Zoom
	if scale < 0.20 {
		return
	}

	is6Lane := (approachLanes >= 6)
	is4Lane := (approachLanes >= 4)

	var curbPos sim.Vec2
	var armEnd sim.Vec2
	var thruPos sim.Vec2
	var thruPos2 sim.Vec2
	var leftPos sim.Vec2
	isHorizontalHead := true

	switch approach {
	case 0: // West approach (Eastbound, right curb on +Y side)
		curbY := c.Y + laneWidth*1.0 + 8.0
		if is6Lane {
			curbY = c.Y + laneWidth*float64(approachLanes)/2 + 12.0
		} else if is4Lane {
			curbY = c.Y + laneWidth*2.0 + 12.0
		}
		curbPos = sim.Vec2{X: c.X - crosswalkDist, Y: curbY}
		armEnd = sim.Vec2{X: c.X - crosswalkDist, Y: c.Y + 4.0}
		leftPos = sim.Vec2{X: c.X - crosswalkDist, Y: c.Y + laneWidth*0.52}
		if is6Lane {
			thruPos = sim.Vec2{X: c.X - crosswalkDist, Y: c.Y + laneWidth*1.50}
			thruPos2 = sim.Vec2{X: c.X - crosswalkDist, Y: c.Y + laneWidth*2.48}
		} else {
			thruPos = sim.Vec2{X: c.X - crosswalkDist, Y: c.Y + laneWidth*1.48}
		}
		isHorizontalHead = true

	case 1: // East approach (Westbound, right curb on -Y side)
		curbY := c.Y - (laneWidth*1.0 + 8.0)
		if is6Lane {
			curbY = c.Y - (laneWidth*float64(approachLanes)/2 + 12.0)
		} else if is4Lane {
			curbY = c.Y - (laneWidth*2.0 + 12.0)
		}
		curbPos = sim.Vec2{X: c.X + crosswalkDist, Y: curbY}
		armEnd = sim.Vec2{X: c.X + crosswalkDist, Y: c.Y - 4.0}
		leftPos = sim.Vec2{X: c.X + crosswalkDist, Y: c.Y - laneWidth*0.52}
		if is6Lane {
			thruPos = sim.Vec2{X: c.X + crosswalkDist, Y: c.Y - laneWidth*1.50}
			thruPos2 = sim.Vec2{X: c.X + crosswalkDist, Y: c.Y - laneWidth*2.48}
		} else {
			thruPos = sim.Vec2{X: c.X + crosswalkDist, Y: c.Y - laneWidth*1.48}
		}
		isHorizontalHead = true

	case 2: // North approach (Southbound, right curb on -X side)
		curbX := c.X - (laneWidth*1.0 + 8.0)
		if is6Lane {
			curbX = c.X - (laneWidth*float64(approachLanes)/2 + 12.0)
		} else if is4Lane {
			curbX = c.X - (laneWidth*2.0 + 12.0)
		}
		curbPos = sim.Vec2{X: curbX, Y: c.Y - crosswalkDist}
		armEnd = sim.Vec2{X: c.X - 4.0, Y: c.Y - crosswalkDist}
		leftPos = sim.Vec2{X: c.X - laneWidth*0.52, Y: c.Y - crosswalkDist}
		if is6Lane {
			thruPos = sim.Vec2{X: c.X - laneWidth*1.50, Y: c.Y - crosswalkDist}
			thruPos2 = sim.Vec2{X: c.X - laneWidth*2.48, Y: c.Y - crosswalkDist}
		} else {
			thruPos = sim.Vec2{X: c.X - laneWidth*1.48, Y: c.Y - crosswalkDist}
		}
		isHorizontalHead = false

	case 3: // South approach (Northbound, right curb on +X side)
		curbX := c.X + (laneWidth*1.0 + 8.0)
		if is6Lane {
			curbX = c.X + (laneWidth*float64(approachLanes)/2 + 12.0)
		} else if is4Lane {
			curbX = c.X + (laneWidth*2.0 + 12.0)
		}
		curbPos = sim.Vec2{X: curbX, Y: c.Y + crosswalkDist}
		armEnd = sim.Vec2{X: c.X + 4.0, Y: c.Y + crosswalkDist}
		leftPos = sim.Vec2{X: c.X + laneWidth*0.52, Y: c.Y + crosswalkDist}
		if is6Lane {
			thruPos = sim.Vec2{X: c.X + laneWidth*1.50, Y: c.Y + crosswalkDist}
			thruPos2 = sim.Vec2{X: c.X + laneWidth*2.48, Y: c.Y + crosswalkDist}
		} else {
			thruPos = sim.Vec2{X: c.X + laneWidth*1.48, Y: c.Y + crosswalkDist}
		}
		isHorizontalHead = false
	}

	// 1. Draw Curb Post Base on the sidewalk
	spCurb := cam.WorldToScreen(curbPos)
	baseR := float32(4.5 * scale)
	if baseR < 2.0 {
		baseR = 2.0
	}
	filledCircle(screen, float32(spCurb.X), float32(spCurb.Y), baseR, sim.RGBA(71, 85, 105, 255), true)
	strokeCircle(screen, float32(spCurb.X), float32(spCurb.Y), baseR, 1.0, sim.RGBA(148, 163, 184, 255), true)

	if is4Lane {
		// 2. Draw Overhead Cantilever Arm spanning across the incoming lanes
		spEnd := cam.WorldToScreen(armEnd)
		armLw := float32(3.5 * scale)
		if armLw < 1.5 {
			armLw = 1.5
		}
		// Mast shadow
		strokeLine(screen, float32(spCurb.X+3), float32(spCurb.Y+3), float32(spEnd.X+3), float32(spEnd.Y+3), armLw, sim.RGBA(2, 6, 23, 100), true)
		// Mast steel truss beam
		strokeLine(screen, float32(spCurb.X), float32(spCurb.Y), float32(spEnd.X), float32(spEnd.Y), armLw, sim.RGBA(51, 65, 85, 240), true)
		strokeLine(screen, float32(spCurb.X), float32(spCurb.Y), float32(spEnd.X), float32(spEnd.Y), armLw*0.35, sim.RGBA(148, 163, 184, 200), true)

		// The median bus lane has its own heads. General traffic turns left
		// from the next lane out, so replace that lane's usual through head.
		if medianBRT {
			leftPos = thruPos
		} else {
			drawSignalHead(screen, cam, thruPos, thruSig, signalThrough, isHorizontalHead)
		}
		drawSignalHead(screen, cam, leftPos, leftSig, signalLeft, isHorizontalHead)
		if is6Lane {
			drawSignalHead(screen, cam, thruPos2, thruSig, signalThrough, isHorizontalHead)
		}
	} else {
		// Minor 2-lane street: Signal head mounted directly at the curb post at the zebra crossing
		drawSignalHead(screen, cam, curbPos, thruSig, signalThrough, isHorizontalHead)
	}
}

type signalArrow int

const (
	signalThrough signalArrow = iota
	signalLeft
)

func drawSignalHead(screen *ebiten.Image, cam *sim.Camera, pos sim.Vec2, currentSig sim.LightSignal, arrow signalArrow, isHorizontal bool) {
	if !cam.IsPointVisible(pos, 30/cam.Zoom) {
		return
	}
	sp := cam.WorldToScreen(pos)
	scale := cam.Zoom
	if scale < 0.22 {
		return
	}

	lampR := float32(2.8 * scale)
	if lampR < 1.5 {
		lampR = 1.5
	}

	var boxW, boxH float32
	if isHorizontal {
		boxW = float32(25 * scale)
		boxH = float32(11 * scale)
	} else {
		boxW = float32(11 * scale)
		boxH = float32(25 * scale)
	}

	boxX := float32(sp.X) - boxW/2
	boxY := float32(sp.Y) - boxH/2

	// Head housing frosted glass: shadow, translucent base, specular sheen & crisp glass border
	filledRect(screen, boxX-1, boxY-1, boxW+2, boxH+2, sim.RGBA(2, 6, 23, 110), true)
	filledRect(screen, boxX, boxY, boxW, boxH, sim.RGBA(15, 23, 42, 215), true)
	filledRect(screen, boxX+0.5, boxY+0.5, boxW-1, boxH*0.45, sim.RGBA(255, 255, 255, 25), true)
	strokeRect(screen, boxX, boxY, boxW, boxH, 1.0, sim.RGBA(255, 255, 255, 75), true)

	if isHorizontal {
		cy := boxY + boxH/2
		drawSignalLamp(screen, boxX+boxW*0.20, cy, lampR, sim.SignalRed, currentSig, arrow, scale)
		drawSignalLamp(screen, boxX+boxW*0.50, cy, lampR, sim.SignalYellow, currentSig, arrow, scale)
		drawSignalLamp(screen, boxX+boxW*0.80, cy, lampR, sim.SignalGreen, currentSig, arrow, scale)
	} else {
		cx := boxX + boxW/2
		drawSignalLamp(screen, cx, boxY+boxH*0.20, lampR, sim.SignalRed, currentSig, arrow, scale)
		drawSignalLamp(screen, cx, boxY+boxH*0.50, lampR, sim.SignalYellow, currentSig, arrow, scale)
		drawSignalLamp(screen, cx, boxY+boxH*0.80, lampR, sim.SignalGreen, currentSig, arrow, scale)
	}
}

func drawSignalLamp(screen *ebiten.Image, cx, cy, radius float32, lampKind sim.LightSignal, currentSig sim.LightSignal, arrow signalArrow, scale float64) {
	isActive := (lampKind == currentSig)

	var lampColor, glowColor color.RGBA
	switch lampKind {
	case sim.SignalRed:
		if isActive {
			lampColor = sim.RGBA(248, 40, 40, 255)
			glowColor = sim.RGBA(248, 40, 40, 80)
		} else {
			lampColor = sim.RGBA(65, 15, 15, 230)
		}
	case sim.SignalYellow:
		if isActive {
			lampColor = sim.RGBA(250, 204, 21, 255)
			glowColor = sim.RGBA(250, 204, 21, 80)
		} else {
			lampColor = sim.RGBA(65, 55, 15, 230)
		}
	case sim.SignalGreen:
		if isActive {
			lampColor = sim.RGBA(34, 197, 94, 255)
			glowColor = sim.RGBA(34, 197, 94, 80)
		} else {
			lampColor = sim.RGBA(15, 55, 25, 230)
		}
	}

	// Glowing halo for active lamp
	if isActive {
		filledCircle(screen, cx, cy, radius*1.8, glowColor, true)
	}

	// Main lamp circle
	filledCircle(screen, cx, cy, radius, lampColor, true)

	// Arrow glyph for directional lenses
	if scale >= 0.38 {
		arrowCol := sim.RGBA(255, 255, 255, 240)
		if !isActive {
			arrowCol = sim.RGBA(100, 116, 139, 120)
		}
		aLw := float32(1.2 * scale)
		if aLw < 1.0 {
			aLw = 1.0
		}

		if arrow == signalLeft {
			tip := cx - radius*.4
			back := cx - radius*.1
			strokeLine(screen, cx+radius*.4, cy, tip, cy, aLw, arrowCol, true)
			strokeLine(screen, tip, cy, back, cy-radius*.4, aLw, arrowCol, true)
			strokeLine(screen, tip, cy, back, cy+radius*.4, aLw, arrowCol, true)
		} else if isActive && lampKind == sim.SignalGreen {
			// Straight arrow (⬆) inside green lamp
			strokeLine(screen, cx, cy+radius*0.4, cx, cy-radius*0.4, aLw, arrowCol, true)
			strokeLine(screen, cx, cy-radius*0.4, cx-radius*0.4, cy-radius*0.1, aLw, arrowCol, true)
			strokeLine(screen, cx, cy-radius*0.4, cx+radius*0.4, cy-radius*0.1, aLw, arrowCol, true)
		}
	}
}
