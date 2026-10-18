package view

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

// DrawRouteLine draws connecting route lines along the driving lanes on the side of the road.
// In Driver Mode (!isPlanMode), it strictly draws ONLY the route to the next target stop, eliminating road confusion.
// In Planning Mode (isPlanMode), it draws the full planned transit line with the upcoming segment highlighted.
func DrawRouteLine(r *sim.Route, screen *ebiten.Image, cam *sim.Camera, world *sim.World, isPlanMode bool, laneWidth ...float64) {
	if len(r.StopIDs) < 1 {
		return
	}

	lw := 42.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}

	if !isPlanMode {
		// DRIVER MODE: Only draw the suggested route leading to the next stop!
		targetStopID := r.GetCurrentTargetStopID()
		if targetStopID < 0 || targetStopID >= len(world.Stops) {
			return
		}

		prevIdx := (r.CurrentIndex - 1 + len(r.StopIDs)) % len(r.StopIDs)
		prevStopID := r.StopIDs[prevIdx]

		s1 := world.Stops[prevStopID]
		s2 := world.Stops[targetStopID]

		var path []sim.Vec2
		if s1.ID != s2.ID {
			path = sim.GetRoadPathBetweenStopsInWorld(world, s1, s2, lw)
		} else if len(r.StopIDs) > 1 {
			path = []sim.Vec2{s1.Pos, s2.Pos}
		}

		if len(path) < 2 {
			return
		}

		lineColor := sim.RGBA(250, 204, 21, 240) // Glowing Gold for active navigation line
		drawSegmentPath(screen, cam, path, lineColor)
		return
	}

	// PLANNING MODE: Draw all planned segments so player can review full line
	for i := 0; i < len(r.StopIDs); i++ {
		currID := r.StopIDs[i]
		nextID := r.StopIDs[(i+1)%len(r.StopIDs)]

		s1 := world.Stops[currID]
		s2 := world.Stops[nextID]
		if s1.ID == s2.ID {
			continue
		}

		path := sim.GetRoadPathBetweenStopsInWorld(world, s1, s2, lw)
		lineColor := sim.RGBA(r.Color.R, r.Color.G, r.Color.B, 130)
		if i == (r.CurrentIndex-1+len(r.StopIDs))%len(r.StopIDs) || (r.CurrentIndex == 0 && i == len(r.StopIDs)-1) {
			lineColor = sim.RGBA(250, 204, 21, 240) // Highlight upcoming segment in gold
		}
		drawSegmentPath(screen, cam, path, lineColor)
	}
}

func drawSegmentPath(screen *ebiten.Image, cam *sim.Camera, path []sim.Vec2, lineColor color.RGBA) {
	strokeWidth := float32(math.Max(2.5, 5.0*cam.Zoom))
	for sIdx := 0; sIdx < len(path)-1; sIdx++ {
		w1 := path[sIdx]
		w2 := path[sIdx+1]

		p1 := cam.WorldToScreen(w1)
		p2 := cam.WorldToScreen(w2)

		// Route line along the road
		vector.StrokeLine(screen, float32(p1.X), float32(p1.Y), float32(p2.X), float32(p2.Y),
			strokeWidth, lineColor, true)

		// Draw directional arrows along segments that are sufficiently long on screen
		segLen := w1.Distance(w2)
		if segLen*cam.Zoom > 22 {
			mid := p1.Add(p2).Mul(0.5)
			dir := p2.Sub(p1).Normalize()
			perp := sim.Vec2{X: -dir.Y, Y: dir.X}
			arrLen := math.Max(8.0, 12*cam.Zoom)
			arrWid := math.Max(4.5, 7*cam.Zoom)
			arr1 := mid.Sub(dir.Mul(arrLen)).Add(perp.Mul(arrWid))
			arr2 := mid.Sub(dir.Mul(arrLen)).Sub(perp.Mul(arrWid))
			arrLw := float32(math.Max(1.8, 3.0*cam.Zoom))
			vector.StrokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr1.X), float32(arr1.Y),
				arrLw, lineColor, true)
			vector.StrokeLine(screen, float32(mid.X), float32(mid.Y), float32(arr2.X), float32(arr2.Y),
				arrLw, lineColor, true)
		}
	}
}

// DrawNavArrow draws a compass/directional navigation arrow around the bus towards the target stop along roads
func DrawNavArrow(r *sim.Route, screen *ebiten.Image, cam *sim.Camera, bus *sim.Bus, targetStop *sim.BusStop, worldOptional ...*sim.World) {
	if targetStop == nil {
		return
	}

	dist := bus.Pos.Distance(targetStop.Pos)
	if dist < 65 {
		return // Already arrived at stop
	}

	var w *sim.World = sim.ActiveWorldForRoute
	if len(worldOptional) > 0 && worldOptional[0] != nil {
		w = worldOptional[0]
	}

	targetPoint := targetStop.Pos

	if w != nil && len(r.StopIDs) > 0 {
		prevIdx := (r.CurrentIndex - 1 + len(r.StopIDs)) % len(r.StopIDs)
		prevStopID := r.StopIDs[prevIdx]
		if prevStopID >= 0 && prevStopID < len(w.Stops) {
			prevStop := w.Stops[prevStopID]
			path := sim.GetRoadPathBetweenStopsInWorld(w, prevStop, targetStop)
			if len(path) >= 2 {
				closestIdx := 0
				minDist := math.MaxFloat64
				for idx, pt := range path {
					d := bus.Pos.Distance(pt)
					if d < minDist {
						minDist = d
						closestIdx = idx
					}
				}
				// Look ahead along path to find the next waypoint ahead of the bus
				for idx := closestIdx; idx < len(path); idx++ {
					if bus.Pos.Distance(path[idx]) > 70 {
						targetPoint = path[idx]
						break
					}
				}
			}
		}
	}

	// Direction towards target point
	dir := targetPoint.Sub(bus.Pos).Normalize()
	angleToStop := math.Atan2(dir.Y, dir.X)
	screenAngle := angleToStop - cam.Rotation

	// Draw floating navigation arrow around bus
	busScreen := cam.WorldToScreen(bus.Pos)
	navRadius := 68.0 * cam.Zoom
	arrowCenter := busScreen.Add(sim.Vec2{
		X: math.Cos(screenAngle) * navRadius,
		Y: math.Sin(screenAngle) * navRadius,
	})

	cosA := math.Cos(screenAngle)
	sinA := math.Sin(screenAngle)
	perpX := -sinA
	perpY := cosA

	// Triangle pointer
	tip := arrowCenter.Add(sim.Vec2{X: cosA * 15, Y: sinA * 15})
	base1 := arrowCenter.Add(sim.Vec2{X: -cosA*8 + perpX*8, Y: -sinA*8 + perpY*8})
	base2 := arrowCenter.Add(sim.Vec2{X: -cosA*8 - perpX*8, Y: -sinA*8 - perpY*8})

	vector.DrawFilledCircle(screen, float32(arrowCenter.X), float32(arrowCenter.Y), float32(16*cam.Zoom),
		sim.RGBA(15, 23, 42, 190), true)
	vector.StrokeCircle(screen, float32(arrowCenter.X), float32(arrowCenter.Y), float32(16*cam.Zoom),
		float32(1.5), sim.RGBA(250, 204, 21, 255), true)

	// Draw pointer
	vector.StrokeLine(screen, float32(tip.X), float32(tip.Y), float32(base1.X), float32(base1.Y), 2.5, sim.RGBA(250, 204, 21, 255), true)
	vector.StrokeLine(screen, float32(tip.X), float32(tip.Y), float32(base2.X), float32(base2.Y), 2.5, sim.RGBA(250, 204, 21, 255), true)
	vector.StrokeLine(screen, float32(base1.X), float32(base1.Y), float32(base2.X), float32(base2.Y), 2.5, sim.RGBA(250, 204, 21, 255), true)
}
