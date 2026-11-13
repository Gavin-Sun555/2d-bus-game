package sim

import "math"

// ConfigureCenterBRT configures median bus lanes and ensures all BRT stops are symmetric.
func (w *World) ConfigureCenterBRT(lw float64) {
	w.configureCenterBRT(lw)
}

const (
	// BRTStationMaxFlare defines the outward expansion of general traffic lanes around a central BRT station.
	// This provides space for the widened passenger platform without encroaching into the bus lane or traffic lanes.
	BRTStationMaxFlare    = 26.0
	BRTPlatformWidth      = 24.0
	BRTPlatformHalfLength = 125.0
	BRTCrossingCoreHalf   = 140.0
	BRTCrossingTaperLen   = 160.0
)

type brtFlareIndex struct {
	laneWidth float64
	stopCount int
	cells     map[junctionCell][]*BusStop
}

func (w *World) indexBRTFlares(laneWidth float64) {
	w.brtFlares = nil
	if !w.CenterBRT {
		return
	}
	index := &brtFlareIndex{laneWidth: laneWidth, stopCount: len(w.Stops), cells: make(map[junctionCell][]*BusStop)}
	for _, stop := range w.Stops {
		if stop == nil || !stop.CenterBRT {
			continue
		}
		axis := UnitFromAngle(stop.Heading)
		along, across := BRTCrossingCoreHalf+BRTCrossingTaperLen, 4.5*laneWidth+30
		extent := Vec2{
			X: math.Abs(axis.X)*along + math.Abs(axis.Y)*across + 1,
			Y: math.Abs(axis.Y)*along + math.Abs(axis.X)*across + 1,
		}
		lo := junctionCellAt(stop.RoadCenter.Sub(extent))
		hi := junctionCellAt(stop.RoadCenter.Add(extent))
		for x := lo.x; x <= hi.x; x++ {
			for y := lo.y; y <= hi.y; y++ {
				cell := junctionCell{x, y}
				index.cells[cell] = append(index.cells[cell], stop)
			}
		}
	}
	w.brtFlares = index
}

// BRTPlatformOffset returns the lateral distance from RoadCenter to the center of the right-hand BRT platform.
func BRTPlatformOffset(lw float64) float64 {
	return lw + BRTStationMaxFlare*0.5
}

// BRTStationFlareAt returns the lateral road expansion offset (in pixels) for accommodating
// the widened BRT platform and flared general traffic lanes near a central BRT station.
func (w *World) BRTStationFlareAt(pos Vec2, lw float64) float64 {
	if w == nil || !w.CenterBRT {
		return 0
	}
	const maxFlare = BRTStationMaxFlare
	const coreHalf = BRTCrossingCoreHalf
	const taperLen = BRTCrossingTaperLen
	flare := 0.0
	candidates := w.Stops
	if index := w.brtFlares; index != nil && lw <= index.laneWidth && len(w.Stops) == index.stopCount {
		candidates = index.cells[junctionCellAt(pos)]
	}
	for _, s := range candidates {
		if s == nil || !s.CenterBRT {
			continue
		}
		u := UnitFromAngle(s.Heading)
		dLong := pos.Sub(s.RoadCenter).Dot(u)
		distLong := math.Abs(dLong)
		if distLong > coreHalf+taperLen {
			continue
		}
		r := Vec2{X: -u.Y, Y: u.X}
		dLat := pos.Sub(s.RoadCenter).Dot(r)
		if math.Abs(dLat) > 4.5*lw+30.0 {
			continue
		}
		cur := maxFlare
		if distLong > coreHalf {
			t := (coreHalf + taperLen - distLong) / taperLen
			cur = maxFlare * t * t * (3 - 2*t)
		}
		if cur > flare {
			flare = cur
		}
	}
	return flare
}

// Configure after all authored and generated stops exist, before bay clearance.
// Station IDs and route membership remain stable across the two map variants.
func (w *World) configureCenterBRT(lw float64) {
	w.brtFlares = nil
	w.CenterBRT = w.CenterBRT && w.HasBRTEligibleRoads()
	for i := range w.Roads {
		r := &w.Roads[i]
		r.MedianBusLane = w.CenterBRT && r.HasBusLane() && r.LaneCount() == 8
	}
	w.fitBuiltinBRTBridges(lw)
	for _, s := range w.Stops {
		ew := math.Abs(math.Cos(s.Heading)) > .5
		coord := s.RoadCenter.X
		if ew {
			coord = s.RoadCenter.Y
		}
		road, ok := w.RoadAt(coord, ew, s.RoadCenter)
		s.CenterBRT = ok && road.HasMedianBusLane()
		s.UpdateGeometry(lw)
	}
	w.ensureSymmetricCenterBRTStops(lw)
	w.syncPairedStopNames(lw)
	w.indexBRTFlares(lw)
}

func (w *World) ensureSymmetricCenterBRTStops(lw float64) {
	if w == nil || !w.CenterBRT {
		return
	}
	paired := make([]bool, len(w.Stops))
	clearOfWaterAndJunction := func(rc Vec2, h float64, lanes int) bool {
		right := Vec2{X: -math.Sin(h), Y: math.Cos(h)}
		pos := rc.Add(right.Mul(lw * 0.5))
		if w.InsideJunction(pos, lw, StandardStopLength/2+64) {
			return false
		}
		for _, v := range w.Waters {
			if math.Abs(rc.X-v.Center.X) < v.Width/2+80 && math.Abs(rc.Y-v.Center.Y) < v.Height/2+80 {
				return false
			}
		}
		for _, r := range w.Rivers {
			if math.Abs(rc.Y-r.Y) < r.Height/2+130 {
				return false
			}
		}
		return true
	}
	slideClearOfWater := func(rc Vec2, h float64, lanes int) Vec2 {
		if clearOfWaterAndJunction(rc, h, lanes) {
			return rc
		}
		u := UnitFromAngle(h)
		for offset := 260.0; offset <= 720.0; offset += 60.0 {
			for _, sign := range []float64{-1, 1} {
				cand := rc.Add(u.Mul(sign * offset))
				if clearOfWaterAndJunction(cand, h, lanes) {
					return cand
				}
			}
		}
		return rc
	}
	alignPair := func(a, b *BusStop, fallback Vec2, hasFallback bool) {
		primary, secondary := a, b
		if b.ID < a.ID {
			primary, secondary = b, a
		}
		targetRC := primary.RoadCenter
		if !clearOfWaterAndJunction(targetRC, primary.Heading, primary.RoadLanes) {
			if clearOfWaterAndJunction(secondary.RoadCenter, secondary.Heading, secondary.RoadLanes) {
				targetRC = secondary.RoadCenter
			} else if hasFallback && clearOfWaterAndJunction(fallback, primary.Heading, primary.RoadLanes) {
				targetRC = fallback
			} else {
				targetRC = slideClearOfWater(targetRC, primary.Heading, primary.RoadLanes)
			}
		}
		a.RoadCenter = targetRC
		b.RoadCenter = targetRC
		a.CenterBRT = true
		b.CenterBRT = true
		a.UpdateGeometry(lw)
		b.UpdateGeometry(lw)
	}

	for _, block := range w.EligibleBlockStops(lw) {
		h0 := NormalizeAngle(block.Heading)
		h1 := NormalizeAngle(block.Heading + math.Pi)
		ew := math.Abs(math.Cos(h0)) > .5
		var s0, s1 *BusStop
		for _, s := range w.Stops {
			if s == nil {
				continue
			}
			along, cross := s.RoadCenter.Y, s.RoadCenter.X
			coord := block.Center.X
			if ew {
				along, cross = s.RoadCenter.X, s.RoadCenter.Y
				coord = block.Center.Y
			}
			if math.Abs(cross-coord) < 1 && along > block.Start && along < block.End {
				if math.Cos(s.Heading-h0) > .99 && s0 == nil {
					s0 = s
				} else if math.Cos(s.Heading-h1) > .99 && s1 == nil {
					s1 = s
				}
			}
		}
		if s0 != nil && s1 != nil && (s0.CenterBRT || s1.CenterBRT) {
			if s0.ID >= 0 && s0.ID < len(paired) {
				paired[s0.ID] = true
			}
			if s1.ID >= 0 && s1.ID < len(paired) {
				paired[s1.ID] = true
			}
			alignPair(s0, s1, block.Center, true)
		}
	}

	baseCount := len(w.Stops)
	maxGap := 900.0 * w.GridScale()
	for i := 0; i < baseCount; i++ {
		s0 := w.Stops[i]
		if s0 == nil || !s0.CenterBRT || paired[i] {
			continue
		}
		ew := math.Abs(math.Cos(s0.Heading)) > .5
		var best *BusStop
		bestIdx := -1
		bestDist := maxGap
		for j := 0; j < baseCount; j++ {
			if i == j || paired[j] {
				continue
			}
			s1 := w.Stops[j]
			if s1 == nil || !s1.CenterBRT || math.Cos(s0.Heading-s1.Heading) > -0.98 {
				continue
			}
			cross := math.Abs(s0.RoadCenter.X - s1.RoadCenter.X)
			along0, along1 := s0.RoadCenter.Y, s1.RoadCenter.Y
			coord := s0.RoadCenter.X
			if ew {
				cross = math.Abs(s0.RoadCenter.Y - s1.RoadCenter.Y)
				along0, along1 = s0.RoadCenter.X, s1.RoadCenter.X
				coord = s0.RoadCenter.Y
			}
			dist := math.Abs(along0 - along1)
			if cross >= 2.0 || dist >= bestDist {
				continue
			}
			lo, hi := math.Min(along0, along1), math.Max(along0, along1)
			separated := false
			for _, inter := range w.Intersections {
				ic, ia := inter.X, inter.Y
				if ew {
					ic, ia = inter.Y, inter.X
				}
				if math.Abs(ic-coord) < 5 && ia > lo+1 && ia < hi-1 {
					separated = true
					break
				}
			}
			if !separated {
				best = s1
				bestIdx = j
				bestDist = dist
			}
		}
		if best != nil {
			paired[i] = true
			paired[bestIdx] = true
			alignPair(s0, best, s0.RoadCenter, false)
			continue
		}
		paired[i] = true
		s0.RoadCenter = slideClearOfWater(s0.RoadCenter, s0.Heading, s0.RoadLanes)
		s0.UpdateGeometry(lw)
		oppH := NormalizeAngle(s0.Heading + math.Pi)
		oppID := len(w.Stops)
		oppName := WithStopDirection(StripStopDirection(s0.Name), oppH)
		opp := NewBusStop(oppID, oppName, s0.RoadCenter, oppH, s0.RoadLanes, s0.AccentColor, lw, 0)
		opp.CenterBRT = true
		opp.Length = s0.Length
		opp.UpdateGeometry(lw)
		s0.Name = WithStopDirection(StripStopDirection(s0.Name), s0.Heading)
		s0.cachedInfoText = ""
		w.Stops = append(w.Stops, opp)
	}
}
