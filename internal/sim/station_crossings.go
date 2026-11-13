package sim

import (
	"math"
	"sort"
)

type StationCrossingKind uint8

const (
	StationFootbridge StationCrossingKind = iota
	StationUnderpass
	StationZebra
	StationSignalZebra
)

const StationCrossingStopOffset = 35.
const stationSignalCycle = 36.

type StationCrossingSignal uint8

const (
	StationSignalGreen StationCrossingSignal = iota
	StationSignalYellow
	StationSignalRed
)

type StationCrossing struct {
	ID          int
	Center      Vec2
	Heading     float64
	Lanes       int
	Kind        StationCrossingKind
	BusLane     bool
	phaseOffset float64
}

type stationCrossingIndex struct {
	all     []StationCrossing
	signals map[junctionCell][]int
}

func (w *World) StationCrossings() []StationCrossing {
	return w.stationCrossings.all
}

func (w *World) rebuildStationCrossings(laneWidth float64) {
	w.stationCrossings = stationCrossingIndex{}
	if !w.LargeGrid || w.Theme == MapThemeCustom {
		return
	}
	w.PrepareJunctionGeometry(laneWidth)
	// Use one crossing for nearby opposing platforms, even when one side
	// was shifted to clear the junction. Unpaired stops get their own.
	used := make([]bool, len(w.Stops))
	type placement struct {
		crossing StationCrossing
		order    uint32
	}
	var placements []placement
	var centers []Vec2
	for i, stop := range w.Stops {
		if used[i] || stop == nil || stop.CenterBRT {
			continue
		}
		ew := math.Abs(math.Cos(stop.Heading)) > .5
		axis := Vec2{Y: 1}
		coord := stop.RoadCenter.X
		if ew {
			axis, coord = Vec2{X: 1}, stop.RoadCenter.Y
		}
		road, ok := w.RoadAt(coord, ew, stop.RoadCenter)
		if !ok {
			continue
		}
		right := Vec2{X: -axis.Y, Y: axis.X}
		center := stop.RoadCenter
		mate, nearest := -1, 520.
		for j := i + 1; j < len(w.Stops); j++ {
			other := w.Stops[j]
			if used[j] || other == nil || other.CenterBRT || math.Cos(other.Heading-stop.Heading) > -.99 {
				continue
			}
			delta := other.RoadCenter.Sub(center)
			if d := math.Abs(delta.Dot(axis)); d < nearest && math.Abs(delta.Dot(right)) < 1 {
				mate, nearest = j, d
			}
		}
		if mate >= 0 {
			used[mate] = true
			center = center.Add(w.Stops[mate].RoadCenter).Mul(.5)
		}
		center = w.stationCrossingPosition(center, axis, road, laneWidth, centers)
		centers = append(centers, center)
		hash := uint32(stop.ID+1)*0x9e3779b1 ^ uint32(w.Theme+1)*0x85ebca77
		hash ^= hash >> 16
		hash *= 0x7feb352d
		hash ^= hash >> 15
		placements = append(placements, placement{
			crossing: StationCrossing{
				Center:      center,
				Heading:     math.Atan2(axis.Y, axis.X),
				Lanes:       road.LaneCount(),
				BusLane:     road.HasBusLane(),
				phaseOffset: float64(hash%3600) / 100,
			},
			order: hash,
		})
	}

	sort.SliceStable(placements, func(i, j int) bool { return placements[i].order < placements[j].order })
	index := stationCrossingIndex{all: make([]StationCrossing, 0, len(placements)), signals: make(map[junctionCell][]int)}
	for i, p := range placements {
		p.crossing.ID = i
		p.crossing.Kind = StationCrossingKind(i % 4)
		index.all = append(index.all, p.crossing)
		if p.crossing.Kind == StationSignalZebra {
			cell := junctionCellAt(p.crossing.Center)
			index.signals[cell] = append(index.signals[cell], i)
		}
	}
	w.stationCrossings = index
}

func (w *World) stationCrossingPosition(anchor, axis Vec2, road RoadSegment, lw float64, placed []Vec2) Vec2 {
	lo, hi := road.P1.Dot(axis), road.P2.Dot(axis)
	if lo > hi {
		lo, hi = hi, lo
	}
	for _, offset := range [...]float64{220, -220, 310, -310, 420, -420, 540, -540, 670, -670, 0} {
		p := anchor.Add(axis.Mul(offset))
		along := p.Dot(axis)
		if along < lo+70 || along > hi-70 || w.InsideJunction(p, lw, 100) || w.IsAtRoundabout(p) {
			continue
		}
		clear := true
		for _, rail := range w.Railways() {
			clear = clear && math.Abs(p.Y-rail.Y) >= rail.HalfWidth+RailGateMargin+220
		}
		if !clear {
			continue
		}
		for _, other := range placed {
			if p.Distance(other) < 125 {
				clear = false
				break
			}
		}
		for _, bridge := range w.Bridges {
			if math.Abs(math.Sin(bridge.Heading-math.Atan2(axis.Y, axis.X))) < .1 &&
				math.Abs(p.Sub(bridge.Pos).Dot(axis)) < bridge.Length/2+65 &&
				math.Abs(p.Sub(bridge.Pos).Dot(Vec2{X: -axis.Y, Y: axis.X})) < 4 {
				clear = false
				break
			}
		}
		for _, water := range w.Waters {
			if math.Abs(p.X-water.Center.X) < water.Width/2+42 && math.Abs(p.Y-water.Center.Y) < water.Height/2+42 {
				clear = false
				break
			}
		}
		for _, river := range w.Rivers {
			if math.Abs(p.Y-river.Y) < river.Height/2+42 {
				clear = false
				break
			}
		}
		if clear {
			return p
		}
	}
	return anchor
}

// StationSignalPhase shares a staggered green/yellow/red cycle across both
// vehicle approaches, leaving one simultaneous pedestrian crossing interval.
func (tls *TrafficLightSystem) StationSignalPhase(c StationCrossing) StationCrossingSignal {
	if tls != nil && tls.StationCrossingStates != nil {
		if st, ok := tls.StationCrossingStates[c.ID]; ok {
			return st.Phase
		}
	}
	elapsed := 0.
	if tls != nil {
		elapsed = tls.StationCrossingElapsed
	}
	switch phase := math.Mod(elapsed+c.phaseOffset, stationSignalCycle); {
	case phase < 24:
		return StationSignalGreen
	case phase < 28:
		return StationSignalYellow
	default:
		return StationSignalRed
	}
}

// StationCrossingSpeedLimit checks only nearby midblock signals. Keeping them
// separate from junctions preserves routing, turn selection and right-on-red rules.
func (w *World) StationCrossingSpeedLimit(tls *TrafficLightSystem, bumper Vec2, heading, speed, lw, braking float64) float64 {
	if w == nil || tls == nil || len(w.stationCrossings.signals) == 0 {
		return math.Inf(1)
	}
	forward := UnitFromAngle(intersectionApproachHeading(heading))
	right := Vec2{X: -forward.Y, Y: forward.X}
	lookAhead := math.Max(150, speed*speed/(2*braking)+80)
	extent := Vec2{X: math.Abs(forward.X)*(lookAhead+StationCrossingStopOffset) + math.Abs(right.X)*(4*lw+ShoulderWidth),
		Y: math.Abs(forward.Y)*(lookAhead+StationCrossingStopOffset) + math.Abs(right.Y)*(4*lw+ShoulderWidth)}
	lo, hi := junctionCellAt(bumper.Sub(extent)), junctionCellAt(bumper.Add(extent))
	limit := math.Inf(1)
	for x := lo.x; x <= hi.x; x++ {
		for y := lo.y; y <= hi.y; y++ {
			for _, idx := range w.stationCrossings.signals[junctionCell{x, y}] {
				crossing := w.stationCrossings.all[idx]
				if math.Abs(math.Cos(crossing.Heading-heading)) < .85 {
					continue
				}
				delta := crossing.Center.Sub(bumper)
				dist := delta.Dot(forward) - StationCrossingStopOffset
				if dist <= 0 || dist >= lookAhead || math.Abs(delta.Dot(right)) > float64(crossing.Lanes)*lw/2+ShoulderWidth {
					continue
				}
				phase := tls.StationSignalPhase(crossing)
				if phase == StationSignalGreen || phase == StationSignalYellow && dist < speed*speed/(2*braking)+12 {
					continue
				}
				limit = math.Min(limit, math.Sqrt(2*braking*math.Max(0, dist-12)))
			}
		}
	}
	return limit
}
