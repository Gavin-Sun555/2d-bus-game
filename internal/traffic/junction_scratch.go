package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"slices"
)

type carPlanKey struct {
	world                                                  *sim.World
	revision                                               uint64
	center                                                 sim.Vec2
	heading, roadCenter, lateral, laneWidth, length, width float64
	lane                                                   int
	ew, major                                              bool
	lanes                                                  [4]int
	roads, dedicated                                       [4]bool
}
type carJunctionScratch struct {
	bodyKey                     carPlanKey
	bodyValid                   bool
	sampledTurn                 *sim.CarJunctionTurn
	sampledPoints               []sim.Vec2
	sampledBodies               []sim.OBB
	sampleOffsets               []int
	sampledLength, sampledWidth float64
	owner                       *sim.TrafficCar
	key                         carPlanKey
	valid                       bool
	plan                        *sim.CarJunctionTurn
	points                      []sim.Vec2
	corridor                    *sim.JunctionCorridor
}

func (f carFrame) junctionScratch() *carJunctionScratch {
	if f.scratch != nil && f.scratch.owner == f.car {
		return f.scratch
	}
	return nil
}

// Plans are immutable templates. Motion commits a copy and may crop its points;
// neither advancing a turn nor publishing a corridor mutates the shared template.
func planCarJunctionTurn(f carFrame, inter *sim.TrafficIntersection) *sim.CarJunctionTurn {
	scratch := f.junctionScratch()
	if scratch == nil {
		return buildCarJunctionTurn(f, inter)
	}
	key := carJunctionKey(f, inter)
	if !scratch.valid || scratch.key != key {
		scratch.plan = buildCarJunctionTurn(f, inter)
		scratch.key, scratch.valid = key, true
	}
	return scratch.plan
}

func carJunctionKey(f carFrame, inter *sim.TrafficIntersection) carPlanKey {
	car := f.car
	lateral := car.Pos.Sub(inter.Center).Dot(sim.Vec2{X: -math.Sin(car.Heading), Y: math.Cos(car.Heading)})
	key := carPlanKey{world: f.world, center: inter.Center, heading: car.Heading, roadCenter: car.RoadCenter, lateral: lateral, laneWidth: f.laneWidth, length: car.Length, width: car.Width, lane: car.LaneIndex, ew: car.IsEW, major: inter.IsMajor, lanes: inter.ApproachLanes, roads: inter.ApproachHasRoad, dedicated: inter.ApproachIs4Lane}
	if f.world != nil {
		key.revision = f.world.GeometryRevision()
	}
	return key
}

func (tm *TrafficManager) scratchForCar(car *sim.TrafficCar, index int) *carJunctionScratch {
	if tm.disableJunctionReuse || index < 0 || index >= len(tm.junctionScratch) {
		return nil
	}
	scratch := &tm.junctionScratch[index]
	if scratch.owner != car {
		*scratch = carJunctionScratch{owner: car}
	}
	return scratch
}

// Only the connector from the moving car to the next waypoint changes. The
// remainder is sampled once with exactly the same spacing and duplicate joins.
func (s *carJunctionScratch) appendBodies(dst []sim.OBB, points []sim.Vec2, turn *sim.CarJunctionTurn, length, width float64) []sim.OBB {
	if turn == nil || len(points) < 2 {
		return sim.AppendPathBodies(dst, points, length, width)
	}
	first := len(turn.Points) - (len(points) - 1)
	if first < 0 || !slices.Equal(points[1:], turn.Points[first:]) {
		return sim.AppendPathBodies(dst, points, length, width)
	}
	if s.sampledTurn != turn || s.sampledLength != length || s.sampledWidth != width || !slices.Equal(s.sampledPoints, turn.Points) {
		s.sampledTurn, s.sampledLength, s.sampledWidth = turn, length, width
		s.sampledPoints = append(s.sampledPoints[:0], turn.Points...)
		s.sampledBodies = s.sampledBodies[:0]
		if cap(s.sampleOffsets) < len(turn.Points) {
			s.sampleOffsets = make([]int, len(turn.Points))
		} else {
			s.sampleOffsets = s.sampleOffsets[:len(turn.Points)]
		}
		for i := 0; i < len(turn.Points)-1; i++ {
			s.sampleOffsets[i] = len(s.sampledBodies)
			s.sampledBodies = sim.AppendPathBodies(s.sampledBodies, turn.Points[i:i+2], length, width)
		}
		s.sampleOffsets[len(turn.Points)-1] = len(s.sampledBodies)
	}
	dst = sim.AppendPathBodies(dst, points[:2], length, width)
	return append(dst, s.sampledBodies[s.sampleOffsets[first]:]...)
}
