package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestReplanOffRouteResetsBusPinnedAtDeadEndBarrierAcrossRoad(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -4000, Y: 0}, P2: sim.Vec2{X: 0, Y: 0}, Lanes: 8},
			{P1: sim.Vec2{X: 0, Y: -4000}, P2: sim.Vec2{X: 0, Y: 4000}, Lanes: 8},
		},
		Buildings: []sim.Building{
			{Pos: sim.Vec2{X: 280, Y: 130}, W: 60, H: 120},
		},
	}
	w.RebuildIntersections()

	from := sim.NewBusStop(0, "from", sim.Vec2{X: -1200, Y: 130}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -350, Y: -1200}, -math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)

	// Bus pushed back out of the dead-end east barrier onto x=204 still overlaps
	// the perpendicular NS carriageway (crossRoad=true) while its nose hits ZoneHardBarrier.
	pinned := sim.NewBus(204, 130, 0)
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.drivePath = []sim.Vec2{{X: -1200, Y: 130}, {X: -350, Y: 130}, {X: -350, Y: -1200}}
	ad.replanOffRoute(pinned, w, to, lw, 2.5)
	if !ad.OffRoadNetwork() {
		_, _, ok := roadAbeam(w, pinned.Pos, 0, lw)
		_, _, cross := roadAbeam(w, pinned.Pos, math.Pi/2, lw)
		nose := pinned.Pos.Add(sim.Vec2{X: pinned.Length/2 + 8})
		t.Fatalf("expected OffRoadNetwork=true at %+v (pathDist=%.1f roadAbeam=%v cross=%v hits=%v noseZone=%v)",
			pinned.Pos, ad.pathOffRouteDistance(pinned), ok, cross, pinned.HitsHardBarrier(w, lw), w.CheckBoundaries(nose, pinned.Width/2, lw).Zone)
	}

	// A stopped bus on the crossroad that is not pinned against any barrier
	// must not be treated as off the road network.
	onRoad := sim.NewBus(0, 130, 0)
	ad2 := NewAutopilotDriver()
	ad2.FleetBus = true
	ad2.drivePath = []sim.Vec2{from.Pos, {X: -350, Y: -1200}}
	ad2.replanOffRoute(onRoad, w, to, lw, 2.5)
	if ad2.OffRoadNetwork() {
		t.Fatalf("expected OffRoadNetwork=false for clear bus inside crossroad at %+v", onRoad.Pos)
	}
}

func TestReplanOffRouteResetsMovingBusAfterHardBarrierImpact(t *testing.T) {
	const lw = 52.
	w := &sim.World{
		Roads: []sim.RoadSegment{
			{P1: sim.Vec2{X: -4000, Y: 0}, P2: sim.Vec2{X: 0, Y: 0}, Lanes: 8},
			{P1: sim.Vec2{X: 0, Y: -4000}, P2: sim.Vec2{X: 0, Y: 4000}, Lanes: 8},
		},
	}
	w.RebuildIntersections()
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -350, Y: -1200}, -math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)

	bus := sim.NewBus(240, 130, 0)
	bus.Speed = 95
	ad := NewAutopilotDriver()
	ad.FleetBus = true
	ad.drivePath = []sim.Vec2{{X: -1200, Y: 130}, {X: -350, Y: 130}, {X: -350, Y: -1200}}
	ad.NoteHardBarrierImpact()
	ad.replanOffRoute(bus, w, to, lw, 2.5)
	if !ad.OffRoadNetwork() {
		t.Fatalf("expected OffRoadNetwork=true after barrier pushback bounce at %+v", bus.Pos)
	}
}
