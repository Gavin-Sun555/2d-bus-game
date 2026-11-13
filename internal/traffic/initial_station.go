package traffic

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"sort"
)

// PlaceInitialBusAtStation prefers the least occupied distinct route stops.
// A berth is committed only after full-body, traffic, junction and ring checks.
func PlaceInitialBusAtStation(bus *sim.Bus, route *sim.Route, w *sim.World, lw float64, occupied []*sim.Bus, cars []*sim.TrafficCar, preferred int) (int, bool) {
	if bus == nil || route == nil || w == nil {
		return 0, false
	}
	type choice struct{ index, load, distance int }
	choices := []choice{}
	seen := map[int]bool{}
	n := len(route.StopIDs)
	for index, id := range route.StopIDs {
		if id < 0 || id >= len(w.Stops) || seen[id] {
			continue
		}
		seen[id] = true
		st := w.Stops[id]
		load := 0
		u := sim.UnitFromAngle(st.Heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		for _, other := range occupied {
			if other == nil || other == bus {
				continue
			}
			d := other.Pos.Sub(st.Pos)
			if math.Cos(other.Heading-st.Heading) > .9 && math.Abs(d.Dot(u)) < st.Length/2+other.OverallLength() && math.Abs(d.Dot(r)) < st.Width/2+lw {
				load++
			}
		}
		distance := (index - preferred + n) % max(1, n)
		choices = append(choices, choice{index, load, distance})
	}
	sort.SliceStable(choices, func(i, j int) bool {
		if choices[i].load != choices[j].load {
			return choices[i].load < choices[j].load
		}
		return choices[i].distance < choices[j].distance
	})
	// Pass 0 uses the berths of every stop before pass 1 allows a short queue
	// behind any stop, so one crowded stop never fills while others are empty.
	// Queue slots pass the same junction, ring and traffic checks, which keeps
	// them out of the intersection or roundabout feeding the station.
	for pass := 0; pass < 2; pass++ {
		for _, c := range choices {
			st := w.Stops[route.StopIDs[c.index]]
			back := sim.UnitFromAngle(st.Heading).Mul(-1)
			positions := []sim.Vec2{st.FrontBerthPosition(), st.RearBerthPosition(), st.Pos}
			if pass == 1 {
				positions = positions[:0]
				for k := 1; k <= initialStationQueueSlots; k++ {
					positions = append(positions, st.RearBerthPosition().Add(back.Mul(float64(k)*(bus.OverallLength()+sim.StationBerthGap))))
				}
			}
			for _, pos := range positions {
				candidate := *bus
				candidate.Pos, candidate.Heading = pos, st.Heading
				candidate.Speed, candidate.SteeringAngle = 0, 0
				candidate.ResetArticulation()
				if !safeInitialSpawn(&candidate, w, lw, occupied, cars) {
					continue
				}
				*bus = candidate
				return c.index, true
			}
		}
	}
	return 0, false
}

const initialStationQueueSlots = 2

func SafeInitialBusPosition(bus *sim.Bus, w *sim.World, lw float64) bool {
	return bus != nil && w != nil && safeInitialSpawn(bus, w, lw, nil, nil)
}

// PlaceInitialBusOnRoute also permits a clear straight road when every berth
// is occupied. It uses the same exclusion zones as the fleet dispatcher.
func PlaceInitialBusOnRoute(bus *sim.Bus, driver *autopilot.AutopilotDriver, route *sim.Route, w *sim.World, lw float64, occupied []*sim.Bus, cars []*sim.TrafficCar) bool {
	return len(distributeInitialLine([]*AIBus{{Bus: bus, Driver: driver, Route: route}}, w, lw, occupied, cars)) == 1
}
