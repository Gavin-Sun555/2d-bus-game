package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"sort"
)

// Older saves can contain buses separated sideways by the collision solver at
// a shared terminus. Repair only conflicting stationary dispatch groups.
func (g *Game) repairSharedTerminusQueues() {
	type member struct {
		bus    *sim.Bus
		driver *autopilot.AutopilotDriver
		route  *sim.Route
		id     int
	}
	all := []member{{g.Bus, g.AutopilotDriver, g.Route, 0}}
	for _, b := range g.AIBusMgr.Buses {
		all = append(all, member{b.Bus, b.Driver, b.Route, b.ID})
	}
	groups := map[int][]member{}
	for _, m := range all {
		if m.route.CurrentIndex != 0 || len(m.route.StopIDs) == 0 || math.Abs(m.bus.Speed) >= 3 {
			continue
		}
		id := m.route.StopIDs[0]
		st := g.World.Stops[id]
		u := sim.UnitFromAngle(st.Heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		d := m.bus.Pos.Sub(st.Pos)
		if math.Cos(m.bus.Heading-st.Heading) > .98 && math.Abs(d.Dot(r)) < g.Settings.GetLaneWidth()+m.bus.Width && d.Dot(u) <= m.bus.Length/2 && d.Dot(u) > -float64(len(all)+1)*120 {
			groups[id] = append(groups[id], m)
		}
	}
	ids := make([]int, 0, len(groups))
	for id := range groups {
		ids = append(ids, id)
	}
	sort.Ints(ids)
	for _, id := range ids {
		group := groups[id]
		st := g.World.Stops[id]
		u := sim.UnitFromAngle(st.Heading)
		r := sim.Vec2{X: -u.Y, Y: u.X}
		conflict := false
		for i, a := range group {
			for _, b := range group[i+1:] {
				if math.Abs(a.bus.Pos.Sub(b.bus.Pos).Dot(u)) < (a.bus.Length+b.bus.Length)/2+23.9 {
					conflict = true
				}
			}
		}
		if !conflict {
			continue
		}
		sort.SliceStable(group, func(i, j int) bool {
			delta := group[i].bus.Pos.Sub(group[j].bus.Pos).Dot(u)
			if math.Abs(delta) > 1 {
				return delta > 0
			}
			return math.Abs(group[i].bus.Pos.Sub(st.Pos).Dot(r)) < math.Abs(group[j].bus.Pos.Sub(st.Pos).Dot(r))
		})
		members := map[*sim.Bus]bool{}
		for _, m := range group {
			members[m.bus] = true
		}
		others := []*sim.Bus{}
		for _, m := range all {
			if !members[m.bus] {
				others = append(others, m.bus)
			}
		}
		for _, m := range group {
			old := m.bus.Pos
			queued := sim.PlaceBusInStationQueue(m.bus, st, others, g.Settings.GetLaneWidth())
			if queued {
				m.bus.DoorsOpen = false
			}
			if m.bus.Pos != old {
				g.Operations.InterruptJourney(m.id)
			}
			m.driver.QueueAtStation(id)
			others = append(others, m.bus)
		}
	}
}

func (g *Game) queuePlayerAtStop(stop *sim.BusStop) {
	if g.Bus == nil || g.AIBusMgr == nil {
		return
	}
	others := make([]*sim.Bus, 0, len(g.AIBusMgr.Buses))
	for _, b := range g.AIBusMgr.Buses {
		others = append(others, b.Bus)
	}
	if sim.PlaceBusInStationQueue(g.Bus, stop, others, g.Settings.GetLaneWidth()) {
		g.Bus.DoorsOpen = false
		g.boardedThisStop = false
		if g.AutopilotDriver != nil {
			g.AutopilotDriver.QueueAtStation(stop.ID)
		}
	}
}
