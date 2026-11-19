package autopilot

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

// A railway's Y coordinate spans the map, including grade-separated roads.
// Only the physical level crossings constrain a road recovery maneuver.
func busOverlapsLevelCrossing(bus *sim.Bus, world *sim.World) bool {
	for _, rail := range world.Railways() {
		for _, crossing := range rail.Crossings {
			if !crossing.Level {
				continue
			}
			zone := sim.OBB{Center: crossing.Center, HalfLen: crossing.RoadHalfWidth + sim.ShoulderWidth, HalfWid: rail.HalfWidth + sim.RailGateMargin}
			for _, part := range bus.BodyParts() {
				if part.HalfLen > 0 {
					if hit, _, _ := sim.CheckOBBCollision(part, zone); hit {
						return true
					}
				}
			}
		}
	}
	return false
}
