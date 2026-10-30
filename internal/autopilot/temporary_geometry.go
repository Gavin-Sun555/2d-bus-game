package autopilot

import "github.com/Gavin-Sun555/2d-bus-game/internal/sim"

const maxTemporaryBodies = 4096
const maxTemporaryObstacles = 2048

// Per-driver, simulation-thread-only scratch. Never publish these arrays as a
// vehicle corridor. Large custom inputs retain the original allocating fallback.
type temporaryGeometry struct {
	junctionMargin [][2]sim.OBB
	junctionActual [][2]sim.OBB
	admission      []sim.OBB
	ringBodies     []sim.OBB
	ringIndices    []int
}

func (ad *AutopilotDriver) geometryScratch() *temporaryGeometry {
	if ad.temporaryGeometry == nil {
		ad.temporaryGeometry = &temporaryGeometry{}
	}
	return ad.temporaryGeometry
}
