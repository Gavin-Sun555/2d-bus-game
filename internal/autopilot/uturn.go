package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

type uTurnManeuver struct {
	center, forward sim.Vec2
	entered         bool
}

// Identify the incoming and reversed outgoing tangents through the same mouth.
// Looking only for a far/side exit loses a U-turn on a wide road.
func (ad *AutopilotDriver) uTurnMovement(center, forward sim.Vec2, stop float64) (float64, bool) {
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	entered, incoming := false, false
	for i := 1; i < len(ad.drivePath); i++ {
		p := ad.drivePath[i].Sub(center)
		fwd, lat := p.Dot(forward), p.Dot(right)
		tangent := ad.drivePath[i].Sub(ad.drivePath[i-1]).Normalize().Dot(forward)
		if fwd >= -stop && fwd <= stop && math.Abs(lat) <= stop {
			entered = true
			if tangent > .95 && lat > 0 {
				incoming = true
			}
		}
		if entered && incoming && tangent < -.9 && lat < 0 && fwd <= stop {
			return ad.cumLen[i], true
		}
		if entered && (fwd > stop || math.Abs(lat) > stop) {
			return 0, false
		}
	}
	return 0, false
}

func (ad *AutopilotDriver) updateUTurn(bus *sim.Bus, w *sim.World, lw float64) {
	if ad.uTurn != nil {
		g := w.JunctionAt(ad.uTurn.center, lw)
		half := math.Abs(ad.uTurn.forward.X)*g.HalfX + math.Abs(ad.uTurn.forward.Y)*g.HalfY
		along := bus.Pos.Sub(ad.uTurn.center).Dot(ad.uTurn.forward)
		if ad.uTurn.entered && along < -half-bus.Length/2-80 {
			ad.uTurn = nil
			return
		}
		ad.uTurn.entered = ad.uTurn.entered || along+bus.Length/2 >= -half-sim.JunctionStopInset
		return
	}
	for _, center := range w.Intersections {
		if w.IsAtRoundabout(center) {
			continue
		}
		g := w.JunctionAt(center, lw)
		delta := bus.Pos.Sub(center)
		if math.Abs(delta.X) > g.HalfX+bus.Length+180 || math.Abs(delta.Y) > g.HalfY+bus.Length+180 {
			continue
		}
		for _, f := range []sim.Vec2{{X: 1}, {Y: 1}, {X: -1}, {Y: -1}} {
			half := math.Abs(f.X)*g.HalfX + math.Abs(f.Y)*g.HalfY
			end, ok := ad.uTurnMovement(center, f, half+sim.JunctionStopInset)
			if !ok || ad.LastPathS > end+150 {
				continue
			}
			ad.uTurn = &uTurnManeuver{center: center, forward: f, entered: delta.Dot(f)+bus.Length/2 >= -half-sim.JunctionStopInset}
			return
		}
	}
}

// Keep the junction commitment until the rear clears for signals and lane policy,
// but release the turning speed cap as soon as the bus has straightened on the
// outgoing carriageway. Recheck every frame in case steering is needed again.
func (turn *uTurnManeuver) needsTurnSpeed(bus *sim.Bus) bool {
	if turn == nil || !turn.entered {
		return false
	}
	direction := sim.UnitFromAngle(bus.Heading)
	if direction.Dot(turn.forward) > -.995 || math.Abs(bus.SteeringAngle) > .06 {
		return true
	}
	right := sim.Vec2{X: -turn.forward.Y, Y: turn.forward.X}
	lateralExtent := math.Abs(direction.Dot(right))*bus.Length/2 + math.Abs(direction.Dot(turn.forward))*bus.Width/2
	return bus.Pos.Sub(turn.center).Dot(right)+lateralExtent > 0
}
