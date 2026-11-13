package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
)

func (ad *AutopilotDriver) publishJunctionCorridor(f autopilotFrame, dt float64) {
	if f.world == nil || len(ad.drivePath) < 2 || f.tls == nil {
		return
	}
	var center *sim.Vec2
	nearest := math.Inf(1)
	advance := math.Max(100, f.speedAbs*f.speedAbs/(2*130)+100)
	for _, it := range f.tls.Intersections {
		if f.world.IsAtRoundabout(it.Center) {
			continue
		}
		g := f.world.JunctionAt(it.Center, f.laneWidth)
		d := f.bus.Pos.Sub(it.Center)
		if math.Abs(d.X) > g.HalfX+f.bus.Length || math.Abs(d.Y) > g.HalfY+f.bus.Length {
			heading := approachRoadHeading(f.bus.Heading)
			lateral := math.Abs(-d.X*math.Sin(heading) + d.Y*math.Cos(heading))
			if lateral > it.ApproachRoadHalfWidth(heading, f.laneWidth)+f.bus.Length {
				continue
			}
		}

		if math.Abs(d.X) > g.HalfX+f.bus.Length+advance || math.Abs(d.Y) > g.HalfY+f.bus.Length+advance {
			continue
		}
		if dist := d.Length(); dist < nearest {
			c := it.Center
			center = &c
			nearest = dist
		}
	}
	if center == nil {
		return
	}
	if f.bus.JunctionCorridor == nil {
		f.bus.JunctionCorridor = &sim.JunctionCorridor{}
	}
	corridor := f.bus.JunctionCorridor
	if corridor.Remaining <= 0 || corridor.Center.Distance(*center) > 1 {
		corridor.EntryHeading = approachRoadHeading(f.bus.Heading)
		corridor.Entered = false
	}
	corridor.Center = *center
	corridor.CurbLeft = false
	corridor.UTurnReady = false
	u := sim.UnitFromAngle(corridor.EntryHeading)
	for _, it := range f.tls.Intersections {
		if it.Center.Distance(*center) < 1 {
			corridor.CurbLeft = ad.busLaneJunction != nil && ad.busLaneJunction.Distance(*center) < 1 &&
				it.ApproachLanes[it.GetApproachIndexForHeading(corridor.EntryHeading)] == 4
			gap := center.Sub(f.bus.Pos).Dot(u) - it.GetStopLineDistForHeading(corridor.EntryHeading, f.laneWidth) - f.bus.Length/2
			if turn := ad.uTurn; turn != nil && turn.center.Distance(*center) < 1 {
				if gap < 0 || gap > 32 {
					turn.entryWait = 0
				} else if f.speedAbs < 5 {
					turn.entryWait += dt
				}
				corridor.UTurnReady = turn.entryWait >= 4
			}
			if gap <= 0 {
				corridor.Entered = true
			}
			break
		}
	}
	corridor.Remaining = .25
	corridor.Bodies = corridor.Bodies[:0]
	corridor.Bodies = append(corridor.Bodies, sim.OBB{Center: f.bus.Pos, Heading: f.bus.Heading, HalfLen: f.bus.Length/2 + 4, HalfWid: f.bus.Width/2 + 3})
	g := f.world.JunctionAt(*center, f.laneWidth)
	start := ad.pointAt(f.s)
	tangent := ad.pointAt(f.s + 2).Sub(start).Normalize()
	origin := start.Add(sim.Vec2{X: -tangent.Y, Y: tangent.X}.Mul(ad.LaneOffset))
	errorPos := f.bus.Pos.Sub(origin)
	horizon := f.bus.Pos.Distance(*center) + 4*(g.HalfX+g.HalfY) + 200
	for d := 16.; d <= horizon; d += 16 {
		p := ad.pointAt(f.s + d)
		u := ad.pointAt(f.s + d + 2).Sub(p).Normalize()
		if u.Length() < .5 {
			break
		}
		error := errorPos
		if corridor.CurbLeft {
			// The approach taper removes this offset before the turning
			// arc. Carrying it through the box falsely reserves the inner lane.
			error = error.Mul(math.Max(0, 1-d/120))
		}
		p = p.Add(sim.Vec2{X: -u.Y, Y: u.X}.Mul(ad.LaneOffset)).Add(error)
		corridor.Bodies = append(corridor.Bodies, sim.OBB{Center: p, Heading: math.Atan2(u.Y, u.X), HalfLen: f.bus.Length/2 + 4, HalfWid: f.bus.Width/2 + 3})
		delta := p.Sub(*center)
		if (math.Abs(delta.X) > g.HalfX+f.bus.Length+60 || math.Abs(delta.Y) > g.HalfY+f.bus.Length+60) && delta.Dot(u) > 0 {
			break
		}
	}
	corridor.RebuildBounds()
	if corridor.UTurnReady {
		for _, ob := range f.obstacles {
			body := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length / 2, HalfWid: ob.Width / 2}
			if sim.ExitQueueBlocksEntry(corridor.Bodies, corridor.Center, g, f.bus.Length, f.bus.Width, body, ob.Speed) {
				corridor.UTurnReady = false
				break
			}
		}
	}
}
