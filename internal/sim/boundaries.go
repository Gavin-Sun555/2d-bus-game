package sim

import (
	"math"
)

// CheckBoundaries checks if a position is on the road, on the shoulder, or hitting a hard barrier
func (w *World) CheckBoundaries(pos Vec2, busRadius float64, laneWidth float64) BoundaryCheckResult {
	// 1. Check decorative Building collisions (Hard Barrier)
	for _, b := range w.Buildings {
		hw := b.W/2 + busRadius
		hh := b.H/2 + busRadius
		if math.Abs(pos.X-b.Pos.X) < hw && math.Abs(pos.Y-b.Pos.Y) < hh {
			// Collision with building
			dx := pos.X - b.Pos.X
			dy := pos.Y - b.Pos.Y
			var push Vec2
			if math.Abs(dx)/hw > math.Abs(dy)/hh {
				if dx > 0 {
					push = Vec2{X: 1, Y: 0}
				} else {
					push = Vec2{X: -1, Y: 0}
				}
			} else {
				if dy > 0 {
					push = Vec2{X: 0, Y: 1}
				} else {
					push = Vec2{X: 0, Y: -1}
				}
			}
			return BoundaryCheckResult{
				Zone:         ZoneHardBarrier,
				Pushback:     push,
				Penetration:  10.0,
				ObstacleName: "建筑墙体硬阻隔",
			}
		}
	}

	// 2. Check Outer Metropolitan Perimeter Wall (Hard Barrier)
	boundX := 3200.0 * MapGridScale
	boundY := 1480.0 * MapGridScale
	if pos.X > boundX {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: -1, Y: 0}, Penetration: pos.X - boundX, ObstacleName: "东侧边界隔离墙"}
	}
	if pos.X < -boundX {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 1, Y: 0}, Penetration: -boundX - pos.X, ObstacleName: "西侧边界隔离墙"}
	}
	if pos.Y > boundY {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 0, Y: -1}, Penetration: pos.Y - boundY, ObstacleName: "南侧边界隔离墙"}
	}
	if pos.Y < -boundY {
		return BoundaryCheckResult{Zone: ZoneHardBarrier, Pushback: Vec2{X: 0, Y: 1}, Penetration: -boundY - pos.Y, ObstacleName: "北侧边界隔离墙"}
	}

	// 2.5 Check Roundabout Road Corridors and Central Islands FIRST
	for _, rb := range w.Roundabouts {
		d := pos.Distance(rb.Center)
		if d < rb.InnerRadius-15.0 {
			dir := pos.Sub(rb.Center).Normalize()
			if dir.Length() == 0 {
				dir = Vec2{X: 1, Y: 0}
			}
			return BoundaryCheckResult{
				Zone:         ZoneHardBarrier,
				Pushback:     dir,
				Penetration:  (rb.InnerRadius - 15.0) - d + 5.0,
				ObstacleName: "环岛中央景观安全岛",
			}
		}
		if d <= rb.Radius+15.0 {
			return BoundaryCheckResult{Zone: ZoneRoad}
		}
	}

	// 2.7 Check Intersection Turning Clearances:
	// Provide generous turning clearance at all intersections of the active map so long buses can comfortably turn
	interTurnClearance := math.Max(260.0, laneWidth*4.2+55.0)
	for _, ipt := range w.Intersections {
		dx := pos.X - ipt.X
		dy := pos.Y - ipt.Y
		if dx*dx+dy*dy < interTurnClearance*interTurnClearance {
			return BoundaryCheckResult{Zone: ZoneRoad}
		}
	}

	// 2.8 Check Bus Stop Bays, Platforms, and Approach/Exit Transition Tapers:
	// Bus stops feature indented turnout bays with lead-in deceleration and lead-out merge tapers.
	// Ensure that vehicles pulling into or out of bus bays navigate smoothly without hitting phantom roadside barriers.
	for _, st := range w.Stops {
		cosH := math.Cos(st.Heading)
		sinH := math.Sin(st.Heading)
		dx := pos.X - st.Pos.X
		dy := pos.Y - st.Pos.Y

		// Longitudinal offset along direction of travel
		dLong := dx*cosH + dy*sinH
		// Lateral offset perpendicular to travel (positive towards sidewalk/shelter, negative towards road)
		dLat := -dx*sinH + dy*cosH

		// 1. Shelter physical structure collision check (located on sidewalk at dLat = 36.0, length 68, width 18)
		if dLat > st.Width*0.5 {
			shelterHW := 34.0 + 8.0 // half-length 34 + buffer
			if math.Abs(dLong) < shelterHW && dLat >= 24.0 && dLat <= 48.0 {
				pushX := sinH
				pushY := -cosH
				return BoundaryCheckResult{
					Zone:         ZoneHardBarrier,
					Pushback:     Vec2{X: pushX, Y: pushY},
					Penetration:  dLat - 24.0,
					ObstacleName: "公交候车亭立柱玻璃顶棚",
				}
			}
		}

		// 2. Bus Bay & Transit Corridor Envelope:
		// Bay box length is st.Length (140 px, dLong in [-70, +70]).
		// Approach queue corridor: dLong in [-1200, -70] (safely accommodates multi-bus fleet departure queue up to 8 buses).
		// Departure taper: dLong in [+70, +180] (110 px smooth exit corridor).
		if dLong >= -1200.0 && dLong <= 180.0 {
			curbDist := (float64(st.RoadLanes)/2.0)*laneWidth + st.Width*0.48
			// Only apply to the bus stop side of the road
			if dLat >= -curbDist-15.0 {
				mainRoadCurbLat := -(st.Width * 0.48)
				bayOuterCurbLat := st.Width*0.5 + 12.0

				var curbLat float64
				if dLong < -70.0 {
					// Approach queuing corridor: maintain full outer bay envelope clearance for queued AI fleet
					curbLat = bayOuterCurbLat
				} else if dLong > 70.0 {
					// Departure transition taper
					t := (180.0 - dLong) / 110.0
					curbLat = mainRoadCurbLat + t*(bayOuterCurbLat-mainRoadCurbLat)
				} else {
					// Directly inside bus stop bay box
					curbLat = bayOuterCurbLat
				}

				if dLat <= curbLat {
					return BoundaryCheckResult{Zone: ZoneRoad}
				}
			}
		}
	}

	// 3. Check Road and Shoulder distances across all straight, curved, and rotary road segments
	minDistBeyondCurb := math.MaxFloat64
	var closestPush Vec2

	for _, r := range w.Roads {
		roadW := r.GetWidth(laneWidth)
		roadHW := roadW / 2

		d, push := DistToSegment(pos, r.P1, r.P2)
		dBeyond := d - roadHW
		if dBeyond < minDistBeyondCurb {
			minDistBeyondCurb = dBeyond
			closestPush = push
		}
	}

	// Decision based on minimum distance beyond curb
	if minDistBeyondCurb <= 0 {
		// Fully on asphalt road
		return BoundaryCheckResult{Zone: ZoneRoad}
	} else if minDistBeyondCurb <= ShoulderWidth {
		// On roadside shoulder/sidewalk (can drive, but slows down)
		return BoundaryCheckResult{
			Zone:        ZoneShoulder,
			Penetration: minDistBeyondCurb,
		}
	}

	// Beyond shoulder -> Hard barrier impact!
	return BoundaryCheckResult{
		Zone:         ZoneHardBarrier,
		Pushback:     closestPush,
		Penetration:  minDistBeyondCurb - ShoulderWidth,
		ObstacleName: "路侧硬阻隔防撞护栏",
	}
}

// distToSegment calculates the minimum Euclidean distance from point p to line segment ab,
// returning the distance and a unit pushback vector pointing from p towards the nearest point on ab.
func DistToSegment(p, a, b Vec2) (float64, Vec2) {
	ab := b.Sub(a)
	l2 := ab.X*ab.X + ab.Y*ab.Y
	if l2 == 0 {
		d := p.Distance(a)
		if d == 0 {
			return 0, Vec2{X: 0, Y: 0}
		}
		return d, a.Sub(p).Mul(1.0 / d)
	}
	t := (p.Sub(a)).Dot(ab) / l2
	if t < 0 {
		t = 0
	} else if t > 1 {
		t = 1
	}
	proj := a.Add(ab.Mul(t))
	d := p.Distance(proj)
	if d == 0 {
		return 0, Vec2{X: 0, Y: 0}
	}
	push := proj.Sub(p).Mul(1.0 / d)
	return d, push
}
