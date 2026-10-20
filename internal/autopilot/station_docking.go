package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type stationDockPlan struct {
	stopID int
	rank   int // bay, curb lane, then successively farther lanes
	target sim.Vec2
	points []sim.Vec2
}

type stationDockInputs struct {
	throttle, brake, steer float64
	toggleDoors, isDocked  bool
}

// Plan in stop coordinates so lane preference is independent of overtaking
// offsets and of the tapered route's changing distance from the road center.
func (ad *AutopilotDriver) stationDockControls(f autopilotFrame) (stationDockInputs, bool) {
	stop := f.targetStop
	if !ad.BerserkMode || ad.CanSkipTargetStop || ad.State == AutopilotStateDeparting || stop == nil {
		return stationDockInputs{}, false
	}
	nearbyTraffic := false
	for _, ob := range f.obstacles {
		if ob.Pos.Distance(stop.Pos) < 300 {
			nearbyTraffic = true
			break
		}
	}
	if ad.stationDock == nil && math.Abs(ad.LaneOffset) < 1 && !nearbyTraffic {
		return stationDockInputs{}, false
	}
	forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	distance := stop.Pos.Sub(f.bus.Pos).Dot(forward)
	if (ad.stationDock == nil || ad.stationDock.stopID != stop.ID) && (f.remaining > 400 || distance > 360 || distance < -20 || math.Cos(f.bus.Heading-stop.Heading) < 0.85) {
		return stationDockInputs{}, false
	}
	right := sim.Vec2{X: -forward.Y, Y: forward.X}
	plan := ad.stationDock
	// Reconsider nearer lanes while there is enough runway. Once braking beside
	// the stop, hold the chosen lane through the door cycle instead of swerving.
	if plan == nil || plan.stopID != stop.ID || distance > 140 {
		candidates := []sim.Vec2{stop.Pos}
		count := stop.RoadLanes / 2
		if count < 1 {
			count = 1
		}
		for lane := 0; lane < count; lane++ {
			candidates = append(candidates, stop.RoadCenter.Add(right.Mul((float64(count-lane)-0.5)*f.laneWidth)))
		}
		if stop.RoadLanes <= 2 {
			candidates = append(candidates, stop.RoadCenter.Sub(right.Mul(f.laneWidth/2)))
		}
		best := plan
		if best != nil && !stationDockCurveClear(f.bus, remainingStationCurve(best.points, entryPosition(best.points, f.bus.Pos, forward)), f.obstacles) {
			best = nil
		}
		for rank, target := range candidates {
			if best != nil && best.stopID == stop.ID && rank >= best.rank {
				break
			}
			lateral := math.Abs(target.Sub(f.bus.Pos).Dot(right))
			if lateral > math.Max(10, (distance-60)*0.65) {
				continue
			}
			points := stationDockCurve(f.bus, target, forward)
			if !stationDockCurveClear(f.bus, points, f.obstacles) {
				continue
			}
			best = &stationDockPlan{stopID: stop.ID, rank: rank, target: target, points: points}
			break
		}
		plan = best
		ad.stationDock = plan
	}
	if plan == nil {
		return stationDockInputs{brake: 1}, true
	}
	ad.LaneOffset, ad.TargetLaneOffset = 0, 0
	ad.OvertakeState = OvertakeNone
	ad.LaneOffsetProgress = 1
	ad.lastTargetLaneOffset = 0
	remaining := math.Max(0, plan.target.Sub(f.bus.Pos).Dot(forward))
	lateral := math.Abs(plan.target.Sub(f.bus.Pos).Dot(right))
	aligned := math.Abs(sim.NormalizeAngle(f.bus.Heading-stop.Heading)) < 0.15
	if remaining < 10 && lateral < 10 && aligned && f.speedAbs < 12 {
		ad.beginDocking(stop)
		return stationDockInputs{brake: 1, toggleDoors: !f.bus.DoorsOpen && f.bus.DoorProgress <= 0.05, isDocked: true}, true
	}
	if f.bus.DoorsOpen || f.bus.DoorProgress > 0.05 {
		return stationDockInputs{brake: 1, toggleDoors: f.bus.DoorsOpen && f.speedAbs < 15}, true
	}
	// Look forward beyond the endpoint to finish parallel and never chase the
	// station's bay across several lanes once the selected lane has been reached.
	s := entryPosition(plan.points, f.bus.Pos, forward)
	lookahead := sim.Clamp(f.speedAbs*0.35+30, 35, 65)
	target := entryPointAt(plan.points, forward, s+lookahead)
	if s+lookahead >= stationCurveLength(plan.points) {
		target = plan.target.Add(forward.Mul(lookahead))
	}
	delta := target.Sub(f.bus.Pos)
	alpha := sim.NormalizeAngle(math.Atan2(delta.Y, delta.X) - f.bus.Heading)
	steer := sim.Clamp(math.Atan2(2*f.bus.WheelBase*math.Sin(alpha), math.Max(20, delta.Length()))/f.bus.MaxSteerAngle, -1, 1)
	speed := math.Min(100, math.Sqrt(2*130*math.Max(0, remaining-3)))
	speed = ad.trafficLightSpeed(f, speed)
	// Recheck the swept path when traffic changes after selecting a lane.
	if !stationDockCurveClear(f.bus, remainingStationCurve(plan.points, s), f.obstacles) {
		speed = 0
	}
	throttle, brake := ad.longitudinalControls(f.bus.Speed, speed)
	ad.LastRemaining = remaining
	ad.LastTargetSpeed = speed
	ad.LastPursuitPt = target
	return stationDockInputs{throttle: throttle, brake: brake, steer: steer}, true
}

func stationDockCurve(bus *sim.Bus, target, forward sim.Vec2) []sim.Vec2 {
	distance := math.Max(0, target.Sub(bus.Pos).Dot(forward))
	straight := math.Min(120, distance*0.4)
	end := target.Sub(forward.Mul(straight))
	handle := math.Min(100, distance*0.35)
	heading := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
	a, b := bus.Pos.Add(heading.Mul(handle)), end.Sub(forward.Mul(handle))
	points := make([]sim.Vec2, 0, 34)
	for i := 0; i <= 32; i++ {
		t := float64(i) / 32
		u := 1 - t
		points = append(points, bus.Pos.Mul(u*u*u).Add(a.Mul(3*u*u*t)).Add(b.Mul(3*u*t*t)).Add(end.Mul(t*t*t)))
	}
	for along := 5.0; along < straight; along += 5 {
		points = append(points, end.Add(forward.Mul(along)))
	}
	return append(points, target)
}

func stationCurveLength(points []sim.Vec2) float64 {
	length := 0.0
	for i := 1; i < len(points); i++ {
		length += points[i].Distance(points[i-1])
	}
	return length
}

func stationDockCurveClear(bus *sim.Bus, points []sim.Vec2, obstacles []VehicleObstacle) bool {
	travel := 0.0
	for i := 0; i < len(points); i++ {
		delta := sim.Vec2{X: math.Cos(bus.Heading), Y: math.Sin(bus.Heading)}
		if i+1 < len(points) {
			delta = points[i+1].Sub(points[i])
		} else if i > 0 {
			delta = points[i].Sub(points[i-1])
		}
		heading := math.Atan2(delta.Y, delta.X)
		if delta.Length() < 0.01 {
			heading = bus.Heading
		}
		body := sim.OBB{Center: points[i], Heading: heading, HalfLen: bus.Length/2 + 5, HalfWid: bus.Width/2 + 2}
		for _, ob := range obstacles {
			other := sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length/2 + 4, HalfWid: ob.Width/2 + 2}
			// Compare vehicles at the same future instant. Testing every curve
			// point against a moving car's current position freezes its old
			// footprint across the entire approach and creates a false stop.
			time := travel / math.Max(40, math.Abs(bus.Speed))
			other.Center = other.Center.Add(sim.Vec2{X: math.Cos(ob.Heading), Y: math.Sin(ob.Heading)}.Mul(ob.Speed * time))
			if hit, _, _ := sim.CheckOBBCollision(body, other); hit {
				return false
			}
		}
		travel += delta.Length()
	}
	return true
}

func (ad *AutopilotDriver) beginDocking(stop *sim.BusStop) {
	ad.State = AutopilotStateDocked
	ad.DockedStopID = stop.ID
	ad.DwellTimer = ad.DwellTotal
	if ad.BerserkMode {
		ad.DwellTimer = 1
	}
	ad.DwellFinished = false
}

func remainingStationCurve(points []sim.Vec2, s float64) []sim.Vec2 {
	remaining := []sim.Vec2{}
	total := 0.0
	for i := 1; i < len(points); i++ {
		total += points[i].Distance(points[i-1])
		if total > s+2 {
			remaining = append(remaining, points[i])
		}
	}
	return remaining
}
