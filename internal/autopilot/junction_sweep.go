package autopilot

import (
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// A turning bus sweeps across more than its straight-road width. Check this
// before adjacent-lane exemptions can discard a car on the outgoing road.
func (ad *AutopilotDriver) junctionSweptSpeed(f autopilotFrame) float64 {
	if ad != nil {
		ad.junctionMergeSteerCapValid = false
	}
	if f.world == nil || len(f.obstacles) == 0 || len(ad.drivePath) < 2 || !f.world.InsideJunction(f.bus.Pos, f.laneWidth, f.bus.Length/2) {
		return math.Inf(1)
	}
	pointAt := func(distance float64) (sim.Vec2, float64) {
		p := ad.pointAt(f.s + distance)
		u := ad.pointAt(f.s + distance + 2).Sub(p).Normalize()
		p = p.Add(sim.Vec2{X: -u.Y, Y: u.X}.Mul(ad.LaneOffset))
		return p, math.Atan2(u.Y, u.X)
	}
	lookahead := math.Max(140, f.speedAbs*f.speedAbs/(2*180)+40)
	scratch := ad.geometryScratch()
	candidates := scratch.junctionCandidates[:0]
	for i, ob := range f.obstacles {
		radius := lookahead + math.Abs(ob.Speed)*1.5 + (f.bus.Length+ob.Length)/2
		delta := ob.Pos.Sub(f.bus.Pos)
		if delta.Dot(delta) > radius*radius {
			continue
		}
		candidates = append(candidates, i)
	}
	scratch.junctionCandidates = candidates
	if len(candidates) == 0 {
		return math.Inf(1)
	}

	base, angle := pointAt(0)
	errorPos := f.bus.Pos.Sub(base)
	errorHeading := sim.NormalizeAngle(f.bus.Heading - angle)
	r0 := sim.Vec2{X: -math.Sin(angle), Y: math.Cos(angle)}
	lat0 := errorPos.Dot(r0)
	approachConverging := ad != nil && ad.approachMerge != nil && ad.busTurnHoldDistance <= 0 &&
		math.Abs(lat0) > 4 && math.Abs(lat0) <= f.laneWidth
	headingConverging := math.Abs(lat0) > 6 && math.Abs(errorHeading) > .08 && lat0*errorHeading < 0
	converging := math.Abs(lat0) > f.laneWidth || approachConverging || headingConverging
	sign := 1.0
	if lat0 > 0 {
		sign = -1.0
	}
	var curvScale float64 = 1.0
	var straightFromPose bool
	var uCur sim.Vec2
	samplePose := func(distance float64) (sim.Vec2, float64) {
		if straightFromPose {
			return f.bus.Pos.Add(uCur.Mul(distance)), f.bus.Heading
		}
		if !converging {
			pos, heading := pointAt(distance)
			return pos.Add(errorPos), heading + errorHeading
		}
		curv := sign * curvScale * math.Tan(f.bus.MaxSteerAngle) / math.Max(1, f.bus.WheelBase)
		yaw := errorHeading
		along, lat := 0.0, lat0
		for d := 0.0; d < distance; d += 4 {
			step := math.Min(4, distance-d)
			if lat*sign < 0 {
				yaw = sim.Clamp(yaw+curv*step, -1.2, 1.2)
			} else {
				yaw *= math.Max(0, 1-step/40)
			}
			along += step * math.Cos(yaw)
			lat += step * math.Sin(yaw)
			if (lat0 < 0 && lat > 0) || (lat0 > 0 && lat < 0) {
				lat = 0
			}
		}
		pos, heading := pointAt(along)
		right := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
		return pos.Add(right.Mul(lat)), heading + yaw
	}
	closingSpeed := math.Max(25, f.speedAbs)
	var actualCache *sweepPoseCache
	active := len(candidates)
	// Distance determines the cap. Visit samples in that order so a closer
	// blocker can end the sweep before distant obstacles are predicted.
	for distance := 0.; distance <= lookahead; distance += 6 {
		arrival := distance / closingSpeed
		center, heading := samplePose(distance)
		sample := sweepSample{front: sim.OBB{Center: center, Heading: heading, HalfLen: f.bus.Length/2 + 6, HalfWid: f.bus.Width/2 + 4}}
		for slot, index := range candidates {
			if index < 0 {
				continue
			}
			ob := f.obstacles[index]
			if !sample.collides(f.bus, ob, math.Max(0, ob.Speed)*arrival, 4, 2) {
				continue
			}
			// A queued neighbor may overlap only the extra safety margin.
			// Waive that obstacle only after proving the complete actual sweep.
			if (distance <= 30 || converging) && (math.Abs(ob.Speed) < 1 || parallelLeftRearCar(f, ob)) {
				if actualCache == nil {
					if scratch.junctionCache == nil {
						scratch.junctionCache = &sweepPoseCache{}
					} else {
						clear(scratch.junctionCache.ready[:])
						scratch.junctionCache.overflowReady = 0
					}
					actualCache = scratch.junctionCache
					if lookahead >= 128*4 {
						actualCache.overflow = junctionPoseStorage(&scratch.junctionActual, lookahead, 4)
					}
				}
				rate := 0.
				if math.Abs(ob.Speed) >= 1 {
					rate = ob.Speed / closingSpeed
				}
				if junctionActualSweepClearAtRate(f.bus, ob, samplePose, sim.Vec2{}, 0, lookahead, rate, actualCache) ||
					junctionRearTrailerSeparates(f.bus, ob, samplePose, lookahead) {
					candidates[slot] = -1
					active--
					if active == 0 {
						return math.Inf(1)
					}
					continue
				}
				// The projected turn can sweep back into a stopped bus after
				// translating the path onto our actual pose. A compact bus
				// can first pull its rear clear at the checked straight crawl.
				if distance <= 30 && f.speedAbs <= 18 && !f.bus.IsArticulated() &&
					ob.Bus != nil && math.Abs(ob.Speed) < 1 && sim.Cos(ob.Heading-f.bus.Heading) > .95 &&
					ob.Pos.Sub(f.bus.Pos).Dot(sim.UnitFromAngle(f.bus.Heading)) < -f.bus.Length/2 {
					goal := ad.pointAt(f.s + 60)
					if ad.stationForwardEscapeClear(f, 0, 18, goal, 5) {
						ad.junctionMergeSteerCapValid = true
						ad.junctionMergeSteerCap = 0
						return 18
					}
				}
				if distance <= 30 && f.speedAbs <= 18 && f.bus.IsArticulated() && !straightFromPose &&
					math.Abs(ob.Speed) < 1 && math.Cos(ob.Heading-f.bus.Heading) > .95 &&
					ob.Pos.Sub(f.bus.Pos).Dot(sim.UnitFromAngle(f.bus.Heading)) < 0 &&
					ad.controlSweptSpeed(f, 0) >= 16 && ad.articulatedSweptSpeed(f, 0, true) >= 16 {
					ad.junctionMergeSteerCapValid = true
					ad.junctionMergeSteerCap = 0
					uCur = sim.UnitFromAngle(f.bus.Heading)
					straightFromPose = true
					candidates[slot] = -1
					active--
					if active == 0 {
						return 18
					}
					continue
				}
				if approachConverging && math.Abs(ob.Speed) < 1 && !f.bus.IsArticulated() {
					nCands := 2
					cands := [3]struct {
						capVal   float64
						scale    float64
						straight bool
					}{
						{sign * .25, .25, false},
						{sign * .10, .10, false},
					}
					if errorHeading*sign > .02 {
						uCur = sim.UnitFromAngle(f.bus.Heading)
						barrierOK := true
						for bd := 8.; bd <= 64.; bd += 8. {
							probe := f.bus.AtPose(f.bus.Pos.Add(uCur.Mul(bd)), f.bus.Heading)
							if probe.HitsHardBarrier(f.world, f.laneWidth) {
								barrierOK = false
								break
							}
						}
						if barrierOK {
							cands[2] = struct {
								capVal   float64
								scale    float64
								straight bool
							}{0, 0, true}
							nCands = 3
						}
					}
					for ci := 0; ci < nCands; ci++ {
						curvScale = cands[ci].scale
						straightFromPose = cands[ci].straight
						allClear := true
						for _, idx := range candidates {
							if idx < 0 {
								continue
							}
							other := f.obstacles[idx]
							otherRate := 0.
							if math.Abs(other.Speed) >= 1 {
								otherRate = other.Speed / closingSpeed
							}
							if !junctionActualSweepClearAtRate(f.bus, other, samplePose, sim.Vec2{}, 0, lookahead, otherRate) {
								allClear = false
								break
							}
						}
						if allClear {
							curvScale, straightFromPose = 1.0, false
							ad.junctionMergeSteerCapValid = true
							ad.junctionMergeSteerCap = cands[ci].capVal
							return math.Inf(1)
						}
					}
					curvScale, straightFromPose = 1.0, false
				}
			}
			return math.Sqrt(2 * 180 * math.Max(0, distance-30))
		}
	}
	return math.Inf(1)
}

// A car trailing in the inner lane can touch the padded trailer envelope.
// Keep the time-aligned sweep authoritative when the bus is already admitted.
func parallelLeftRearCar(f autopilotFrame, ob VehicleObstacle) bool {
	c := f.bus.JunctionCorridor
	if c == nil || !c.Entered || len(c.Bodies) < 2 || ob.Bus != nil || ob.Speed <= 0 || ob.Speed > f.speedAbs ||
		math.Cos(ob.Heading-c.EntryHeading) < .98 {
		return false
	}
	u := sim.UnitFromAngle(c.EntryHeading)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	if math.Abs(ob.Pos.Sub(c.Center).Dot(r)-.5*f.laneWidth) >= .25*f.laneWidth ||
		ob.Pos.Sub(f.bus.Pos).Dot(u) >= -f.bus.Length {
		return false
	}
	if c.CurbLeft {
		return true
	}
	last := c.Bodies[len(c.Bodies)-1]
	out := r.Mul(-1)
	return f.bus.Pos.Sub(c.Center).Dot(r) >= 1.3*f.laneWidth &&
		last.Center.Sub(c.Center).Dot(u) >= 1.3*f.laneWidth &&
		sim.UnitFromAngle(last.Heading).Dot(out) > .98
}

func junctionActualSweepClear(bus *sim.Bus, ob VehicleObstacle, pointAt func(float64) (sim.Vec2, float64), errorPos sim.Vec2, errorHeading, horizon float64, caches ...*sweepPoseCache) bool {
	return junctionActualSweepClearAtRate(bus, ob, pointAt, errorPos, errorHeading, horizon, 0, caches...)
}

func junctionActualSweepClearAtRate(bus *sim.Bus, ob VehicleObstacle, pointAt func(float64) (sim.Vec2, float64), errorPos sim.Vec2, errorHeading, horizon, rate float64, caches ...*sweepPoseCache) bool {
	index := 0
	for distance := 0.; distance <= horizon; distance += 4 {
		var sample *sweepSample
		if len(caches) > 0 {
			sample = caches[0].load(index)
		}
		var uncached sweepSample
		if sample == nil {
			pos, heading := pointAt(distance)
			body := sim.OBB{Center: pos.Add(errorPos), Heading: heading + errorHeading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
			uncached = sweepSample{front: body}
			if len(caches) > 0 {
				sample = caches[0].store(index, uncached)
			}
			if sample == nil {
				sample = &uncached
			}
		}
		if sample.collides(bus, ob, distance*rate, 0, 0) {
			return false
		}
		index++
	}
	return true
}

// Allow an articulated bus whose rear trailer slightly grazes a stopped
// follower behind the powered section to pull forward when the sweep clears it.
func junctionRearTrailerSeparates(bus *sim.Bus, ob VehicleObstacle, samplePose func(float64) (sim.Vec2, float64), horizon float64) bool {
	if bus == nil || !bus.IsArticulated() || math.Abs(ob.Speed) >= 1 {
		return false
	}
	if math.Cos(ob.Heading-bus.Heading) < 0.5 {
		return false
	}
	forward := sim.UnitFromAngle(bus.Heading)
	if ob.Pos.Sub(bus.Pos).Dot(forward) >= -bus.Length/2 {
		return false
	}
	if ob.Bus != nil && ob.Bus.Pos.Sub(bus.Pos).Dot(forward) >= -bus.Length/2 {
		return false
	}
	parts := predictedObstacleBodies(ob, 0)
	maxDepth := func(body sim.OBB) float64 {
		best := 0.0
		for _, part := range parts {
			if part.HalfLen <= 0 {
				continue
			}
			if hit, _, depth := sim.CheckOBBCollision(body, part); hit && depth > best {
				best = depth
			}
		}
		return best
	}
	initDepth := maxDepth(bus.TrailerBody())
	maxAllowed := math.Max(initDepth+1.0, 4.0)
	clearDist := -1.0
	for distance := 0.; distance <= horizon; distance += 4 {
		pos, heading := samplePose(distance)
		front := sim.OBB{Center: pos, Heading: heading, HalfLen: bus.Length / 2, HalfWid: bus.Width / 2}
		if maxDepth(front) > 0 {
			return false
		}
		tail := bus.TrailerBody()
		if distance > 0 {
			tail = bus.TrailerBodyAtPose(pos, heading)
		}
		depth := maxDepth(tail)
		if depth > maxAllowed {
			return false
		}
		if depth > 0 {
			clearDist = -1
		} else if clearDist < 0 {
			clearDist = distance
		}
	}
	return clearDist >= 0 && clearDist <= math.Min(horizon, 96)
}

// Per-scan validity; only junction scans may borrow persistent overflow storage.
type sweepPoseCache struct {
	bodies        [128]sweepSample
	ready         [128]bool
	overflow      []sweepSample
	overflowReady int
}

// Only junction scans borrow overflow storage. Reset validity on each scan,
// while retaining bounded capacity in the driver's simulation-thread scratch.
func junctionPoseStorage(storage *[]sweepSample, horizon, step float64) []sweepSample {
	if !(horizon >= 0 && horizon <= 16384) {
		return nil
	}
	count := min(4096, int(horizon/step)+1) - 128
	if count <= 0 {
		return nil
	}
	if cap(*storage) < count {
		// Grow geometrically: acceleration must not allocate a new large buffer
		// every time the speed adds one more prediction sample.
		capacity := max(count, min(4096-128, max(128, cap(*storage)*2)))
		*storage = make([]sweepSample, count, capacity)
	} else {
		*storage = (*storage)[:count]
	}
	return *storage
}
func (c *sweepPoseCache) load(index int) *sweepSample {
	if index < len(c.ready) {
		if c.ready[index] {
			return &c.bodies[index]
		}
		return nil
	}
	i := index - len(c.ready)
	if i < c.overflowReady {
		return &c.overflow[i]
	}
	return nil
}
func (c *sweepPoseCache) store(index int, sample sweepSample) *sweepSample {
	if index < len(c.ready) {
		c.bodies[index], c.ready[index] = sample, true
		return &c.bodies[index]
	}
	i := index - len(c.ready)
	if i < len(c.overflow) {
		c.overflow[i] = sample
		if i == c.overflowReady {
			c.overflowReady++
		}
		return &c.overflow[i]
	}
	return nil
}
