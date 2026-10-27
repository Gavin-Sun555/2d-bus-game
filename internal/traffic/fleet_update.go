package traffic

import (
	"math"
	"slices"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Update advances all AI route buses per frame
func (abm *AIBusManager) Update(
	dt float64,
	playerBus *sim.Bus,
	world *sim.World,
	tls *sim.TrafficLightSystem,
	tm *TrafficManager,
	laneWidth float64,
	densityLevel ...int,
) {
	if world == nil || len(abm.Buses) == 0 {
		return
	}
	world.BeginBoundaryQueries(laneWidth)
	defer world.EndBoundaryQueries()

	var activeCars []*sim.TrafficCar
	if tm != nil {
		density := 2
		if len(densityLevel) > 0 {
			density = densityLevel[0]
		}
		activeCars = tm.GetActiveCars(density)
	}

	// Reuse observations between ticks; controllers consume them synchronously.
	baseObstacles := abm.baseObstacles[:0]
	if cap(baseObstacles) < 1+len(activeCars) {
		baseObstacles = make([]autopilot.VehicleObstacle, 0, 1+len(activeCars))
	}
	if playerBus != nil {
		baseObstacles = append(baseObstacles, autopilot.ObserveBus(playerBus))
	}
	for _, c := range activeCars {
		baseObstacles = append(baseObstacles, autopilot.VehicleObstacle{
			Pos:              c.Pos,
			YieldingTo:       c.YieldingToBus,
			JunctionTurn:     c.JunctionTurn,
			JunctionCorridor: c.JunctionCorridor,
			Heading:          c.Heading,
			Speed:            c.Speed,
			Length:           c.Length,
			Width:            c.Width,
		})
	}

	abm.baseObstacles = baseObstacles
	hasRetired := false
	for _, ab := range abm.Buses {
		abm.updateMergeYield(ab.Bus, playerBus, laneWidth)
		obstacles := append(abm.obstacles[:0], baseObstacles...)
		for _, other := range abm.Buses {
			if other.ID == ab.ID {
				continue
			}
			obstacles = append(obstacles, autopilot.ObserveBus(other.Bus))
		}

		abm.obstacles = obstacles
		if ab.Driver != nil {
			ab.Driver.DisallowOncomingOvertake = true
		}
		throttle, brake, steer, handbrake, toggleDoors, isDocked := ab.Driver.ComputeControls(
			ab.Bus,
			ab.Route,
			world,
			tls,
			obstacles,
			laneWidth,
			dt,
		)

		ab.Bus.Update(dt, throttle, brake, steer, handbrake, toggleDoors)

		// Passenger exchange simulation when parked at station
		if isDocked && ab.Bus.DoorsOpen && !((ab.Bus.PendingModel != nil || ab.Retiring) && ab.Route.IsTerminusIndex(ab.Route.CurrentIndex) && math.Abs(ab.Bus.Speed) > 3) {
			targetStopID := ab.Route.GetCurrentTargetStopID()
			if targetStopID >= 0 && targetStopID < len(world.Stops) {
				st := world.Stops[targetStopID]

				if ab.LastExchangedStopID != targetStopID {
					ab.LastExchangedStopID = targetStopID
					// 1. Alight passengers destined for this stop
					offCount := ab.Bus.RemovePassengersAlightingAt(targetStopID)
					if ab.Route.IsTerminusIndex(ab.Route.CurrentIndex) {
						offCount += ab.Bus.ClearPassengers()
					}
					abm.Operations.Alight(ab.ID, targetStopID, offCount)
					if ab.Retiring && ab.Route.IsTerminusIndex(ab.Route.CurrentIndex) {
						ab.Route.AdvanceToNextStop()
						abm.Operations.Track(ab.ID, ab.Bus, ab.Route, dt)
						ab.retireReady = true
						ab.Bus.LaneMergeRequest = nil
						ab.Bus.StationMergeRequest = nil
						ab.Bus.JunctionCorridor = nil
						hasRetired = true
					}
				}

				if ab.Bus.PendingModel != nil && ab.Route.IsTerminusIndex(ab.Route.CurrentIndex) && !ab.retireReady {
					ab.Bus.TerminalReplacementHold = true
					site := sim.VehicleHandoverSite{Stop: st, World: world, LaneWidth: laneWidth}
					for _, ob := range obstacles {
						if ob.Bus != nil {
							for _, part := range ob.Bus.BodyParts() {
								if part.HalfLen > 0 {
									part.HalfLen += 2
									part.HalfWid += 2
									site.Obstacles = append(site.Obstacles, part)
								}
							}
						} else {
							site.Obstacles = append(site.Obstacles, sim.OBB{Center: ob.Pos, Heading: ob.Heading, HalfLen: ob.Length/2 + 2, HalfWid: ob.Width/2 + 2})
						}
					}
					if !ab.Bus.ReplaceModelAtTerminus(ab.Route, targetStopID, site) {
						if ab.Bus.PendingModel != nil && *ab.Bus.PendingModel == sim.BusArticulated && site.OtherPlatformBus() {
							ab.Bus.TerminalReplacementHold = false
						}
						ab.DeadlockTimer = 0
						continue
					}
				}
				if ab.retireReady {
					continue
				}
				// 2. Board waiting passengers with downstream destinations:
				// Because only 1 route is planned per map, passengers board any bus that arrives.
				// As long as there is available capacity on the bus, all waiting passengers board.
				boardSpace := ab.Bus.Capacity - len(ab.Bus.Passengers)
				if st.WaitingPassengers > 0 && boardSpace > 0 {
					candidates := ab.Route.GetUpcomingCandidateStopIDsForStop(targetStopID)
					if len(candidates) == 0 && len(ab.Route.StopIDs) > 0 {
						for _, id := range ab.Route.StopIDs {
							if id != targetStopID {
								candidates = append(candidates, id)
							}
						}
					}
					take := st.WaitingPassengers
					if take > boardSpace {
						take = boardSpace
					}
					actualBoarded, _ := abm.Operations.Board(ab.ID, ab.Bus, ab.Route, targetStopID, take, candidates)
					st.WaitingPassengers -= actualBoarded
					ab.PassengersCarried += actualBoarded
				}
			}
		} else if !isDocked {
			ab.LastExchangedStopID = -1
		}

		// Boundary clearance check to keep AI bus safely on road
		bRes := ab.Bus.CheckBoundaries(world, laneWidth)
		if ab.Bus.OnShoulder && (bRes.Zone != sim.ZoneShoulder || ab.Bus.IsOnStationAccessShoulder(world, laneWidth)) {
			ab.Bus.OnShoulder = false
		}
		if bRes.Zone == sim.ZoneHardBarrier {
			ab.Bus.Pos = ab.Bus.Pos.Add(bRes.Pushback.Mul(bRes.Penetration + 4.0))
			ab.Bus.Speed = 0
		}

		// Recover only after this bus has remained jammed for two full signal cycles.
		if math.Abs(ab.Bus.Speed) < 3.0 {
			// If bus is actively docked and exchanging passengers, it is operating normally, NOT deadlocked!
			if isDocked && ab.Bus.DoorsOpen {
				ab.DeadlockTimer = 0
				continue
			}

			isBusDeadlocked := false
			playerBlocking := false

			isNearInter := false
			if world != nil {
				for _, pt := range world.Intersections {
					if ab.Bus.Pos.Distance(pt) < 130.0 {
						isNearInter = true
						break
					}
				}
			}

			if playerBus != nil && ab.Bus.Pos.Distance(playerBus.Pos) < 180.0 {
				toBus := playerBus.Pos.Sub(ab.Bus.Pos)
				sinH, cosH := math.Sincos(ab.Bus.Heading)
				fwd := toBus.X*cosH + toBus.Y*sinH
				lat := math.Abs(-toBus.X*sinH + toBus.Y*cosH)
				headDiff := math.Cos(playerBus.Heading - ab.Bus.Heading)
				// If queued cleanly behind player bus heading in the same direction in the same lane:
				isCleanQueue := headDiff > 0.96 && lat < 24.0 && fwd > (ab.Bus.Length+playerBus.Length)*0.5+4.0
				if isCleanQueue {
					playerBlocking = true
				} else {
					// Wedged alongside, diagonal, or colliding with player bus:
					if fwd > -15.0 || ab.Bus.Pos.Distance(playerBus.Pos) < (ab.Bus.Length+playerBus.Length)*0.5+15.0 {
						if hit, _, _ := sim.CheckBusesCollision(ab.Bus, playerBus); hit {
							isBusDeadlocked = true
						}
					}
				}
			}

			if !playerBlocking {
				for _, other := range abm.Buses {
					if other.ID == ab.ID || other.Bus == nil || math.Abs(other.Bus.Speed) >= 3.0 {
						continue
					}
					toOther := other.Bus.Pos.Sub(ab.Bus.Pos)
					sinH, cosH := math.Sincos(ab.Bus.Heading)
					fwd := toOther.X*cosH + toOther.Y*sinH
					lat := math.Abs(-toOther.X*sinH + toOther.Y*cosH)

					// If cleanly queued behind the other AI bus in same lane, normal queue following:
					headDiff := math.Abs(sim.NormalizeAngle(ab.Bus.Heading - other.Bus.Heading))
					if headDiff < 0.5 && lat < 24.0 && fwd > (ab.Bus.Length+other.Bus.Length)*0.5+4.0 && other.DeadlockTimer == 0 {
						continue
					}

					dist := ab.Bus.Pos.Distance(other.Bus.Pos)
					if dist < (ab.Bus.Length+other.Bus.Length)*0.5+40.0 {
						if fwd > -15.0 || dist < (ab.Bus.Length+other.Bus.Length)*0.5+15.0 {
							if hit, _, _ := sim.CheckBusesCollision(ab.Bus, other.Bus); hit {
								isBusDeadlocked = true
								break
							}
						}
					}
				}

				// ALSO check against motionless civilian AI cars
				if !isBusDeadlocked {
					for _, c := range activeCars {
						if c == nil || math.Abs(c.Speed) >= 3.0 || !busCarDeadlockCorridor(ab.Bus, c) {
							continue
						}
						dist := ab.Bus.Pos.Distance(c.Pos)
						if dist < (ab.Bus.Length+c.Length)*0.5+40.0 {
							toCar := c.Pos.Sub(ab.Bus.Pos)
							sinH, cosH := math.Sincos(ab.Bus.Heading)
							fwd := toCar.X*cosH + toCar.Y*sinH
							headDiff := math.Cos(c.Heading - ab.Bus.Heading)
							// An aligned lead car may be offset while a merge is waiting.
							// Its lateral offset is not evidence of a physical wedge.
							// If queued behind car in same direction on open road, skip
							if !isNearInter && headDiff > 0.96 && fwd > (ab.Bus.Length+c.Length)*0.5+6.0 {
								continue
							}
							if carClearsStoppedBusAhead(c, ab.Bus) {
								continue
							}
							if fwd > -15.0 || dist < (ab.Bus.Length+c.Length)*0.5+15.0 {
								// Close parallel traffic can wait for a merge without
								// touching the bus. Only a physical wedge warrants
								// destructive recovery, never proximity alone.
								body := sim.OBB{Center: c.Pos, Heading: c.Heading, HalfLen: c.Length / 2, HalfWid: c.Width / 2}
								if hit, _, _ := ab.Bus.CollidesBody(body); hit {
									isBusDeadlocked = true
									break
								}
							}
						}
					}
				}
			}

			if ab.updateDeadlock(dt, isBusDeadlocked, tls, world) {
				others := make([]*sim.Bus, 0, len(abm.Buses)+1)
				if playerBus != nil {
					others = append(others, playerBus)
				}
				for _, other := range abm.Buses {
					if other != ab {
						others = append(others, other.Bus)
					}
				}
				sim.PlaceBusInStationQueue(ab.Bus, world.Stops[ab.Route.StopIDs[0]], others, laneWidth)
			}
		} else {
			ab.DeadlockTimer = 0
		}
	}

	if hasRetired {
		abm.Buses = slices.DeleteFunc(abm.Buses, func(b *AIBus) bool { return b.retireReady })
		abm.BusCount = len(abm.Buses)
	}
	// Collision separation between AI buses and active civilian cars, and among AI buses
	var separations *BusCarSeparations
	if tm != nil {
		separations = tm.BeginBusCarSeparations(activeCars)
	}
	for i, ab := range abm.Buses {
		if ab.Bus == nil {
			continue
		}
		if separations != nil {
			separations.Resolve(ab.Bus)
		} else {
			for _, car := range activeCars {
				sim.ResolveBusCarSeparation(ab.Bus, car)
			}
		}
		for j := i + 1; j < len(abm.Buses); j++ {
			if abm.Buses[j].Bus != nil {
				sim.ResolveBusBusSeparation(ab.Bus, abm.Buses[j].Bus)
			}
		}
	}
}
