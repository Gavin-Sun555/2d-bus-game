package traffic

import (
	"math"

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
		baseObstacles = append(baseObstacles, autopilot.VehicleObstacle{
			Pos:     playerBus.Pos,
			Heading: playerBus.Heading,
			Speed:   playerBus.Speed,
			Length:  playerBus.Length,
			Width:   playerBus.Width,
		})
	}
	for _, c := range activeCars {
		baseObstacles = append(baseObstacles, autopilot.VehicleObstacle{
			Pos:     c.Pos,
			Heading: c.Heading,
			Speed:   c.Speed,
			Length:  c.Length,
			Width:   c.Width,
		})
	}

	abm.baseObstacles = baseObstacles
	for _, ab := range abm.Buses {
		obstacles := append(abm.obstacles[:0], baseObstacles...)
		for _, other := range abm.Buses {
			if other.ID == ab.ID {
				continue
			}
			obstacles = append(obstacles, autopilot.VehicleObstacle{
				Pos:     other.Bus.Pos,
				Heading: other.Bus.Heading,
				Speed:   other.Bus.Speed,
				Length:  other.Bus.Length,
				Width:   other.Bus.Width,
			})
		}

		abm.obstacles = obstacles
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
		if isDocked && ab.Bus.DoorsOpen {
			targetStopID := ab.Route.GetCurrentTargetStopID()
			if targetStopID >= 0 && targetStopID < len(world.Stops) {
				st := world.Stops[targetStopID]

				if ab.LastExchangedStopID != targetStopID {
					ab.LastExchangedStopID = targetStopID
					// 1. Alight passengers destined for this stop
					ab.Bus.RemovePassengersAlightingAt(targetStopID)
					if ab.Route.IsTerminusIndex(ab.Route.CurrentIndex) {
						ab.Bus.ClearPassengers()
					}
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
					actualBoarded := ab.Bus.BoardPassengers(take, candidates)
					st.WaitingPassengers -= actualBoarded
					ab.PassengersCarried += actualBoarded
				}
			}
		} else if !isDocked {
			ab.LastExchangedStopID = -1
		}

		// Boundary clearance check to keep AI bus safely on road
		bRes := world.CheckBoundaries(ab.Bus.Pos, ab.Bus.Width/2, laneWidth)
		if bRes.Zone == sim.ZoneHardBarrier {
			ab.Bus.Pos = ab.Bus.Pos.Add(bRes.Pushback.Mul(bRes.Penetration + 4.0))
			ab.Bus.Speed = 0
		}

		// Recover only after this bus has remained jammed for 1.5 full signal cycles.
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
				cosH := math.Cos(ab.Bus.Heading)
				sinH := math.Sin(ab.Bus.Heading)
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
						isBusDeadlocked = true
					}
				}
			}

			if !playerBlocking {
				for _, other := range abm.Buses {
					if other.ID == ab.ID || other.Bus == nil || math.Abs(other.Bus.Speed) >= 3.0 {
						continue
					}
					toOther := other.Bus.Pos.Sub(ab.Bus.Pos)
					cosH := math.Cos(ab.Bus.Heading)
					sinH := math.Sin(ab.Bus.Heading)
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
							isBusDeadlocked = true
							break
						}
					}
				}

				// ALSO check against motionless civilian AI cars
				if !isBusDeadlocked {
					for _, c := range activeCars {
						if c == nil || math.Abs(c.Speed) >= 3.0 {
							continue
						}
						dist := ab.Bus.Pos.Distance(c.Pos)
						if dist < (ab.Bus.Length+c.Length)*0.5+40.0 {
							toCar := c.Pos.Sub(ab.Bus.Pos)
							cosH := math.Cos(ab.Bus.Heading)
							sinH := math.Sin(ab.Bus.Heading)
							fwd := toCar.X*cosH + toCar.Y*sinH
							headDiff := math.Cos(c.Heading - ab.Bus.Heading)
							lat := math.Abs(-toCar.X*sinH + toCar.Y*cosH)
							// If queued behind car in same direction on open road, skip
							if !isNearInter && headDiff > 0.96 && lat < 24.0 && fwd > (ab.Bus.Length+c.Length)*0.5+6.0 && c.DeadlockTimer == 0 {
								continue
							}
							if fwd > -15.0 || dist < (ab.Bus.Length+c.Length)*0.5+15.0 {
								isBusDeadlocked = true
								break
							}
						}
					}
				}
			}

			ab.updateDeadlock(dt, isBusDeadlocked, tls, world)

		} else {
			ab.DeadlockTimer = 0
		}
	}

	// Collision separation between AI buses and active civilian cars, and among AI buses
	for i, ab := range abm.Buses {
		if ab.Bus == nil {
			continue
		}
		for _, car := range activeCars {
			sim.ResolveBusCarSeparation(ab.Bus, car)
		}
		for j := i + 1; j < len(abm.Buses); j++ {
			if abm.Buses[j].Bus != nil {
				sim.ResolveBusBusSeparation(ab.Bus, abm.Buses[j].Bus)
			}
		}
	}
}
