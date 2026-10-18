package sim

import (
	"fmt"
	"math"
)

// CheckPlayerViolation checks if player's bus is running a red light.
// Core rule: As long as the bus crosses the stop line before yellow (during green) or during yellow,
// it has lawful right of way and will not be fined even if the light turns yellow or red while traversing.
func (tls *TrafficLightSystem) CheckPlayerViolation(bus *Bus, laneWidth ...float64) (violated bool, fineNotice string) {
	lw := 52.0
	if len(laneWidth) > 0 && laneWidth[0] > 0 {
		lw = laneWidth[0]
	}

	halfLen := bus.Length * 0.5
	cosH := math.Cos(bus.Heading)
	sinH := math.Sin(bus.Heading)

	frontPos := Vec2{
		X: bus.Pos.X + cosH*halfLen,
		Y: bus.Pos.Y + sinH*halfLen,
	}
	rearPos := Vec2{
		X: bus.Pos.X - cosH*halfLen,
		Y: bus.Pos.Y - sinH*halfLen,
	}

	leadingPos := frontPos
	trailingPos := rearPos
	if bus.Speed < -1.0 {
		leadingPos = rearPos
		trailingPos = frontPos
	}

	for _, inter := range tls.Intersections {
		ewStop, _, nsStop, _ := inter.GetApproachDimensions(lw)
		maxStop := math.Max(ewStop, nsStop)
		dist := bus.Pos.Distance(inter.Center)
		maxZone := maxStop + bus.Length + 50.0

		// If bus is outside this intersection zone, reset tracking for this intersection
		if dist > maxZone {
			if inter.PlayerInside || inter.PassageCompleted {
				inter.PlayerInside = false
				inter.PassageCompleted = false
				inter.PlayerCrossedBeforeYellow = false
				inter.PlayerCrossedOnRed = false
				inter.PlayerTurnedRight = false
				inter.PlayerTurnedLeft = false
				inter.FinedThisPassage = false
			}
			continue
		}

		// Effective moving heading of vehicle
		effHeading := bus.Heading
		if bus.Speed < -1.0 {
			effHeading = NormalizeAngle(bus.Heading + math.Pi)
		}

		// Direction vectors based on current heading (or entry heading if already inside)
		refHeading := effHeading
		if inter.PlayerInside {
			refHeading = inter.PlayerEntryHeading
		}

		stopLineDist := inter.GetStopLineDistForHeading(refHeading, lw)

		uFwd := Vec2{X: math.Cos(refHeading), Y: math.Sin(refHeading)}
		vRight := Vec2{X: -math.Sin(refHeading), Y: math.Cos(refHeading)}

		// Projected position along forward entry axis (origin = intersection center)
		rLead := leadingPos.Sub(inter.Center)
		sLead := rLead.X*uFwd.X + rLead.Y*uFwd.Y

		rTrail := trailingPos.Sub(inter.Center)
		sTrail := rTrail.X*uFwd.X + rTrail.Y*uFwd.Y

		// Motion vector towards intersection center:
		toCenter := inter.Center.Sub(bus.Pos)
		fwdDir := Vec2{X: math.Cos(effHeading), Y: math.Sin(effHeading)}
		movingTowardsCenter := toCenter.X*fwdDir.X + toCenter.Y*fwdDir.Y

		// Lateral offset from approach road corridor centerline (checked only before entry)
		latDist := math.Abs(rLead.X*vRight.X + rLead.Y*vRight.Y)
		roadHalfWidth := lw*2.5 + 35.0
		if !inter.PlayerInside && latDist > roadHalfWidth {
			continue
		}

		isPastStopLine := sLead >= -stopLineDist
		isPastExit := sTrail > stopLineDist

		// If the vehicle is before the stop line approaching the intersection,
		// reset PassageCompleted so it can be monitored for this new approach:
		if !isPastStopLine {
			inter.PassageCompleted = false
		}

		// Intersection entry tracking
		if !inter.PlayerInside && !inter.PassageCompleted {
			// A vehicle can only enter if it is moving towards the intersection center.
			// Departing/exiting vehicles (movingTowardsCenter < -5) cannot trigger re-entry!
			if movingTowardsCenter <= -5.0 && sLead > 0 {
				continue
			}

			if isPastStopLine && !isPastExit {
				// Crossing event: Leading bumper crosses stop line into intersection
				inter.PlayerInside = true
				inter.FinedThisPassage = false
				inter.PlayerTurnedRight = false
				inter.PlayerTurnedLeft = false

				inter.PlayerEntryHeading = effHeading
				inter.PlayerEntryPos = bus.Pos

				// Robust approach corridor determination:
				// If heading is clearly aligned (|cos| > 0.707 or |sin| > 0.707), use heading.
				// If vehicle is already steering into a turn at the stop line, use approach position!
				cosEntry := math.Abs(math.Cos(effHeading))
				sinEntry := math.Abs(math.Sin(effHeading))
				relPos := bus.Pos.Sub(inter.Center)
				isEW := true
				if cosEntry > 0.707 {
					isEW = true
				} else if sinEntry > 0.707 {
					isEW = false
				} else {
					isEW = math.Abs(relPos.X) >= math.Abs(relPos.Y)
				}
				inter.PlayerEntryIsEW = isEW

				// Determine entry lane:
				// Single-lane roads (2-lane roads with 1 lane in each direction) do NOT have dedicated lanes.
				// On multi-lane avenues:
				// 0 = inner (dedicated left-turn lane), 1 = outer (straight/right lane)
				hasDedicatedLanes := inter.ApproachHasDedicatedLanesForHeading(effHeading)
				inter.PlayerEntryLane = 1
				if hasDedicatedLanes {
					distFromCenter := 0.0
					if isEW {
						distFromCenter = math.Abs(bus.Pos.Y - inter.Center.Y)
					} else {
						distFromCenter = math.Abs(bus.Pos.X - inter.Center.X)
					}
					// Lane 0 is inner lane: distFromCenter in [0, 0.95*lw).
					// If vehicle is in Lane 1 or already steering/turning right, never mark as Lane 0!
					if distFromCenter < lw*0.95 && bus.SteeringAngle <= 0.01 {
						inter.PlayerEntryLane = 0
					}
				}

				// Check initial steering intent at entry
				if bus.SteeringAngle > 0.02 {
					inter.PlayerTurnedRight = true
				}
				if bus.SteeringAngle < -0.015 {
					inter.PlayerTurnedLeft = true
				}

				// Retrieve signals of the approach corridor the vehicle entered from:
				ewThru, ewL, nsThru, nsL := inter.GetDetailedSignals()
				thruSig := nsThru
				leftSig := nsL
				if inter.PlayerEntryIsEW {
					thruSig = ewThru
					leftSig = ewL
				}

				movementSig := thruSig
				if hasDedicatedLanes && (inter.PlayerEntryLane == 0 || inter.PlayerTurnedLeft) {
					movementSig = leftSig
				}

				// Lawful entry evaluation:
				// 1. Right turn is ALWAYS lawful and exempt from red light violations!
				// 2. Left turn when left signal is Green/Yellow gives lawful right-of-way!
				// 3. Green or Yellow entry gives lawful right-of-way!
				if inter.PlayerTurnedRight || movementSig == SignalGreen || movementSig == SignalYellow {
					inter.PlayerCrossedBeforeYellow = true
					inter.PlayerCrossedOnRed = false
				} else if hasDedicatedLanes && (leftSig == SignalGreen || leftSig == SignalYellow) && (inter.PlayerEntryLane == 0 || inter.PlayerTurnedLeft) {
					inter.PlayerCrossedBeforeYellow = true
					inter.PlayerCrossedOnRed = false
				} else if !hasDedicatedLanes && (thruSig == SignalGreen || thruSig == SignalYellow) {
					inter.PlayerCrossedBeforeYellow = true
					inter.PlayerCrossedOnRed = false
				} else {
					inter.PlayerCrossedBeforeYellow = false
					inter.PlayerCrossedOnRed = true
				}
			}
		}

		dHead := NormalizeAngle(effHeading - inter.PlayerEntryHeading)

		// Relative lateral movement towards the cross-streets:
		entryVRight := Vec2{X: -math.Sin(inter.PlayerEntryHeading), Y: math.Cos(inter.PlayerEntryHeading)}
		disp := bus.Pos.Sub(inter.PlayerEntryPos)
		latRight := disp.X*entryVRight.X + disp.Y*entryVRight.Y
		latLeft := -latRight

		// Retrieve signals of the approach corridor entered:
		ewThru, ewL, nsThru, nsL := inter.GetDetailedSignals()
		thruSig := nsThru
		leftSig := nsL
		if inter.PlayerEntryIsEW {
			thruSig = ewThru
			leftSig = ewL
		}

		// Right-Turn Detection:
		// Right turn is completely lawful and never fined!
		// If steering right, heading turned clockwise, or drifted right:
		if bus.SteeringAngle > 0.02 || dHead > 0.05 || (latRight > 5.0 && dHead > -0.05) {
			inter.PlayerTurnedRight = true
			inter.PlayerCrossedOnRed = false
			inter.PlayerCrossedBeforeYellow = true
		}

		// Left-Turn Detection (Dynamic & Lenient):
		// If steering left, heading turned counter-clockwise, or drifted left towards cross street:
		if bus.SteeringAngle < -0.015 || dHead < -0.05 || (latLeft > 5.0 && dHead < 0.05) {
			inter.PlayerTurnedLeft = true
			turnSig := thruSig
			if inter.ApproachHasDedicatedLanesForHeading(inter.PlayerEntryHeading) {
				turnSig = leftSig
			}
			// If the signal is Green or Yellow, the vehicle has lawful right-of-way!
			if turnSig == SignalGreen || turnSig == SignalYellow || (!inter.ApproachHasDedicatedLanesForHeading(inter.PlayerEntryHeading) && (thruSig == SignalGreen || thruSig == SignalYellow)) {
				inter.PlayerCrossedOnRed = false
				inter.PlayerCrossedBeforeYellow = true
			}
		}

		// 1. Red Light Violation Check (闯红灯抓拍 - 对应方向车道后轮过线判定):
		// Core rules (宽松友好设计):
		// - 右转永远合法，不抓拍 (PlayerTurnedRight).
		// - 绿灯/黄灯进入路口合法通行，绝不抓拍 (PlayerCrossedBeforeYellow).
		// - 绿灯左右转弯在路口清空或出路口时绝不误判为闯红灯 (PassageCompleted).
		// - 只有当对应进道方向为红灯、非合法转向、且后轮越过停止线 (sTrail >= -stopLineDist) 时才抓拍。
		if inter.PlayerInside && inter.PlayerCrossedOnRed && !inter.PlayerTurnedRight && !inter.PassageCompleted && !inter.FinedThisPassage {
			hasDedicatedLanes := inter.ApproachHasDedicatedLanesForHeading(inter.PlayerEntryHeading)
			reqSig := thruSig
			if (inter.PlayerEntryLane == 0 || inter.PlayerTurnedLeft) && hasDedicatedLanes {
				reqSig = leftSig
			}

			// Left turn immunity if left signal is currently Green or Yellow:
			if inter.PlayerTurnedLeft && ((hasDedicatedLanes && (leftSig == SignalGreen || leftSig == SignalYellow)) || (!hasDedicatedLanes && (thruSig == SignalGreen || thruSig == SignalYellow))) {
				inter.PlayerCrossedOnRed = false
				inter.PlayerCrossedBeforeYellow = true
			} else if reqSig == SignalGreen || reqSig == SignalYellow || (!hasDedicatedLanes && (thruSig == SignalGreen || thruSig == SignalYellow)) {
				// If signal turned Green/Yellow before rear wheels crossed, forgive!
				inter.PlayerCrossedOnRed = false
				inter.PlayerCrossedBeforeYellow = true
			} else if reqSig == SignalRed && math.Abs(bus.Speed) >= 10.0 && inter.Cooldown <= 0 {
				// 后轮过线判定:
				if sTrail >= -stopLineDist {
					inter.FinedThisPassage = true
					inter.Cooldown = 6.0
					return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 闯红灯（后轮越过停止线）！", inter.ID+1)
				}
			}
		}

		// 2. Lane Discipline Violation Check (不按导向车道行驶抓拍 - 到达对侧正式判定):
		// Core rules:
		// - 只有具备多车道导向体系（专门左转车道）的大道入口才存在导向车道违章！
		// - 单车道道路（2车道道路，各方向仅1车道）为混行车道，直行、左转、右转皆完全合法，绝不进行导向车道判定！
		// - 右转永远合法：在单车道道路或右侧车道右转，绝不得判定为违规！
		hasDedicatedLanes := inter.ApproachHasDedicatedLanesForHeading(inter.PlayerEntryHeading)
		if inter.PlayerInside && hasDedicatedLanes && !inter.FinedThisPassage && inter.Cooldown <= 0 {
			vLeft := Vec2{X: math.Sin(inter.PlayerEntryHeading), Y: -math.Cos(inter.PlayerEntryHeading)}
			sLeft := rLead.X*vLeft.X + rLead.Y*vLeft.Y
			sRight := rLead.X*vRight.X + rLead.Y*vRight.Y

			// Determine if vehicle has reached an exit on the opposite side:
			reachedStraightExit := sLead >= stopLineDist || isPastExit
			reachedLeftExit := sLeft >= maxStop-15.0 && dHead < -0.35
			reachedRightExit := sRight >= maxStop-15.0 && dHead > 0.35

			if reachedStraightExit || reachedLeftExit || reachedRightExit {
				// Vehicle has reached the opposite side! Now evaluate whether the vehicle obeyed the lane arrow:
				// Case A: Left-Turn Lane (0) must turn left!
				if inter.PlayerEntryLane == 0 {
					// Only fine if vehicle truly went straight without turning left
					if reachedStraightExit && math.Abs(dHead) <= 0.35 && !inter.PlayerTurnedLeft {
						inter.FinedThisPassage = true
						inter.Cooldown = 6.0
						inter.PassageCompleted = true
						inter.PlayerInside = false
						return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 不按导向车道行驶（左转车道违规直行）！", inter.ID+1)
					} else if (reachedRightExit || dHead > 0.40) && !inter.PlayerTurnedLeft {
						inter.FinedThisPassage = true
						inter.Cooldown = 6.0
						inter.PassageCompleted = true
						inter.PlayerInside = false
						return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 不按导向车道行驶（左转车道违规右转）！", inter.ID+1)
					}
				}

				// Case B: Straight/Right Lane (1) must NOT turn left!
				// 宽松判定：如果左转灯为绿灯/黄灯，或者只是借道转弯/压线的大巴车，不误判为违规左转。
				// 只有在左转为红灯（禁止左转）时从直行车道左转，才判定违规！
				if inter.PlayerEntryLane == 1 {
					if (reachedLeftExit || dHead < -0.35) && leftSig != SignalGreen && leftSig != SignalYellow {
						inter.FinedThisPassage = true
						inter.Cooldown = 6.0
						inter.PassageCompleted = true
						inter.PlayerInside = false
						return true, fmt.Sprintf("[交通违章抓拍] 车辆在交叉口 #%d 不按导向车道行驶（直行车道违规左转）！", inter.ID+1)
					}
				}

				// If legally exited, mark passage completed and clear inside tracking
				inter.PassageCompleted = true
				inter.PlayerInside = false
				inter.PlayerCrossedBeforeYellow = true
				inter.PlayerCrossedOnRed = false
				inter.PlayerTurnedRight = false
				inter.PlayerTurnedLeft = false
				inter.FinedThisPassage = false
			}
		}

		// Exit clearing if vehicle cleared the intersection (for both major and minor intersections)
		if inter.PlayerInside && isPastExit {
			inter.PassageCompleted = true
			inter.PlayerInside = false
			inter.PlayerCrossedBeforeYellow = true
			inter.PlayerCrossedOnRed = false
			inter.PlayerTurnedRight = false
			inter.PlayerTurnedLeft = false
			inter.FinedThisPassage = false
		}
	}

	return false, ""
}

// CheckStopForVehicle checks whether an approaching vehicle should stop for a red/yellow light,
// and returns signed distance to the stop line along the vehicle's heading.
// If vehicle is inside the intersection or turning right, it will NOT stop.
func (tls *TrafficLightSystem) CheckStopForVehicle(pos Vec2, heading float64, lookAhead float64, laneWidth float64, turnOptions ...bool) (shouldStop bool, distToStopLine float64) {
	wantLeft := len(turnOptions) > 0 && turnOptions[0]
	wantRight := len(turnOptions) > 1 && turnOptions[1]
	allowYellow := len(turnOptions) > 2 && turnOptions[2]

	// Mutual exclusion: if vehicle intends to turn left, it cannot turn right
	if wantLeft {
		wantRight = false
	}

	cosH := math.Cos(heading)
	sinH := math.Sin(heading)

	for _, inter := range tls.Intersections {
		stopLineDist := inter.GetStopLineDistForHeading(heading, laneWidth)
		toInter := inter.Center.Sub(pos)
		forwardDist := toInter.X*cosH + toInter.Y*sinH
		lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

		// Must be on the approaching corridor heading towards the intersection (accommodates 2, 4, and 6 lane approaches)
		if forwardDist > -stopLineDist && forwardDist < stopLineDist+lookAhead+60 && lateralDist < laneWidth*3.6 {
			distToLine := forwardDist - stopLineDist

			// If already at or past the stop line into the intersection (distToLine <= 0):
			// NEVER stop! Must proceed and clear intersection to avoid blocking zebra crossings!
			if distToLine <= 0 {
				continue
			}

			// Determine which signal to respect (Left-turn vs Through):
			hasDedicatedLanes := inter.ApproachHasDedicatedLanesForHeading(heading)
			inLeftTurnLane := false
			if hasDedicatedLanes {
				if math.Abs(cosH) > math.Abs(sinH) {
					// EW avenue
					dy := math.Abs(pos.Y - inter.Center.Y)
					if dy > 5.0 && dy < laneWidth*0.95 {
						inLeftTurnLane = true
					}
				} else {
					// NS avenue
					dx := math.Abs(pos.X - inter.Center.X)
					if dx > 5.0 && dx < laneWidth*0.95 {
						inLeftTurnLane = true
					}
				}
			}

			// If vehicle is physically positioned in the dedicated left-turn lane, it MUST turn left and CANNOT turn right!
			effectiveWantLeft := wantLeft || (hasDedicatedLanes && inLeftTurnLane)
			effectiveWantRight := wantRight && !effectiveWantLeft && !inLeftTurnLane

			// Vehicles making a lawful right turn (from outer lane) may legally turn on red (红灯允许右转)
			if effectiveWantRight {
				return false, 999.0
			}

			sig := inter.GetSignalForHeading(heading, effectiveWantLeft)

			// If approaching before the stop line within lookAhead
			if distToLine > 0 && distToLine < lookAhead {
				if sig == SignalRed || (!allowYellow && sig == SignalYellow) {
					return true, distToLine
				}
			}
		}
	}
	return false, 999.0
}

// ShouldVehicleStop checks if an approaching vehicle should stop before the intersection stop line
func (tls *TrafficLightSystem) ShouldVehicleStop(pos Vec2, heading float64, lookAhead float64) bool {
	stop, _ := tls.CheckStopForVehicle(pos, heading, lookAhead, 42.0)
	return stop
}

// GetApproachingSignalInfo returns signal info for a vehicle approaching an intersection within lookAhead
func (tls *TrafficLightSystem) GetApproachingSignalInfo(pos Vec2, heading float64, laneWidth float64) (hasInter bool, thruSig LightSignal, leftSig LightSignal, isMajor bool, remainingSec float64, distToLine float64) {
	cosH := math.Cos(heading)
	sinH := math.Sin(heading)

	for _, inter := range tls.Intersections {
		stopLineDist := inter.GetStopLineDistForHeading(heading, laneWidth)
		toInter := inter.Center.Sub(pos)
		forwardDist := toInter.X*cosH + toInter.Y*sinH
		lateralDist := math.Abs(-toInter.X*sinH + toInter.Y*cosH)

		if forwardDist > 0 && forwardDist < stopLineDist+260 && lateralDist < laneWidth*2.5 {
			dist := forwardDist - stopLineDist
			th, lf := inter.GetSignalForApproach(heading)
			rem := 0.0
			switch inter.Phase {
			case PhaseEWGreen, PhaseNSGreen:
				rem = math.Max(0, inter.GreenDuration-inter.Timer)
			case PhaseEWYellow, PhaseNSYellow:
				rem = math.Max(0, inter.YellowDuration-inter.Timer)
			case PhaseEWLeftGreen, PhaseNSLeftGreen:
				rem = math.Max(0, inter.LeftGreenDuration-inter.Timer)
			case PhaseEWLeftYellow, PhaseNSLeftYellow:
				rem = math.Max(0, inter.YellowDuration-inter.Timer)
			}
			hasDedicated := inter.ApproachHasDedicatedLanesForHeading(heading)
			return true, th, lf, hasDedicated, rem, dist
		}
	}
	return false, SignalGreen, SignalRed, false, 0, 999.0
}

// CheckRoadCameraViolation checks if the player bus is driving the wrong way (逆行) near straight road surveillance cameras
func (tls *TrafficLightSystem) CheckRoadCameraViolation(bus *Bus) (violated bool, fineNotice string) {
	for _, cam := range tls.RoadCameras {
		if cam.Cooldown > 0 {
			continue
		}
		dist := bus.Pos.Distance(cam.Pos)
		if dist > 110.0 {
			continue
		}
		// Vehicle must be moving at driving speed
		if math.Abs(bus.Speed) < 15.0 {
			continue
		}

		effHeading := bus.Heading
		if bus.Speed < -1.0 {
			effHeading = NormalizeAngle(bus.Heading + math.Pi)
		}

		isWrongWay := false
		if cam.IsEW {
			// East-West road: centerline at Y = cam.RoadCoord
			// South of centerline (Y > cam.RoadCoord + 18.0): legal flow is East (heading ~ 0, cos > 0)
			// North of centerline (Y < cam.RoadCoord - 18.0): legal flow is West (heading ~ pi, cos < 0)
			if bus.Pos.Y > cam.RoadCoord+18.0 {
				if math.Cos(effHeading) < -0.35 {
					isWrongWay = true
				}
			} else if bus.Pos.Y < cam.RoadCoord-18.0 {
				if math.Cos(effHeading) > 0.35 {
					isWrongWay = true
				}
			}
		} else {
			// North-South road: centerline at X = cam.RoadCoord
			// West of centerline (X < cam.RoadCoord - 18.0): legal flow is South (heading ~ pi/2, sin > 0)
			// East of centerline (X > cam.RoadCoord + 18.0): legal flow is North (heading ~ -pi/2, sin < 0)
			if bus.Pos.X < cam.RoadCoord-18.0 {
				if math.Sin(effHeading) < -0.35 {
					isWrongWay = true
				}
			} else if bus.Pos.X > cam.RoadCoord+18.0 {
				if math.Sin(effHeading) > 0.35 {
					isWrongWay = true
				}
			}
		}

		if isWrongWay {
			cam.Cooldown = 8.0
			cam.FlashTimer = 0.5
			return true, fmt.Sprintf("[交通违章抓拍] 车辆在【%s】监控路段逆向行驶（逆行抓拍）！", cam.RoadName)
		}
	}
	return false, ""
}
