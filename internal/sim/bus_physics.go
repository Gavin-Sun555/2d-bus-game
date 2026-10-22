package sim

import (
	"math"
)

// ShiftTo safely shifts to the specified gear with transmission speed protection
func (b *Bus) ShiftTo(target Gear) bool {
	// Safety interlock: Prevent shifting into reverse while moving forward fast (> 20 px/s)
	if target == GearReverse && b.Speed > 20.0 {
		return false
	}
	// Prevent shifting into forward while reversing fast (< -20 px/s)
	if (target == GearDrive || target >= Gear1) && b.Speed < -20.0 {
		return false
	}
	b.Gear = target
	return true
}

// ShiftUp shifts up one gear level
func (b *Bus) ShiftUp() bool {
	if b.TransmissionMode == TransmissionAuto {
		switch b.Gear {
		case GearReverse:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(GearDrive)
		case GearDrive:
			return false
		default:
			return b.ShiftTo(GearDrive)
		}
	} else {
		// Manual mode progression: R -> N -> 1 -> 2 -> 3 -> 4 -> 5
		switch b.Gear {
		case GearReverse:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(Gear1)
		case Gear1:
			return b.ShiftTo(Gear2)
		case Gear2:
			return b.ShiftTo(Gear3)
		case Gear3:
			return b.ShiftTo(Gear4)
		case Gear4:
			return b.ShiftTo(Gear5)
		case Gear5:
			return false
		}
	}
	return false
}

// ShiftDown shifts down one gear level
func (b *Bus) ShiftDown() bool {
	if b.TransmissionMode == TransmissionAuto {
		switch b.Gear {
		case GearDrive:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(GearReverse)
		case GearReverse:
			return false
		default:
			return b.ShiftTo(GearNeutral)
		}
	} else {
		// Manual mode progression: 5 -> 4 -> 3 -> 2 -> 1 -> N -> R
		switch b.Gear {
		case Gear5:
			return b.ShiftTo(Gear4)
		case Gear4:
			return b.ShiftTo(Gear3)
		case Gear3:
			return b.ShiftTo(Gear2)
		case Gear2:
			return b.ShiftTo(Gear1)
		case Gear1:
			return b.ShiftTo(GearNeutral)
		case GearNeutral:
			return b.ShiftTo(GearReverse)
		case GearReverse:
			return false
		}
	}
	return false
}

// GetGearLimits returns top speed and acceleration multiplier for current gear
func (b *Bus) GetGearLimits() (maxSpeed float64, accelFactor float64) {
	if b.TransmissionMode == TransmissionAuto {
		switch b.Gear {
		case GearDrive:
			return b.MaxSpeedFwd, 1.0
		case GearReverse:
			return b.MaxSpeedRev, 0.75
		default:
			return 0, 0
		}
	}

	// Manual mode specs (reserved functionality)
	switch b.Gear {
	case GearReverse:
		return b.MaxSpeedRev, 0.75
	case GearNeutral:
		return 0, 0
	case Gear1:
		return 75.0, 1.30
	case Gear2:
		return 140.0, 1.10
	case Gear3:
		return 210.0, 0.95
	case Gear4:
		return 275.0, 0.80
	case Gear5:
		return b.MaxSpeedFwd, 0.65
	default:
		return b.MaxSpeedFwd, 1.0
	}
}

// Update handles vehicle kinematics per frame (dt is seconds)
func (b *Bus) Update(dt float64, throttle float64, brake float64, steer float64, handbrake bool, toggleDoors bool) {
	if b.JunctionCorridor != nil {
		b.JunctionCorridor.Remaining -= dt
		if b.JunctionCorridor.Remaining <= 0 {
			b.JunctionCorridor = nil
		}
	}
	if b.RoundaboutEntry != nil {
		b.RoundaboutEntry.Remaining -= dt
		if b.RoundaboutEntry.Remaining <= 0 {
			b.RoundaboutEntry = nil
		}
	}
	if b.LaneMergeRequest != nil {
		b.LaneMergeRequest.Remaining -= dt
		if b.LaneMergeRequest.Remaining <= 0 {
			b.LaneMergeRequest = nil
		}
	}

	if b.StationMergeRequest != nil {
		b.StationMergeRequest.Remaining -= dt
		if b.StationMergeRequest.Remaining <= 0 {
			b.StationMergeRequest = nil
		}
	}
	if b.CrashCooldown > 0 {
		b.CrashCooldown -= dt
	}
	// 1. Doors control
	if toggleDoors {
		// Only allow door opening if bus is stopped or crawling into bay (< 15 px/s)
		if math.Abs(b.Speed) < 15.0 {
			b.DoorsOpen = !b.DoorsOpen
			if b.DoorsOpen {
				b.Speed = 0 // Full stop when doors open
			}
		}
	}

	if b.DoorsOpen {
		b.DoorProgress = math.Min(1.0, b.DoorProgress+dt*3.0)
		// Cannot move while doors are open
		b.Speed = 0
		return
	} else {
		b.DoorProgress = math.Max(0.0, b.DoorProgress-dt*3.0)
	}

	// 2. Steering input with smooth interpolation & auto-centering
	targetSteer := steer * b.MaxSteerAngle
	if steer != 0 {
		if b.SteeringAngle < targetSteer {
			b.SteeringAngle = math.Min(targetSteer, b.SteeringAngle+b.SteerTurnSpeed*dt)
		} else if b.SteeringAngle > targetSteer {
			b.SteeringAngle = math.Max(targetSteer, b.SteeringAngle-b.SteerTurnSpeed*dt)
		}
	} else {
		// Auto return to center
		if b.SteeringAngle > 0 {
			b.SteeringAngle = math.Max(0, b.SteeringAngle-b.SteerReturnSpd*dt)
		} else if b.SteeringAngle < 0 {
			b.SteeringAngle = math.Min(0, b.SteeringAngle+b.SteerReturnSpd*dt)
		}
	}

	// 3. Powertrain: Separated Foot Brake, Handbrake, Accelerator Throttle, and Gears
	b.IsBraking = false
	b.IsHandbraking = handbrake
	b.IsReversing = (b.Gear == GearReverse || b.Speed < -1.0)

	// A. Emergency Handbrake
	if handbrake {
		b.IsBraking = true
		if b.Speed > 0 {
			b.Speed = math.Max(0, b.Speed-b.HandbrakeRate*dt)
		} else if b.Speed < 0 {
			b.Speed = math.Min(0, b.Speed+b.HandbrakeRate*dt)
		}
	} else {
		// B. Foot Brake Pedal (S / Down): strictly decelerates vehicle towards 0 and holds at 0
		// NEVER causes the vehicle to reverse!
		if brake > 0 {
			b.IsBraking = true
			if b.Speed > 0 {
				b.Speed = math.Max(0, b.Speed-b.BrakeRate*brake*dt)
			} else if b.Speed < 0 {
				b.Speed = math.Min(0, b.Speed+b.BrakeRate*brake*dt)
			}
			// When speed reaches 0, foot brake holds the vehicle stationary!
		}

		// C. Accelerator Throttle (W / Up): applies engine torque according to current Gear
		if throttle > 0 && brake == 0 {
			maxSpd, accelRatio := b.GetGearLimits()

			switch b.Gear {
			case GearDrive, Gear1, Gear2, Gear3, Gear4, Gear5:
				// Forward gear: accelerates vehicle forward
				if b.Speed < 0 {
					// Counteract backward roll first
					b.Speed = math.Min(0, b.Speed+b.BrakeRate*dt)
					b.IsBraking = true
				} else {
					b.Speed = math.Min(maxSpd, b.Speed+b.AccelRate*accelRatio*throttle*dt)
				}
			case GearReverse:
				// Reverse gear (R): accelerates vehicle backward
				if b.Speed > 0 {
					// Counteract forward roll first
					b.Speed = math.Max(0, b.Speed-b.BrakeRate*dt)
					b.IsBraking = true
				} else {
					b.Speed = math.Max(-maxSpd, b.Speed-b.AccelRate*accelRatio*throttle*dt)
				}
			case GearNeutral:
				// Neutral gear (N): transmission disengaged, no wheel driving torque
			}
		}

		// D. Natural rolling resistance and coasting deceleration
		if (throttle == 0 || b.Gear == GearNeutral) && brake == 0 {
			if b.Speed > 0 {
				b.Speed = math.Max(0, b.Speed-b.FrictionRate*dt)
			} else if b.Speed < 0 {
				b.Speed = math.Min(0, b.Speed+b.FrictionRate*dt)
			}
		}
	}

	// Extra drag if driving onto roadside shoulder / verge (can drive, but slows down)
	if b.OnShoulder {
		if math.Abs(b.Speed) > 75 {
			b.Speed *= math.Pow(0.91, dt*60)
		}
	}

	// 4. Bicycle Kinematic Model Update
	if math.Abs(b.Speed) > 0.01 {
		// Angular velocity w = (v / L) * tan(delta)
		angularVelocity := (b.Speed / b.WheelBase) * math.Tan(b.SteeringAngle)
		b.Heading = NormalizeAngle(b.Heading + angularVelocity*dt)

		// Move along current heading
		b.Pos.X += b.Speed * math.Cos(b.Heading) * dt
		b.Pos.Y += b.Speed * math.Sin(b.Heading) * dt
	}

	// 5. Update tire tracks
	b.updateTireTracks(dt)
}

func (b *Bus) updateTireTracks(dt float64) {
	// Fade existing tracks
	for i := range b.TireTracks {
		b.TireTracks[i].Alpha -= float32(dt * 0.4)
	}
	// Filter out invisible tracks
	valid := b.TireTracks[:0]
	for _, t := range b.TireTracks {
		if t.Alpha > 0.05 {
			valid = append(valid, t)
		}
	}
	b.TireTracks = valid

	// Add new track if braking hard or drifting
	if (b.IsHandbraking || (b.IsBraking && math.Abs(b.Speed) > 100)) && math.Abs(b.Speed) > 30 {
		rl, rr := b.getRearWheelPositions()
		if len(b.TireTracks) > 1 {
			last := b.TireTracks[len(b.TireTracks)-1]
			if rl.Distance(last.Pos1) > 10 {
				b.TireTracks = append(b.TireTracks,
					TireMark{Pos1: last.Pos1, Pos2: rl, Alpha: 0.6},
					TireMark{Pos1: last.Pos2, Pos2: rr, Alpha: 0.6},
				)
			}
		} else {
			b.TireTracks = append(b.TireTracks,
				TireMark{Pos1: rl, Pos2: rl, Alpha: 0.6},
				TireMark{Pos1: rr, Pos2: rr, Alpha: 0.6},
			)
		}
	}
}

func (b *Bus) getRearWheelPositions() (Vec2, Vec2) {
	cosH := math.Cos(b.Heading)
	sinH := math.Sin(b.Heading)
	perpX := -sinH
	perpY := cosH

	// Rear axle offset from center (-b.WheelBase/2)
	rearX := b.Pos.X - cosH*(b.WheelBase/2)
	rearY := b.Pos.Y - sinH*(b.WheelBase/2)

	halfTrack := b.Width/2 - 2
	rl := Vec2{X: rearX - perpX*halfTrack, Y: rearY - perpY*halfTrack}
	rr := Vec2{X: rearX + perpX*halfTrack, Y: rearY + perpY*halfTrack}
	return rl, rr
}
