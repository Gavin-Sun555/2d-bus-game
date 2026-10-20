package integration_test

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

func TestTrafficAICirculatesRoundaboutWithoutCrossingCenterIsland(t *testing.T) {
	// Build Emerald Lake map which features the 湖心仙岛古风环岛
	w := sim.NewWorld(42.0, sim.MapThemeEmeraldLake)
	if len(w.Roundabouts) == 0 {
		t.Fatalf("Emerald lake map must have a roundabout")
	}
	rb := w.Roundabouts[0]
	center := rb.Center
	innerR := rb.InnerRadius
	laneW := 42.0

	// Test approaches from East, West, North, South
	approaches := []struct {
		name         string
		startPos     sim.Vec2
		heading      float64
		isEW         bool
		roadCenter   float64
		expectedExit float64
	}{
		{
			name:         "Eastbound (from West)",
			startPos:     sim.Vec2{X: center.X - 350, Y: center.Y + laneW*0.75},
			heading:      0,
			isEW:         true,
			roadCenter:   center.Y,
			expectedExit: 0,
		},
		{
			name:         "Westbound (from East)",
			startPos:     sim.Vec2{X: center.X + 350, Y: center.Y - laneW*0.75},
			heading:      math.Pi,
			isEW:         true,
			roadCenter:   center.Y,
			expectedExit: math.Pi,
		},
		{
			name:         "Southbound (from North)",
			startPos:     sim.Vec2{X: center.X + laneW*0.75, Y: center.Y - 350},
			heading:      math.Pi / 2,
			isEW:         false,
			roadCenter:   center.X,
			expectedExit: math.Pi / 2,
		},
		{
			name:         "Northbound (from South)",
			startPos:     sim.Vec2{X: center.X - laneW*0.75, Y: center.Y + 350},
			heading:      -math.Pi / 2,
			isEW:         false,
			roadCenter:   center.X,
			expectedExit: -math.Pi / 2,
		},
	}

	bus := sim.NewBus(5000, 5000, 0) // bus placed far away
	dt := 1.0 / 60.0

	for _, app := range approaches {
		t.Run(app.name, func(t *testing.T) {
			car := &sim.TrafficCar{
				ID:          0,
				Pos:         app.startPos,
				Heading:     app.heading,
				Speed:       140,
				TargetSpeed: 140,
				Length:      46,
				Width:       22,
				RoadCenter:  app.roadCenter,
				IsEW:        app.isEW,
				LaneIndex:   1,
			}
			tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}

			enteredRoundabout := false
			minDistToCenter := math.MaxFloat64

			// Simulate 350 frames (~5.8 seconds), enough to enter, circulate, and exit
			for frame := 0; frame < 350; frame++ {
				tm.Update(dt, bus, nil, 1, laneW, w)

				dist := car.Pos.Distance(center)
				if dist < minDistToCenter {
					minDistToCenter = dist
				}

				if car.InRoundabout {
					enteredRoundabout = true
					// Vehicle must NEVER penetrate the inner island of the roundabout
					if dist < innerR {
						t.Fatalf("[%s] AI Car penetrated central island! Car Pos: %v, Center: %v, Dist: %.2f < InnerRadius: %.2f",
							app.name, car.Pos, center, dist, innerR)
					}
					// Vehicle must maintain buffer distance (>= 15px) outside the inner island
					if dist < innerR+15.0 {
						t.Fatalf("[%s] AI Car too close to central island curb: dist=%.2f, innerR=%.2f",
							app.name, dist, innerR)
					}

					// Verify vehicle heading aligns to circle tangent during steady circulation
					// (permitting smooth gradual steering transition during entry and exit)
					if car.RoundaboutCurve == nil && car.RoundaboutTraveled >= 0.5 && car.RoundaboutTraveled <= 2.0 {
						expectedTangentHeading := sim.NormalizeAngle(car.RoundaboutAngle - math.Pi/2)
						headingDiff := math.Abs(sim.NormalizeAngle(car.Heading - expectedTangentHeading))
						if headingDiff > 0.15 {
							t.Fatalf("[%s] AI Car heading %.2f differs from rotary tangent %.2f (diff=%.3f)",
								app.name, car.Heading, expectedTangentHeading, headingDiff)
						}
					}
				}
			}

			if !enteredRoundabout {
				t.Fatalf("[%s] AI Car should have entered and circulated around the roundabout", app.name)
			}
			if minDistToCenter < innerR {
				t.Fatalf("[%s] AI Car minimum distance to center %.2f is inside inner island %.2f",
					app.name, minDistToCenter, innerR)
			}
		})
	}
}

// TestTrafficAIRoundaboutSmoothnessNoTeleportation strictly monitors the car's frame-to-frame displacement
// and heading rate to guarantee 100% absence of teleportation, snapping, or flickering ("闪现")
// during entry, circular travel, and exit transitions.
func TestTrafficAIRoundaboutSmoothnessNoTeleportation(t *testing.T) {
	w := sim.NewWorld(42.0, sim.MapThemeEmeraldLake)
	if len(w.Roundabouts) == 0 {
		t.Fatalf("Emerald lake map must have a roundabout")
	}
	rb := w.Roundabouts[0]
	center := rb.Center
	laneW := 42.0
	bus := sim.NewBus(5000, 5000, 0)
	dt := 1.0 / 60.0

	// Test Eastbound and Westbound approaches across the small roundabout
	testCases := []struct {
		name     string
		startPos sim.Vec2
		heading  float64
		isEW     bool
		roadC    float64
	}{
		{"Eastbound", sim.Vec2{X: center.X - 320, Y: center.Y + laneW*0.75}, 0, true, center.Y},
		{"Westbound", sim.Vec2{X: center.X + 320, Y: center.Y - laneW*0.75}, math.Pi, true, center.Y},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			car := &sim.TrafficCar{
				ID:          0,
				Pos:         tc.startPos,
				Heading:     tc.heading,
				Speed:       140,
				TargetSpeed: 140,
				Length:      46,
				Width:       22,
				RoadCenter:  tc.roadC,
				IsEW:        tc.isEW,
				LaneIndex:   1,
			}
			tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}

			prevPos := car.Pos
			prevHeading := car.Heading
			maxStepDisplacement := 0.0
			maxHeadingChange := 0.0

			for frame := 0; frame < 360; frame++ {
				tm.Update(dt, bus, nil, 1, laneW, w)

				stepDisplacement := car.Pos.Distance(prevPos)
				headingChange := math.Abs(sim.NormalizeAngle(car.Heading - prevHeading))

				if stepDisplacement > maxStepDisplacement {
					maxStepDisplacement = stepDisplacement
				}
				if headingChange > maxHeadingChange {
					maxHeadingChange = headingChange
				}

				// At speed ~140px/s, nominal frame displacement is 140 * (1/60) = 2.33 px.
				// Any frame-to-frame jump exceeding 8.0 px represents an unphysical snap/teleport ("闪现").
				if stepDisplacement > 8.0 {
					t.Fatalf("[%s] Frame %d: Teleportation detected! Single-frame displacement = %.2f px > 8.0 px limit (prev: %v, curr: %v, inRoundabout: %v)",
						tc.name, frame, stepDisplacement, prevPos, car.Pos, car.InRoundabout)
				}

				// Heading must not instantaneously snap (e.g. 70+ degrees in a single frame)
				if headingChange > 0.25 {
					t.Fatalf("[%s] Frame %d: Instant heading snap detected! delta=%.3f rad (%.1f deg)",
						tc.name, frame, headingChange, headingChange*180.0/math.Pi)
				}

				prevPos = car.Pos
				prevHeading = car.Heading
			}

			// Validate maximum displacement across the entire 6-second journey was smooth and continuous
			if maxStepDisplacement > 6.0 {
				t.Errorf("[%s] Expected silky smooth displacement (<6.0px/frame), got max=%.2f px",
					tc.name, maxStepDisplacement)
			}
		})
	}
}

// TestTrafficAIRoundaboutPureCircularArcNoDiagonalCuts verifies that AI vehicles follow a true circular arc
// inside the roundabout at all times, without expanding outward diagonally or cutting into corner obstacles.
func TestTrafficAIRoundaboutPureCircularArcNoDiagonalCuts(t *testing.T) {
	themes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeEmeraldLake,
	}
	laneW := 42.0
	dt := 1.0 / 60.0
	bus := sim.NewBus(5000, 5000, 0)

	for _, theme := range themes {
		w := sim.NewWorld(laneW, theme)
		if len(w.Roundabouts) == 0 {
			continue
		}
		rb := w.Roundabouts[0]
		center := rb.Center

		t.Run(theme.DisplayName(), func(t *testing.T) {
			// Test approach from West (Eastbound)
			car := &sim.TrafficCar{
				ID:          0,
				Pos:         sim.Vec2{X: center.X - 350, Y: center.Y + laneW*0.75},
				Heading:     0,
				Speed:       140,
				TargetSpeed: 140,
				Length:      46,
				Width:       22,
				RoadCenter:  center.Y,
				IsEW:        true,
				LaneIndex:   1,
			}
			tm := &traffic.TrafficManager{Cars: []*sim.TrafficCar{car}}

			entered := false
			exited := false
			joinedLane := false
			lastRadialError := math.Inf(1)

			for frame := 0; frame < 400; frame++ {
				tm.Update(dt, bus, nil, 1, laneW, w)

				if car.InRoundabout {
					entered = true
					dist := car.Pos.Distance(center)
					exiting := car.RoundaboutCurve != nil && car.RoundaboutCurve.Exiting
					joining := car.RoundaboutCurve != nil && !car.RoundaboutCurve.Exiting

					// Entry converges over driving distance, so stopped/slow cars
					// cannot slide sideways. Once joined, hold the circular lane.
					radialError := math.Abs(dist - car.RoundaboutTargetR)
					if radialError <= 8 && !joining {
						joinedLane = true
					}
					if !joining && !exiting && !joinedLane && radialError > lastRadialError+.01 {
						t.Fatal("entry radius moved away from its lane")
					}
					if car.RoundaboutTraveled >= 1.2 && !joinedLane {
						t.Fatal("car failed to converge to its circulating lane")
					}
					lastRadialError = radialError
					if joinedLane && !joining && !exiting {
						if radialError > 8.0 {
							t.Fatalf("[%s] Frame %d: Car radius deviated from circular target: dist=%.2f, targetR=%.2f, diff=%.2f > 8px",
								theme.DisplayName(), frame, dist, car.RoundaboutTargetR, math.Abs(dist-car.RoundaboutTargetR))
						}
					}

					// 2. Car distance must stay strictly between inner island and outer curb with safe buffer
					if dist < rb.InnerRadius+15.0 {
						t.Fatalf("[%s] Frame %d: Car too close to central island: dist=%.2f, innerR=%.2f",
							theme.DisplayName(), frame, dist, rb.InnerRadius)
					}
					if car.RoundaboutTraveled >= 0.40 && !exiting && dist > rb.Radius-15.0 {
						t.Fatalf("[%s] Frame %d: Car drifted into outer curb obstacle zone: dist=%.2f, outerR=%.2f",
							theme.DisplayName(), frame, dist, rb.Radius)
					}

					// 3. Boundary check must confirm vehicle is squarely on ZoneRoad at all times
					bRes := w.CheckBoundaries(car.Pos, car.Width/2, laneW)
					if bRes.Zone != sim.ZoneRoad {
						t.Fatalf("[%s] Frame %d: Car at %v not on ZoneRoad (got %v, obstacle: %s)",
							theme.DisplayName(), frame, car.Pos, bRes.Zone, bRes.ObstacleName)
					}

					// 4. During steady rotary travel, heading must align with circle tangent
					if !joining && !exiting && car.RoundaboutTraveled >= 0.5 && car.RoundaboutTraveled <= 2.0 {
						expectedTangent := sim.NormalizeAngle(car.RoundaboutAngle - math.Pi/2)
						headingErr := math.Abs(sim.NormalizeAngle(car.Heading - expectedTangent))
						if headingErr > 0.12 {
							t.Fatalf("[%s] Frame %d: Car heading %.2f deviated from circle tangent %.2f by %.3f rad",
								theme.DisplayName(), frame, car.Heading, expectedTangent, headingErr)
						}
					}
				} else if entered {
					exited = true
					// Once exited, verify car is safely on road
					bRes := w.CheckBoundaries(car.Pos, car.Width/2, laneW)
					if bRes.Zone == sim.ZoneHardBarrier {
						t.Fatalf("[%s] Frame %d: Car hit hard barrier upon exiting roundabout: %v, obstacle: %s",
							theme.DisplayName(), frame, car.Pos, bRes.ObstacleName)
					}
				}
			}

			if !entered {
				t.Fatalf("[%s] AI Car failed to enter roundabout", theme.DisplayName())
			}
			if !exited {
				t.Fatalf("[%s] AI Car failed to exit roundabout", theme.DisplayName())
			}
		})
	}
}
