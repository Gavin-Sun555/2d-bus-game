package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Replay the small-but-physical centerline clearance from Twin Cities. No
// candidate can create a two-pixel reserve immediately; full lock is unsafe.
func TestRoadSafetySmallClearanceDoesNotForceFullLock(t *testing.T) {
	for _, lw := range []float64{42, 52, 64, 78} {
		for _, lanes := range []int{4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				t.Run(fmt.Sprintf("width%.0f/lanes%d/heading%.2f", lw, lanes, heading), func(t *testing.T) {
					u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
					right := sim.Vec2{X: -u.Y, Y: u.X}
					w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-4000), P2: u.Mul(4000), Lanes: lanes}}}
					p := right.Mul(19.5)
					bus := sim.NewBus(p.X, p.Y, heading-.01)
					bus.Speed = 415
					bus.SteeringAngle = .025
					ad := NewAutopilotDriver()
					ad.drivePath = []sim.Vec2{right.Mul(lw / 2).Sub(u.Mul(2000)), right.Mul(lw / 2).Add(u.Mul(2000))}
					ad.cumLen = []float64{0, 4000}
					f := autopilotFrame{bus: bus, world: w, laneWidth: lw, s: 2000, speedAbs: 415}
					steer := ad.roadLaneSafeSteering(f, .025)
					if math.Abs(steer) > .25 {
						t.Fatalf("small clearance caused hard steering %.3f", steer)
					}
					if ad.roadSafetyHold {
						t.Fatal("physically clear recovery should remain possible")
					}
					// A yaw into the median cannot be made safe by full right at this speed.
					bus.Heading = heading - .3
					steer = ad.roadLaneSafeSteering(f, -.2)
					if !ad.roadSafetyHold || math.Abs(steer) > .25 {
						t.Fatalf("infeasible sweep must brake with bounded recovery, hold=%v steer=%.2f", ad.roadSafetyHold, steer)
					}
					bus.Pos = right.Mul(lw * .9)
					bus.Heading = heading
					bus.SteeringAngle = 0
					ad.roadLaneSafeSteering(f, 0)
					if ad.roadSafetyHold {
						t.Fatal("hold persisted after safe geometry restored")
					}
				})
			}
		}
	}
}

func TestBorrowingLaneReservesAccelerationAndReturnTime(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
		right := sim.Vec2{X: -u.Y, Y: u.X}
		bus := sim.NewBus(0, 0, heading)
		obs := VehicleObstacle{Pos: u.Mul(2100).Sub(right.Mul(52)), Heading: heading + math.Pi, Speed: 230, Length: 48, Width: 24}
		if isOncomingLaneClear(bus, []VehicleObstacle{obs}, u.X, u.Y, 0, -52, 52, 340, 667) {
			t.Fatalf("heading%.2f borrowed with insufficient time to accelerate, pass and return", heading)
		}
		obs.Pos = u.Mul(5000).Sub(right.Mul(52))
		if !isOncomingLaneClear(bus, []VehicleObstacle{obs}, u.X, u.Y, 0, -52, 52, 340, 667) {
			t.Fatalf("heading%.2f distant vehicle incorrectly prevents pass", heading)
		}
	}
}

func TestJunctionSweepIncludesYawedNoseAndClearedTraffic(t *testing.T) {
	w := sim.NewWorld(26, sim.MapThemeTwinCities)
	ad := NewAutopilotDriver()
	ad.drivePath = []sim.Vec2{{X: 3880, Y: -10}, {X: 3840, Y: -26}, {X: 3500, Y: -26}}
	ad.cumLen = []float64{0, math.Hypot(40, 16), math.Hypot(40, 16) + 340}
	bus := sim.NewBus(3880, -10, 2.7)
	obs := VehicleObstacle{Pos: sim.Vec2{X: 3845, Y: 26}, Heading: 0, Length: 48, Width: 24}
	f := autopilotFrame{bus: bus, world: w, laneWidth: 26, speedAbs: 30, obstacles: []VehicleObstacle{obs}}
	if speed := ad.junctionSweptSpeed(f); speed != 0 {
		t.Fatalf("nose overlapping opposing lane must stop, got %v", speed)
	}
	bus.Heading = math.Atan2(-16, -40)
	if speed := ad.junctionSweptSpeed(f); !math.IsInf(speed, 1) {
		t.Fatalf("body-clear parallel lanes falsely blocked, speed=%v", speed)
	}
	f.obstacles[0].Pos = sim.Vec2{X: 4200, Y: 200}
	if speed := ad.junctionSweptSpeed(f); !math.IsInf(speed, 1) {
		t.Fatalf("cleared traffic falsely blocked, speed=%v", speed)
	}
}

// Reproduce the lake ring's nearly ninety-degree circle-to-road kink. Check
// rotated copies too: the long nose must clear cars waiting on the other side.
func TestNarrowRoundaboutDepartureClearsOpposingQueue(t *testing.T) {
	const lw = 52.
	original := sim.NewWorld(lw, sim.MapThemeEmeraldLake)
	route := sim.GetStandardRouteForMap(sim.MapThemeEmeraldLake, original)
	raw := sim.ComputeRoadPathBetweenStops(original, original.Stops[route.StopIDs[0]], original.Stops[route.StopIDs[1]], lw)
	rb := original.Roundabouts[0]
	for _, angle := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", angle), func(t *testing.T) {
			rotate := func(p sim.Vec2) sim.Vec2 {
				return sim.Vec2{X: p.X*math.Cos(angle) - p.Y*math.Sin(angle), Y: p.X*math.Sin(angle) + p.Y*math.Cos(angle)}
			}
			w := &sim.World{}
			rotatedRing := rb
			rotatedRing.Center = sim.Vec2{}
			w.Roundabouts = []sim.Roundabout{rotatedRing}
			for _, road := range original.Roads {
				road.P1 = rotate(road.P1.Sub(rb.Center))
				road.P2 = rotate(road.P2.Sub(rb.Center))
				w.Roads = append(w.Roads, road)
			}
			path := make([]sim.Vec2, len(raw))
			for i, p := range raw {
				path[i] = rotate(p.Sub(rb.Center))
			}
			exit := -1
			for i := 1; i < len(path)-1; i++ {
				if path[i].Length() <= rb.Radius+15 && path[i+1].Length() > rb.Radius+15 {
					exit = i
					break
				}
			}
			if exit < 0 {
				t.Fatal("fixture missing ring departure")
			}
			outgoing := path[exit+1].Sub(path[exit]).Normalize()
			validator := AutopilotDriver{pathWorld: w}
			start, curve := findRoundaboutDeparture(&validator, &rotatedRing, path, exit, outgoing, lw)
			if start < 0 || len(curve) < 3 {
				t.Fatal("clear narrow-road departure needs a continuous curve")
			}
			opposing := sim.OBB{Center: rotate(sim.Vec2{X: 289, Y: -26}), Heading: angle + math.Pi, HalfLen: 24, HalfWid: 12}
			for i := 1; i < len(curve); i++ {
				u := curve[i].Sub(curve[i-1])
				body := sim.OBB{Center: curve[i], Heading: math.Atan2(u.Y, u.X), HalfLen: 48, HalfWid: 18}
				if hit, _, _ := sim.CheckOBBCollision(body, opposing); hit {
					t.Fatalf("nose hit opposing queue at point%d %v", i, curve[i])
				}
				if !validator.roundaboutExitBodyOnRoad(body, &rotatedRing, lw) {
					t.Fatalf("departure left road at point%d", i)
				}
			}
			smoothed := smoothRoundaboutDepartures(w, path, lw)
			if len(smoothed) == len(path) {
				t.Fatal("path builder retained sharp departure")
			}
		})
	}
}

func TestOncomingSpeedIsNotTreatedAsForwardLeadSpeed(t *testing.T) {
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	bus := sim.NewBus(0, 0, 0)
	f := autopilotFrame{bus: bus, laneWidth: 52, speedAbs: 340, obstacles: []VehicleObstacle{{Pos: sim.Vec2{X: 120}, Heading: math.Pi, Speed: 230, Length: 48, Width: 24}}}
	opposing := ad.obstacleSpeed(f, 667)
	f.obstacles[0].Heading = 0
	sameDirection := ad.obstacleSpeed(f, 667)
	if opposing >= sameDirection-100 {
		t.Fatalf("oncoming car incorrectly contributes forward speed: opposing%.1f following%.1f", opposing, sameDirection)
	}
}

func TestDepartureTaperStillProtectsCenterline(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprintf("heading%.2f", heading), func(t *testing.T) {
			u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
			right := sim.Vec2{X: -u.Y, Y: u.X}
			w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 4}}}
			stop := sim.NewBusStop(0, "start", sim.Vec2{}, heading, 4, sim.RGBA(0, 0, 0, 255), 52)
			w.Stops = []*sim.BusStop{stop}
			ad := NewAutopilotDriver()
			ad.pathFromID = 0
			ad.pathWorld = w
			ad.drivePath = []sim.Vec2{stop.Pos, u.Mul(160).Add(right.Mul(78)), u.Mul(2000).Add(right.Mul(78))}
			ad.cumLen = []float64{0, ad.drivePath[0].Distance(ad.drivePath[1]), ad.drivePath[0].Distance(ad.drivePath[1]) + 1840}
			p := u.Mul(130).Add(right.Mul(50))
			bus := sim.NewBus(p.X, p.Y, heading-.35)
			bus.Speed = 300
			bus.SteeringAngle = -.1
			steer := ad.roadLaneSafeSteering(autopilotFrame{bus: bus, world: w, laneWidth: 52, s: 100, speedAbs: 300}, -1)
			if steer < 0 {
				t.Fatalf("departure taper disabled centerline protection: steer%.2f", steer)
			}
		})
	}
}
