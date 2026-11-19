package autopilot

import (
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestArticulatedBusWaitsForRollingQueueToClearRailExit(t *testing.T) {
	const laneWidth = 52.
	world := &sim.World{LargeGrid: true, CenterBRT: true}
	world.BuildMap(sim.MapThemeRiverValley, laneWidth)
	rail := world.Railways()[0]
	found := false
	for _, crossing := range rail.Crossings {
		found = found || crossing.Center.X == 0 && crossing.Level
	}
	if !found {
		t.Fatal("expected the level crossing on the approach to the central junction")
	}
	gate := rail.HalfWidth + sim.RailGateMargin
	bus := sim.NewBus(laneWidth/2, rail.Y+gate+63, -math.Pi/2)
	bus.SetModel(sim.BusArticulated)
	bumper := bus.Pos.Add(sim.UnitFromAngle(bus.Heading).Mul(bus.Length / 2))
	lead := VehicleObstacle{
		Pos:     sim.Vec2{X: laneWidth / 2, Y: rail.Y - gate - 175},
		Heading: bus.Heading, Speed: 35, Length: 46, Width: 22,
	}
	frame := autopilotFrame{bus: bus, world: world, laneWidth: laneWidth, obstacles: []VehicleObstacle{lead}}
	if limit := railExitQueueSpeed(frame, bumper); limit > .01 {
		t.Fatalf("moving queue does not leave trailer room; got speed cap %.2f", limit)
	}
	frame.obstacles[0].Pos.Y -= 100
	if limit := railExitQueueSpeed(frame, bumper); !math.IsInf(limit, 1) {
		t.Fatalf("bus still held after leader opened space for both sections; got %.2f", limit)
	}
}

func TestBusDoesNotStopOnRailwayTracksForDownstreamIntersectionRedLight(t *testing.T) {
	const lw = 52.0
	for _, large := range []bool{false, true} {
		w := &sim.World{LargeGrid: large}
		w.BuildMap(sim.MapThemeMetropolis, lw)

		tls := sim.NewTrafficLightSystem()
		tls.BuildForWorld(w)

		tested := 0
		for _, rail := range w.Railways() {
			var levelCrossing *sim.RailCrossing
			var downstreamInter *sim.TrafficIntersection
			var dir float64
			for i := range rail.Crossings {
				c := &rail.Crossings[i]
				if !c.Level {
					continue
				}
				for _, inter := range tls.Intersections {
					if math.Abs(inter.Center.X-c.Center.X) > 10 {
						continue
					}
					if math.Abs(inter.Center.Y-rail.Y) < 600 {
						levelCrossing = c
						downstreamInter = inter
						if inter.Center.Y > rail.Y {
							dir = 1 // Southbound crosses railway before intersection
						} else {
							dir = -1 // Northbound crosses railway before intersection
						}
						break
					}
				}
				if levelCrossing != nil {
					break
				}
			}
			if levelCrossing == nil || downstreamInter == nil {
				continue
			}
			tested++

			for _, berserk := range []bool{false, true} {
				heading := math.Pi / 2
				if dir < 0 {
					heading = -math.Pi / 2
				}
				right := sim.Vec2{X: -math.Sin(heading), Y: math.Cos(heading)}
				laneX := levelCrossing.Center.X + right.X*(0.5*lw)
				startY := rail.Y - dir*(rail.HalfWidth+sim.RailGateMargin+260)

				bus := sim.NewBus(laneX, startY, heading)
				bus.SetModel(sim.BusArticulated)
				bus.Speed = 110
				if berserk {
					bus.Speed = 150
				}

				ad := NewAutopilotDriver()
				ad.BerserkMode = berserk
				path := []sim.Vec2{
					{X: laneX, Y: startY - dir*100},
					{X: laneX, Y: rail.Y},
					{X: laneX, Y: downstreamInter.Center.Y},
					{X: downstreamInter.Center.X + dir*300, Y: downstreamInter.Center.Y - dir*0.5*lw},
					{X: downstreamInter.Center.X + dir*1500, Y: downstreamInter.Center.Y - dir*0.5*lw},
				}
				setPath(ad, path)
				ad.articulatedPath = true
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

				// Hold downstream intersection signal RED for North-South traffic (EW Green)
				downstreamInter.Phase = sim.PhaseEWGreen
				downstreamInter.Timer = 0
				tls.RailElapsed = 0 // Keep railway gate open so only the road red light stops the bus

				const dt = 1.0 / 60.0
				for step := 0; step < 300; step++ {
					downstreamInter.Phase = sim.PhaseEWGreen
					downstreamInter.Timer = 1.0
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, dt)
					bus.Update(dt, th, br, st, hb, doors)
				}

				if math.Abs(bus.Speed) > 0.5 {
					t.Fatalf("large=%v railway Y=%.1f berserk=%v: expected bus to come to a stop for red light, got speed=%.2f",
						large, rail.Y, berserk, bus.Speed)
				}
				bumperY := bus.Pos.Y + dir*(bus.Length/2)
				deckStartY := rail.Y - dir*rail.HalfWidth
				if (bumperY-deckStartY)*dir > 0 {
					t.Fatalf("large=%v railway Y=%.1f berserk=%v: bus stopped with front bumper (Y=%.2f) on railway deck (deckStartY=%.2f)",
						large, rail.Y, berserk, bumperY, deckStartY)
				}
				for _, part := range bus.BodyParts() {
					if math.Abs(part.Center.Y-rail.Y)-part.HalfLen < rail.HalfWidth {
						t.Fatalf("large=%v railway Y=%.1f berserk=%v: bus body part %+v overlaps railway tracks [%.1f, %.1f]",
							large, rail.Y, berserk, part, rail.Y-rail.HalfWidth, rail.Y+rail.HalfWidth)
					}
				}

				// Now switch the intersection signal to NS Left Green and verify the bus proceeds across both railway and stop line.
				downstreamInter.Phase = sim.PhaseNSLeftGreen
				downstreamInter.Timer = 0
				clearedTracks := false
				for step := 0; step < 360; step++ {
					downstreamInter.Timer += dt
					if downstreamInter.Timer >= downstreamInter.LeftGreenDuration {
						downstreamInter.Phase = sim.PhaseNSLeftYellow
						downstreamInter.Timer = 0
					}
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, dt)
					bus.Update(dt, th, br, st, hb, doors)
					rearY := bus.Pos.Y - dir*(bus.OverallLength()-bus.Length/2)
					if (rearY-(rail.Y+dir*rail.HalfWidth))*dir > 0 {
						clearedTracks = true
						break
					}
				}
				if !clearedTracks {
					t.Fatalf("large=%v railway Y=%.1f berserk=%v: expected bus to cross and clear railway tracks on green, pos=%+v speed=%.2f",
						large, rail.Y, berserk, bus.Pos, bus.Speed)
				}
			}
		}
		if tested == 0 {
			t.Fatalf("large=%v: no level crossing near an intersection was found", large)
		}
	}
}

func TestBusDoesNotStopOnRailwayTracksBehindQueuedVehicle(t *testing.T) {
	const lw = 52.0
	w := &sim.World{LargeGrid: true}
	w.BuildMap(sim.MapThemeMetropolis, lw)

	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)

	var rail *sim.Railway
	var crossing *sim.RailCrossing
	for _, r := range w.Railways() {
		for i := range r.Crossings {
			if r.Crossings[i].Level {
				rail = r
				crossing = &r.Crossings[i]
				break
			}
		}
		if crossing != nil {
			break
		}
	}
	if rail == nil || crossing == nil {
		t.Fatal("missing level crossing")
	}

	heading := math.Pi / 2
	laneX := crossing.Center.X - 0.5*lw
	startY := rail.Y - (rail.HalfWidth + sim.RailGateMargin + 240)

	bus := sim.NewBus(laneX, startY, heading)
	bus.SetModel(sim.BusArticulated)
	bus.Speed = 110

	ad := NewAutopilotDriver()
	path := []sim.Vec2{
		{X: laneX, Y: startY - 100},
		{X: laneX, Y: rail.Y},
		{X: laneX, Y: rail.Y + 1200},
	}
	setPath(ad, path)
	ad.articulatedPath = true
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	// Place a stopped vehicle just past the railway crossing exit so that an
	// articulated bus cannot fit between the crossing exit and the stopped car.
	exitY := rail.Y + rail.HalfWidth + sim.RailGateMargin
	obs := []VehicleObstacle{
		{
			Pos:     sim.Vec2{X: laneX, Y: exitY + 140},
			Heading: heading,
			Speed:   0,
			Length:  48,
			Width:   22,
		},
	}

	const dt = 1.0 / 60.0
	for step := 0; step < 300; step++ {
		for _, inter := range tls.Intersections {
			inter.Phase = sim.PhaseNSGreen
			inter.Timer = 0
		}
		tls.RailElapsed = 0
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obs, lw, dt)
		bus.Update(dt, th, br, st, hb, doors)
	}

	if math.Abs(bus.Speed) > 0.5 {
		t.Fatalf("expected bus to hold before railway crossing when downstream queue lacks space, got speed=%.2f", bus.Speed)
	}
	bumperY := bus.Pos.Y + bus.Length/2
	deckStartY := rail.Y - rail.HalfWidth
	if bumperY > deckStartY {
		t.Fatalf("bus stopped on railway deck behind queued car: bumperY=%.2f deckStartY=%.2f", bumperY, deckStartY)
	}
}

func TestBusHoldsBeforeRailwayWhenDownstreamGreenExpiresSoon(t *testing.T) {
	const lw = 52.0
	w := &sim.World{LargeGrid: true}
	w.BuildMap(sim.MapThemeMetropolis, lw)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)

	r := w.Railways()[0]
	inter := tls.Intersections[2] // Center=(0.0, -14400.0)

	heading := math.Pi / 2 // Southbound
	laneX := 0.5 * lw
	startY := -15130.0

	bus := sim.NewBus(laneX, startY, heading)
	bus.SetModel(sim.BusArticulated)
	bus.Speed = 80

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.BerserkMaxSpeed = 180

	path := []sim.Vec2{
		{X: laneX, Y: startY - 100},
		{X: laneX, Y: r.Y},
		{X: laneX, Y: inter.Center.Y},
		{X: laneX, Y: inter.Center.Y + 1500},
	}
	setPath(ad, path)
	ad.articulatedPath = true
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	// Downstream intersection has only 2.5 seconds of green left:
	inter.Phase = sim.PhaseNSGreen
	inter.Timer = 5.5 // 8 - 5.5 = 2.5 seconds left
	tls.RailElapsed = 0

	const dt = 1.0 / 60.0
	for step := 0; step < 600; step++ {
		inter.Timer += dt
		if inter.Phase == sim.PhaseNSGreen && inter.Timer >= inter.GreenDuration {
			inter.Phase = sim.PhaseNSYellow
			inter.Timer = 0
		} else if inter.Phase == sim.PhaseNSYellow && inter.Timer >= inter.YellowDuration {
			inter.Phase = sim.PhaseEWGreen
			inter.Timer = 0
		}

		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, dt)
		bus.Update(dt, th, br, st, hb, doors)
		if bus.Speed < 0.1 && step > 60 {
			break
		}
	}

	railNorthDeck := r.Y - r.HalfWidth
	bumperY := bus.Pos.Y + bus.Length/2
	if bumperY > railNorthDeck {
		t.Fatalf("bus entered railway crossing when green time was insufficient: bumperY=%.1f > railNorthDeck=%.1f",
			bumperY, railNorthDeck)
	}
}

func TestBusClearsRailwayTracksWhenStoppedForDownstreamRed(t *testing.T) {
	const lw = 52.0
	w := &sim.World{LargeGrid: true}
	w.BuildMap(sim.MapThemeMetropolis, lw)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)

	r := w.Railways()[0]
	inter := tls.Intersections[2] // Center=(0.0, -14400.0)

	heading := math.Pi / 2 // Southbound
	laneX := 0.5 * lw
	// Place bus directly on railway tracks:
	startY := r.Y

	bus := sim.NewBus(laneX, startY, heading)
	bus.SetModel(sim.BusArticulated)
	bus.Speed = 40

	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.BerserkMaxSpeed = 180

	path := []sim.Vec2{
		{X: laneX, Y: startY - 100},
		{X: laneX, Y: r.Y},
		{X: laneX, Y: inter.Center.Y},
		{X: laneX, Y: inter.Center.Y + 1500},
	}
	setPath(ad, path)
	ad.articulatedPath = true
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	// Downstream intersection is RED:
	inter.Phase = sim.PhaseEWGreen
	inter.Timer = 1.0
	tls.RailElapsed = 0

	const dt = 1.0 / 60.0
	for step := 0; step < 600; step++ {
		th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, nil, lw, dt)
		bus.Update(dt, th, br, st, hb, doors)
		if bus.Speed < 0.1 && step > 60 {
			break
		}
	}

	rearY := bus.Pos.Y - (bus.OverallLength() - bus.Length/2)
	railSouthGate := r.Y + r.HalfWidth + sim.RailGateMargin
	if rearY < railSouthGate {
		t.Fatalf("bus stopped on railway tracks waiting for red light! rearY=%.1f < southGate=%.1f",
			rearY, railSouthGate)
	}
}
