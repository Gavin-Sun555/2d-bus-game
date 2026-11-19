package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestLeftTurnUsesGeneralLaneBesideStationaryBusQueue(t *testing.T) {
	for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for _, model := range []sim.BusModel{sim.BusStandard, sim.BusArticulated} {
			t.Run(fmt.Sprintf("h%.2f/model%d", heading, model), func(t *testing.T) {
				const lw = 52.
				w, _ := turnTestWorld(8, 8, heading, lw)
				for i := range w.Roads {
					w.Roads[i].Type = sim.RoadBusBoulevard8Lane
				}
				in := sim.UnitFromAngle(heading)
				rIn := sim.Vec2{X: -in.Y, Y: in.X}
				out := sim.Vec2{X: in.Y, Y: -in.X}
				rOut := sim.Vec2{X: -out.Y, Y: out.X}
				g := w.JunctionAt(sim.Vec2{}, lw)
				half := math.Abs(out.X)*g.HalfX + math.Abs(out.Y)*g.HalfY
				pos := in.Mul(-1000).Add(rIn.Mul(3.5 * lw))
				bus := sim.NewBus(pos.X, pos.Y, heading)
				bus.SetModel(model)
				ad := NewAutopilotDriver()
				ad.CruiseSpeedLimit = 666.67
				route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
				tls := sim.NewTrafficLightSystem()
				tls.BuildForWorld(w)
				tls.Intersections[0].Phase = sim.PhaseEWLeftGreen
				if math.Abs(in.Y) > .5 {
					tls.Intersections[0].Phase = sim.PhaseNSLeftGreen
				}
				var obstacles []VehicleObstacle
				for _, along := range []float64{half - 220, half + 80, half + 320, half + 560} {
					p := out.Mul(along).Add(rOut.Mul(3.5 * lw))
					other := sim.NewBus(p.X, p.Y, heading-math.Pi/2)
					other.SetModel(sim.BusArticulated)
					other.JunctionCorridor = &sim.JunctionCorridor{Center: sim.Vec2{}, EntryHeading: heading - math.Pi/2, Entered: true, Remaining: 1,
						Bodies: sim.AppendPathBodies(nil, []sim.Vec2{p, p.Add(out.Mul(800))}, other.Length+8, other.Width+6)}
					other.JunctionCorridor.RebuildBounds()
					obstacles = append(obstacles, ObserveBus(other))
				}
				for tick := 0; tick < 5400; tick++ {
					th, br, st, hb, doors, _ := ad.ComputeControls(bus, route, w, tls, obstacles, lw, 1./60)
					bus.Update(1./60, th, br, st, hb, doors)
					for _, ob := range obstacles {
						if sweepBodiesCollision(bus.BodyParts(), ob, 0, 0, 0) {
							t.Fatalf("hit fixed outgoing queue tick%d", tick)
						}
					}
					if bus.Pos.Dot(out) > half+350 {
						// Lane membership, not exact centering: a compact bus may
						// leave the turn a few pixels inside the lane center.
						if lateral := bus.Pos.Dot(rOut); math.Min(math.Abs(lateral-2.5*lw), math.Abs(lateral-1.5*lw)) > lw/4 {
							t.Fatalf("did not retain selected general lane: %v", bus.Pos)
						}
						return
					}
				}
				d := ad.DiagnoseStop(bus, route, w, tls, obstacles, lw)
				t.Fatalf("left turn stayed blocked by curb queue: pos%v speed%.1f limits%v", bus.Pos, bus.Speed, d.Limits)
			})
		}
	}
}

func TestLeftTurnExitChoiceRetainsOccupiedLaneAndCommitmentGuards(t *testing.T) {
	for _, allBlocked := range []bool{false, true} {
		t.Run(fmt.Sprint(allBlocked), func(t *testing.T) {
			const lw = 52.
			w, _ := turnTestWorld(8, 8, 0, lw)
			for i := range w.Roads {
				w.Roads[i].Type = sim.RoadBusBoulevard8Lane
			}
			bus := sim.NewBus(-1000, 3.5*lw, 0)
			ad := NewAutopilotDriver()
			ad.PrepareLeg(w, w.Stops[0], w.Stops[1], lw)
			f := autopilotFrame{bus: bus, world: w, laneWidth: lw}
			f.s, _ = ad.trackPath(bus.Pos)
			for lane := 3; lane >= 1; lane-- {
				if lane != 3 && !allBlocked {
					continue
				}
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: sim.Vec2{X: (float64(lane) + .5) * lw, Y: -600}, Heading: -math.Pi / 2, Length: 96, Width: 36})
			}
			ad.chooseLeftTurnExit(f, sim.Vec2{}, 0)
			if allBlocked {
				if ad.exitCruise != nil {
					t.Fatal("selected an occupied outgoing lane")
				}
				return
			}
			if ad.exitCruise == nil || !ad.exitCruise.LeftTurn {
				t.Fatal("did not choose clear outgoing general lane")
			}
			target := ad.exitCruise.Target
			restored, err := ad.Snapshot().Restore(w)
			if err != nil {
				t.Fatal(err)
			}
			f.obstacles = []VehicleObstacle{{Pos: sim.Vec2{X: target.X, Y: -600}, Heading: -math.Pi / 2, Length: 96, Width: 36}}
			restored.chooseLeftTurnExit(f, sim.Vec2{}, 0)
			if restored.exitCruise.Target != target {
				t.Fatal("traffic change reversed committed turn exit after restore")
			}
		})
	}
}

func TestParallelArticulatedBusesTurnLeftConcurrently(t *testing.T) {
	for _, legacyOuterMidTurn := range []bool{false, true} {
		t.Run(fmt.Sprintf("legacyOuter%v", legacyOuterMidTurn), func(t *testing.T) {
			const lw = 52.0
			w := &sim.World{Roads: []sim.RoadSegment{
				{P1: sim.Vec2{X: -3000, Y: 0}, P2: sim.Vec2{X: 3000, Y: 0}, Lanes: 4},
				{P1: sim.Vec2{X: 0, Y: -3000}, P2: sim.Vec2{X: 0, Y: 3000}, Lanes: 8, Type: sim.RoadBusBoulevard8Lane},
			}}
			w.RebuildIntersections()
			from := sim.NewBusStop(0, "from", sim.Vec2{X: -2000, Y: 1.5 * lw}, 0, 4, sim.RGBA(0, 0, 0, 255), lw)
			to := sim.NewBusStop(1, "to", sim.Vec2{X: 3.5*lw + 47, Y: -900}, -math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), lw)
			w.Stops = []*sim.BusStop{from, to}
			route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

			tls := sim.NewTrafficLightSystem()
			tls.BuildForWorld(w)
			inter := tls.Intersections[0]
			inter.Phase = sim.PhaseEWLeftGreen
			inter.Timer = 0
			line := inter.GetStopLineDistForHeading(0, lw)

			inner := sim.NewBus(-line-60, .5*lw, 0)
			inner.SetModel(sim.BusArticulated)
			inner.Gear = sim.GearDrive
			adInner := NewAutopilotDriver()
			adInner.BerserkMode = true
			adInner.DisallowOncomingOvertake = true
			adInner.State = AutopilotStateCruising
			adInner.PrepareLeg(w, from, to, lw)

			outer := sim.NewBus(-line-60, 1.5*lw, 0)
			outer.SetModel(sim.BusArticulated)
			outer.Gear = sim.GearDrive
			adOuter := NewAutopilotDriver()
			adOuter.BerserkMode = true
			adOuter.DisallowOncomingOvertake = true
			adOuter.State = AutopilotStateCruising
			adOuter.PrepareLeg(w, from, to, lw)
			if legacyOuterMidTurn {
				g := w.JunctionAt(sim.Vec2{}, lw)
				curve := sim.WideLeftTurnPath(sim.Vec2{}, sim.Vec2{X: 1}, sim.Vec2{Y: -1}, 1.5*lw, 3.5*lw, g)
				outerPos := sim.Vec2{X: -line + 175, Y: 43.1}
				pts := append([]sim.Vec2{{X: -1200, Y: 1.5 * lw}}, curve...)
				pts = append(pts, sim.Vec2{X: 3.5 * lw, Y: -850}, to.Pos)
				adOuter.drivePath = pts
				adOuter.cumLen = make([]float64, len(pts))
				for i := 1; i < len(pts); i++ {
					adOuter.cumLen[i] = adOuter.cumLen[i-1] + pts[i].Distance(pts[i-1])
				}
				adOuter.WaypointIdx = 1
				fb := sim.Vec2{}
				adOuter.fallbackJunction = &fb
				next := outer.AtPose(outerPos, -.36)
				outer = &next
				outer.Speed = 95
			}

			for tick := 0; tick < 360; tick++ {
				obsInner := []VehicleObstacle{ObserveBus(outer)}
				obsOuter := []VehicleObstacle{ObserveBus(inner)}
				thO, brO, stO, hbO, dO, _ := adOuter.ComputeControls(outer, route, w, tls, obsOuter, lw, 1./60)
				thI, brI, stI, hbI, dI, _ := adInner.ComputeControls(inner, route, w, tls, obsInner, lw, 1./60)
				outer.Update(1./60, thO, brO, stO, hbO, dO)
				inner.Update(1./60, thI, brI, stI, hbI, dI)
				if sweepBodiesCollision(inner.BodyParts(), ObserveBus(outer), 0, 0, 0) {
					t.Fatalf("parallel turning buses collided at tick %d: inner=%v outer=%v", tick, inner.Pos, outer.Pos)
				}
			}
			if inner.Pos.Y >= -120 || outer.Pos.Y >= -120 {
				t.Fatalf("expected both buses to clear left turn concurrently on same green: inner=%v outer=%v", inner.Pos, outer.Pos)
			}
		})
	}
}

func TestParallelLeftTurnExitSkipsOutgoingLaneTaper(t *testing.T) {
	const lw = 52.0
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -3000, Y: 0}, P2: sim.Vec2{X: 3000, Y: 0}, Lanes: 8},
		{P1: sim.Vec2{X: 0, Y: -3000}, P2: sim.Vec2{X: 0, Y: 3000}, Lanes: 8},
		{P1: sim.Vec2{X: -3000, Y: -1800}, P2: sim.Vec2{X: 3000, Y: -1800}, Lanes: 8},
	}}
	w.RebuildIntersections()
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -1800, Y: 3.5 * lw}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)
	// At (0, -1800) heading north (-Pi/2), turning west (Pi) is a LEFT turn,
	// so PrepareLeg generates a 3.5*lw -> 0.5*lw taper south of (0,0).
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -1800, Y: -1800 - 3.5*lw}, math.Pi, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	tls.Intersections[0].Phase = sim.PhaseEWLeftGreen
	g := w.JunctionAt(sim.Vec2{}, lw)

	line := tls.Intersections[0].GetStopLineDistForHeading(0, lw)
	bus := sim.NewBus(-line-160, .5*lw, 0)
	bus.SetModel(sim.BusArticulated)
	bus.Gear = sim.GearDrive
	ad := NewAutopilotDriver()
	ad.articulatedPath = true
	ad.PrepareLeg(w, from, to, lw)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}

	outer := sim.NewBus(-line-20, 3.5*lw, 0)
	outer.SetModel(sim.BusArticulated)
	outer.Gear = sim.GearDrive
	outer.Speed = 110
	adOuter := NewAutopilotDriver()
	adOuter.articulatedPath = true
	adOuter.PrepareLeg(w, from, to, lw)

	switchedInner := false
	for tick := 0; tick < 600; tick++ {
		obsInner := []VehicleObstacle{ObserveBus(outer)}
		obsOuter := []VehicleObstacle{ObserveBus(bus)}
		thO, brO, stO, hbO, dO, _ := adOuter.ComputeControls(outer, route, w, tls, obsOuter, lw, 1./60)
		thI, brI, stI, hbI, dI, _ := ad.ComputeControls(bus, route, w, tls, obsInner, lw, 1./60)
		outer.Update(1./60, thO, brO, stO, hbO, dO)
		bus.Update(1./60, thI, brI, stI, hbI, dI)
		if ad.exitCruise != nil && ad.exitCruise.LeftTurn && math.Abs(ad.exitCruise.Target.X-0.5*lw) <= 1 {
			switchedInner = true
		}
		if switchedInner && bus.Pos.Y <= -g.HalfY-200 && bus.Pos.Y >= -1500 && bus.Pos.X > 0.5*lw+25 {
			t.Fatalf("parallel left exit swerved into outer lanes at tick %d: pos=%v", tick, bus.Pos)
		}
		if bus.Pos.Y < -1400 {
			break
		}
	}
	if !switchedInner {
		t.Fatalf("expected inner exit choice at 0.5*lw, exitCruise=%+v (pos=%v outer=%v)", ad.exitCruise, bus.Pos, outer.Pos)
	}
	for _, pt := range ad.drivePath {
		if pt.Y <= -g.HalfY-200 && pt.Y >= -1500 && pt.X > 0.5*lw+3 {
			t.Fatalf("parallel left exit swerved back into outer lanes at %v", pt)
		}
	}
}

func TestParallelLeftTurnExitAvoidsMultiKmDiagonal(t *testing.T) {
	const lw = 52.0
	w := &sim.World{Roads: []sim.RoadSegment{
		{P1: sim.Vec2{X: -3000, Y: 0}, P2: sim.Vec2{X: 3000, Y: 0}, Lanes: 8},
		{P1: sim.Vec2{X: 0, Y: -12000}, P2: sim.Vec2{X: 0, Y: 3000}, Lanes: 8},
		{P1: sim.Vec2{X: -3000, Y: -10000}, P2: sim.Vec2{X: 3000, Y: -10000}, Lanes: 8},
	}}
	w.RebuildIntersections()
	from := sim.NewBusStop(0, "from", sim.Vec2{X: -1800, Y: 3.5 * lw}, 0, 8, sim.RGBA(0, 0, 0, 255), lw)
	to := sim.NewBusStop(1, "to", sim.Vec2{X: -1800, Y: -10000 - 3.5*lw}, math.Pi, 8, sim.RGBA(0, 0, 0, 255), lw)
	w.Stops = []*sim.BusStop{from, to}
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	tls.Intersections[0].Phase = sim.PhaseEWLeftGreen
	line := tls.Intersections[0].GetStopLineDistForHeading(0, lw)

	bus := sim.NewBus(-line-160, .5*lw, 0)
	bus.SetModel(sim.BusArticulated)
	bus.Gear = sim.GearDrive
	ad := NewAutopilotDriver()
	ad.articulatedPath = true
	ad.PrepareLeg(w, from, to, lw)

	outer := sim.NewBus(-line-20, 3.5*lw, 0)
	outer.SetModel(sim.BusArticulated)
	outer.Gear = sim.GearDrive
	outer.Speed = 110
	adOuter := NewAutopilotDriver()
	adOuter.articulatedPath = true
	adOuter.PrepareLeg(w, from, to, lw)

	switchedInner := false
	for tick := 0; tick < 600; tick++ {
		obsInner := []VehicleObstacle{ObserveBus(outer)}
		obsOuter := []VehicleObstacle{ObserveBus(bus)}
		thO, brO, stO, hbO, dO, _ := adOuter.ComputeControls(outer, route, w, tls, obsOuter, lw, 1./60)
		thI, brI, stI, hbI, dI, _ := ad.ComputeControls(bus, route, w, tls, obsInner, lw, 1./60)
		outer.Update(1./60, thO, brO, stO, hbO, dO)
		bus.Update(1./60, thI, brI, stI, hbI, dI)
		if ad.exitCruise != nil && ad.exitCruise.LeftTurn && ad.exitCruise.Target.X <= 1.5*lw+1 {
			switchedInner = true
			break
		}
	}
	if !switchedInner {
		t.Fatalf("expected chooseParallelLeftTurnExit to pick 0.5*lw exit lane (exitCruise=%+v, pos=%v)", ad.exitCruise, bus.Pos)
	}
	for i := 1; i < len(ad.drivePath); i++ {
		a, b := ad.drivePath[i-1], ad.drivePath[i]
		if math.Abs(a.X-b.X) > 20 && a.Distance(b) > 1000 {
			t.Fatalf("segment %d->%d spans %.1fpx diagonally across lanes: %v -> %v", i-1, i, a.Distance(b), a, b)
		}
	}
}
