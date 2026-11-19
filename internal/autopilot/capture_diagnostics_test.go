package autopilot

import (
	"encoding/json"
	"math"
	"os"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestAutopilotCaptureClearanceDiagnostics(t *testing.T) {
	p := os.Getenv("BUS_AUTOPILOT_CAPTURE")
	if p == "" {
		t.Skip("BUS_AUTOPILOT_CAPTURE not set")
	}
	id, e := strconv.Atoi(os.Getenv("BUS_AUTOPILOT_ID"))
	if e != nil {
		t.Fatal(e)
	}
	raw, e := os.ReadFile(p)
	if e != nil {
		t.Fatal(e)
	}
	var s struct {
		Data struct {
			World  *sim.World
			Fleet  []eveningFleet
			Cars   []struct{ Car *sim.TrafficCar }
			Player struct {
				Bus    *sim.Bus
				Driver DriverSnapshot
			}
			Lines      []*sim.Route
			PlayerLine int
			Lights     *sim.TrafficLightSystem
		}
	}
	if e = json.Unmarshal(raw, &s); e != nil {
		t.Fatal(e)
	}
	// Match Game.LoadGame's derived railways and platform geometry; these
	// private world structures are not included in JSON captures.
	s.Data.World.ConfigureCenterBRT(52)
	sim.StandardizeStopBays(s.Data.World, 52)
	for _, x := range s.Data.Fleet {
		if x.State.Bus.IsArticulated() {
			sim.EnsureLongBusBays(s.Data.World, x.Vehicle.Route)
		}
	}
	var a eveningFleet
	if id == 0 {
		a.State.Bus = s.Data.Player.Bus
		a.State.Driver = s.Data.Player.Driver
		a.Vehicle.Route = s.Data.Lines[s.Data.PlayerLine]
	}
	for _, x := range s.Data.Fleet {
		if x.Vehicle.ID == id {
			a = x
		}
	}
	if a.State.Bus == nil {
		t.Fatal("bus missing")
	}
	b := a.State.Bus
	ad, e := a.State.Driver.Restore(s.Data.World)
	if e != nil {
		t.Fatal(e)
	}
	ad.FleetBus = true
	ad.DisallowOncomingOvertake = true
	f := autopilotFrame{bus: b, world: s.Data.World, tls: s.Data.Lights, laneWidth: 52, s: ad.LastPathS, targetStop: s.Data.World.Stops[a.State.Driver.PathToID], fromStop: s.Data.World.Stops[a.State.Driver.PathFromID], remaining: ad.LastRemaining, distToStop: b.Pos.Distance(s.Data.World.Stops[a.State.Driver.PathToID].Pos), lateralErr: ad.LastLateralErr}
	for _, x := range s.Data.Fleet {
		if x.Vehicle.ID != id {
			f.obstacles = append(f.obstacles, ObserveBus(x.State.Bus))
		}
	}
	if id != 0 {
		f.obstacles = append(f.obstacles, ObserveBus(s.Data.Player.Bus))
	}
	for _, x := range s.Data.Cars {
		c := x.Car
		f.obstacles = append(f.obstacles, VehicleObstacle{Car: c, Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor, YieldingTo: c.YieldingToBus})
	}
	if os.Getenv("BUS_AUTOPILOT_RING_DETAIL") == "1" {
		for _, ob := range f.obstacles {
			if ob.Bus == b || ob.Pos.Distance(b.Pos) > 1000 {
				continue
			}
			one := f
			one.obstacles = []VehicleObstacle{ob}
			if limit := ad.roundaboutEntryDriveSpeed(one); limit < 50 {
				t.Logf("RING LIMIT %.1f ob%v h%.2f v%.1f bus%v car%v", limit, ob.Pos, ob.Heading, ob.Speed, ob.Bus != nil, ob.Car != nil)
			}
		}
	}
	desired, _, goal, _ := ad.pursuitSteering(f)
	if os.Getenv("BUS_AUTOPILOT_JUNCTION_DETAIL") == "1" {
		probe := *ad
		th, br, st, active := probe.junctionLaneRejoinControls(f)
		t.Logf("rejoin %v controls %.2f/%.2f/%.2f mergeYield%v inside%v busHold%v", active, th, br, st, mergeYieldHold(f), f.world.InsideJunction(b.Pos, 52, b.Length/2), b.JunctionCorridor)
		for _, ob := range f.obstacles {
			one := f
			one.obstacles = []VehicleObstacle{ob}
			if ad.junctionSweptSpeed(one) <= 1 {
				t.Logf("JUNCTION BLOCK ob%v h%.3f v%.1f bus%v car%v", ob.Pos, ob.Heading, ob.Speed, ob.Bus != nil, ob.Car != nil)
			}
		}
	}
	probe := *ad
	ctl, dockActive := probe.stationDockControls(f)
	t.Logf("direct dock%+v active%v unreachable%v stop%v remaining%.1f", ctl, dockActive, probe.stationDockUnreachable, f.targetStop.Pos, f.remaining)
	if os.Getenv("BUS_AUTOPILOT_STATION_DETAIL") == "1" {
		stop := f.targetStop
		forward := sim.UnitFromAngle(stop.Heading)
		t.Logf("boarding frontDistance %.3f inBay %v outer %v clear %v capacity %v permission %v wait %.2f",
			stop.FrontBerthPosition().Sub(b.Pos).Dot(forward), stop.ContainsBus(b), stop.ContainsBusInOuterLane(b, f.laneWidth),
			stationBoardingPoseClear(f), ad.stationBoardingCapacity(b, stop, f.obstacles), ad.outerDockAllowed, ad.outerDockWait)
		for _, ob := range f.obstacles {
			if ob.Pos.Distance(b.Pos) < b.OverallLength()+ob.Length &&
				sweepBodiesCollision(b.BodyParts(), ob, 0, 0, 0) {
				id := 0
				if ob.Car != nil {
					id = ob.Car.ID
				}
				t.Logf("PHYSICAL OVERLAP car%d %v heading %.3f speed %.1f", id, ob.Pos, ob.Heading, ob.Speed)
			}
		}
	}
	if req := ad.approachMerge; req != nil {
		t.Logf("merge junction%+v request%+v current%v path%v", f.world.JunctionAt(req.Junction, 52), req, b.Pos, ad.pointAt(f.s))
		if os.Getenv("BUS_AUTOPILOT_JUNCTION_DETAIL") == "1" {
			u := sim.UnitFromAngle(req.Heading)
			r := sim.Vec2{X: -u.Y, Y: u.X}
			g := f.world.JunctionAt(req.Junction, f.laneWidth)
			a := ad.intersectionApproachWithRunway(f, true, false, true)
			if a.approachingInter != nil {
				line := a.approachingInter.GetStopLineDistForHeading(req.Heading, f.laneWidth)
				turn, known := ad.intersectionExitTurn(req.Junction, u.X, u.Y, line, 2.2*f.laneWidth)
				half := math.Abs(u.X)*g.HalfX + math.Abs(u.Y)*g.HalfY
				entry, exitIdx, planned := ad.fallbackJunctionPlan(req.Junction, req.Junction, u, r, turn, half, g)
				actual := b.Pos.Sub(req.Junction).Dot(r)
				t.Logf("fallback distToLine %.1f stopBumper %.1f turn%d known%v entry%.1f exit%d planned%v yieldOrClearing%v",
					a.distToStopLine, approachMergeStopDistance(f, req), turn, known, entry, exitIdx, planned,
					ad.mergeYieldOrClearingActive(f, req.Heading, u, r, actual, entry+ad.LaneOffset))
				for _, ob := range f.obstacles {
					if ob.YieldingTo == b || (ob.Pos.Distance(b.Pos) < 290 && math.Cos(ob.Heading-req.Heading) > .95) {
						t.Logf("merge neighbor pos%v v%.1f yielding%v gap%.1f", ob.Pos, ob.Speed, ob.YieldingTo == b, stoppedFollowerGap(f, ob, req.Heading))
					}
				}
			}
		}
	}
	t.Logf("pose%v h%.3f tail%.3f steer%.3f desired%.3f goal%v boundary%+v hits%v", b.Pos, b.Heading, b.TailHeading(), b.SteeringAngle, desired, goal, b.CheckBoundaries(f.world, 52), b.HitsHardBarrier(f.world, 52))
	for _, part := range b.BodyParts() {
		if part.HalfLen <= 0 {
			continue
		}
		for _, p := range sim.BodyCorners(part) {
			if bound := f.world.CheckBoundaries(p, .5, 52); bound.Zone == sim.ZoneHardBarrier {
				t.Logf("blocked corner%v boundary%+v", p, bound)
			}
		}
	}
	if os.Getenv("BUS_AUTOPILOT_CACHED") == "1" {
		f.world.BeginBoundaryQueries(52)
		defer f.world.EndBoundaryQueries()
	}
	plan, ok := ad.planBarrierUnpinReverse(f, ad.barrierRecoveryYaw(f))
	t.Logf("reverse plan%+v valid%v yaw%.3f following%.1f obstacles%.1f hold%v", plan, ok, ad.barrierRecoveryYaw(f), ad.followingSpeed(f), ad.obstacleSpeed(f, 30), ad.roadSafetyHold)
	empty := f
	empty.obstacles = nil
	for _, st := range []float64{desired, 0, -.3, .3, -.65, .65, -1, 1} {
		copy := *b
		copy.SteeringAngle = st * copy.MaxSteerAngle
		trial := f
		trial.bus = &copy
		t.Logf("steer%.3f sweep%.1f boundarySweep%.1f actualEscape%v settledEscape%v", st, ad.articulatedSweptSpeed(f, st), ad.articulatedSweptSpeed(empty, st, true), ad.stationForwardEscapeClear(f, st, 18, goal, 5), ad.stationForwardEscapeClear(trial, st, 18, goal, 5))
	}
	if steps, err := strconv.Atoi(os.Getenv("BUS_AUTOPILOT_STEPS")); err == nil && steps > 0 {
		for tick := 0; tick < steps; tick++ {
			s.Data.Lights.Update(1. / 60)
			th, br, st, hb, doors, _ := ad.ComputeControls(b, a.Vehicle.Route, f.world, f.tls, f.obstacles, 52, 1./60)
			b.Update(1./60, th, br, st, hb, doors)
			if tick%60 == 0 {
				t.Logf("t%.1f pos%v v%.2f ctl%.2f/%.2f/%.2f headon%+v barrier%+v cooldown%d hold%v target%.1f", float64(tick)/60, b.Pos, b.Speed, th, br, st, ad.headonRecovery, ad.barrierReverse, ad.barrierReverseCooldown, ad.roadSafetyHold, ad.LastTargetSpeed)
				t.Logf("dock%+v unreachable%v outer%v wait%.2f pathRemain%.1f", ad.stationDock, ad.stationDockUnreachable, ad.outerDockAllowed, ad.outerDockWait, ad.LastRemaining)
				t.Logf("approach%+v mid%+v offset%.2f targetOffset%.2f return%+v goal%v", ad.approachMerge, ad.midLaneRecovery, ad.LaneOffset, ad.TargetLaneOffset, ad.pathReturnHold, ad.LastPursuitPt)
			}
		}
	}

}
