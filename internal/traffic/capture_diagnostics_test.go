package traffic

import (
	"encoding/json"
	"math"
	"os"
	"strconv"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Optional read-only diagnostic for an isolated captured save. Never changes
// the source file or grants a fresh timeout to its vehicles.
func TestTrafficCaptureReverseDiagnostics(t *testing.T) {
	path := os.Getenv("BUS_TRAFFIC_CAPTURE")
	if path == "" {
		t.Skip("BUS_TRAFFIC_CAPTURE not set")
	}
	id, e := strconv.Atoi(os.Getenv("BUS_TRAFFIC_CAR"))
	if e != nil {
		t.Fatal(e)
	}
	raw, e := os.ReadFile(path)
	if e != nil {
		t.Fatal(e)
	}
	var s struct {
		Data struct {
			World *sim.World
			Cars  []struct{ Car *sim.TrafficCar }
			Fleet []struct {
				Vehicle struct {
					ID  int
					Bus *sim.Bus
				}
				State struct{ Bus *sim.Bus }
			}
			Player struct{ Bus *sim.Bus }
			Lights *sim.TrafficLightSystem
		}
	}
	if e = json.Unmarshal(raw, &s); e != nil {
		t.Fatal(e)
	}
	s.Data.World.ConfigureCenterBRT(52)
	sim.StandardizeStopBays(s.Data.World, 52)
	tm := &TrafficManager{AIBusMgr: NewAIBusManager()}
	f := carFrame{world: s.Data.World, tls: s.Data.Lights, laneWidth: 52}
	for _, x := range s.Data.Cars {
		tm.Cars = append(tm.Cars, x.Car)
		if x.Car.ID == id {
			f.car = x.Car
		}
	}
	if f.car == nil {
		t.Fatal("car not found")
	}
	busIDs := []int{}
	for _, x := range s.Data.Fleet {
		if x.State.Bus != nil {
			f.buses = append(f.buses, x.State.Bus)
			busIDs = append(busIDs, x.Vehicle.ID)
			tm.AIBusMgr.Buses = append(tm.AIBusMgr.Buses, &AIBus{ID: x.Vehicle.ID, Bus: x.State.Bus})
		}
	}
	f.buses = append(f.buses, s.Data.Player.Bus)
	busIDs = append(busIDs, 0)
	f.parkedBuses = make([]bool, len(f.buses))
	for i, b := range f.buses {
		if b != nil && b.Pos.Distance(f.car.Pos) <= 450 {
			t.Logf("NEAR BUS %d model=%d pos=%v h=%.3f spd=%.2f doors=%v stReq=%v lnReq=%v jCorr=%v carPathClear=%v",
				busIDs[i], b.Model, b.Pos, b.Heading, b.Speed, b.DoorsOpen, b.StationMergeRequest != nil, b.LaneMergeRequest != nil, b.JunctionCorridor != nil,
				carPathClearsBus(f.car, b, .25))
		}
	}
	f.activeCount = len(tm.Cars)
	if os.Getenv("BUS_TRAFFIC_JUNCTION_DETAIL") == "1" {
		logCaptureJunctionBlockers(t, f, tm.Cars)
	}
	if os.Getenv("BUS_TRAFFIC_PASS_DETAIL") == "1" && f.car.JunctionTurn != nil {
		t.Logf("LANE CHANGE traveled %.2f remaining %.2f conflict %.2f",
			carTurnHistory(f.car.JunctionTurn), carTurnRemaining(f.car.JunctionTurn), tm.carLaneChangeConflict(f, carTurnRemaining(f.car.JunctionTurn)+60, false))
		for i, bus := range f.buses {
			if bus != nil && bus.Pos.Distance(f.car.Pos) < 350 {
				t.Logf("LANE CHANGE bus%d checked path clear %v", busIDs[i], carPathClearsBus(f.car, bus, 0))
			}
		}
	}
	for idx, c := range tm.Cars {
		if c.Pos.Distance(f.car.Pos) > 400 {
			continue
		}
		one := f
		one.car = c
		one.index = idx
		rear, blocker, _ := tm.availableCarReverseDistanceWithBlocker(one, 60)
		blockerID := -1
		if blocker != nil {
			blockerID = blocker.ID
		}
		fl := tm.prepareForcedLeftApproach(one)
		sig := tm.signalSpeed(one, 100)
		jq := tm.junctionQueueSpeed(one, 100)
		bmy := tm.busMergeYieldSpeed(one, 100)
		bf := tm.busFollowingSpeed(one, 100)
		cf := tm.carFollowingSpeed(one, 100)
		iy := tm.intersectionYieldSpeed(one, 100)
		rb := tm.roundaboutSpeed(one, 100)
		jamWho := -1
		for j, oc := range tm.Cars {
			if j == idx || oc.Speed >= 3 || oc.Speed <= -3 {
				continue
			}
			dist := c.Pos.Distance(oc.Pos)
			if dist < (c.Length+oc.Length)*0.5+35.0 && carDeadlockCorridor(c, oc) {
				jamWho = oc.ID
				break
			}
		}
		t.Logf("car%d pos%v h%.3f lane%d spd%.2f fl%.1f sig%.1f jq%.1f bmy%.1f bf%.1f cf%.1f iy%.1f rb%.1f jamCar%d rear%.1f blk%d timer%.1f turn%v",
			c.ID, c.Pos, c.Heading, c.LaneIndex, c.Speed, fl, sig, jq, bmy, bf, cf, iy, rb, jamWho, rear, blockerID, c.DeadlockTimer, c.JunctionTurn != nil)
	}
}

func logCaptureJunctionBlockers(t *testing.T, f carFrame, cars []*sim.TrafficCar) {
	u := sim.UnitFromAngle(f.car.Heading)
	for _, inter := range f.nearbyJunctions() {
		delta := inter.Center.Sub(f.car.Pos)
		line, halfWidth := inter.ApproachGeometry(u, f.laneWidth)
		gap := delta.Dot(u) - line - f.car.Length/2
		if gap < -6 || gap > 180 || math.Abs(delta.X*u.Y-delta.Y*u.X) > halfWidth {
			continue
		}
		plan := planCarJunctionTurn(f, inter)
		if plan == nil {
			t.Logf("JUNCTION %v no turn plan gap %.1f", inter.Center, gap)
			continue
		}
		bodies := carJunctionBodies(f, inter)
		geometry := f.world.JunctionAt(inter.Center, f.laneWidth)
		exit := sim.PrepareJunctionExitQueue(bodies, inter.Center, geometry, f.car.Length, f.car.Width)
		t.Logf("JUNCTION %v gap %.1f exit heading %.3f lane %d end %v", inter.Center, gap, plan.ExitHeading, plan.ExitLane, bodies[len(bodies)-1].Center)
		for i, other := range cars {
			if other == f.car || other.Pos.Distance(inter.Center) > 800 {
				continue
			}
			otherBody := sim.OBB{Center: other.Pos, Heading: other.Heading, HalfLen: other.Length / 2, HalfWid: other.Width / 2}
			if math.Abs(other.Speed) <= 5 && exit.Blocks(otherBody) {
				t.Logf("BLOCKED EXIT car%d %v heading %.3f", other.ID, other.Pos, other.Heading)
			}
			if !otherHasJunctionPriority(f, inter, other, i, gap) {
				continue
			}
			otherFrame := f
			otherFrame.car = other
			carJunctionBodies(otherFrame, inter)
			if !sim.SeparatedJunctionCorridors(f.car.JunctionCorridor, other.JunctionCorridor) {
				t.Logf("PRIORITY car%d %v heading %.3f speed %.1f committed %v", other.ID, other.Pos, other.Heading, other.Speed, other.JunctionTurn != nil)
			}
		}
	}
}
