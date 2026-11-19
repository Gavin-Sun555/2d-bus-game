package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/slot3-blocks-20261116.json.gz
var slot3Blocks []byte

func loadSlot3Blocks(t *testing.T) eveningBlocksScene {
	t.Helper()
	z, e := gzip.NewReader(bytes.NewReader(slot3Blocks))
	if e != nil {
		t.Fatal(e)
	}
	defer z.Close()
	var s eveningBlocksScene
	if e = json.NewDecoder(z).Decode(&s); e != nil {
		t.Fatal(e)
	}
	return s
}

func TestSlot3PlayerRejoinsBesideStoppedBus(t *testing.T) {
	s := loadSlot3Blocks(t)
	b := s.Player.Bus
	ad, e := s.Player.Driver.Restore(s.World)
	if e != nil {
		t.Fatal(e)
	}
	route := &sim.Route{StopIDs: []int{2, 4}, CurrentIndex: 1}
	start := b.Pos
	ad.BeforeRecovery = func() { t.Fatal("player relocated") }
	var obs []VehicleObstacle
	for _, a := range s.Fleet {
		if a.State.Bus.Pos.Distance(b.Pos) < 1000 {
			obs = append(obs, ObserveBus(a.State.Bus))
		}
	}
	for _, c := range s.Cars {
		if c.Pos.Distance(b.Pos) < 1000 {
			obs = append(obs, VehicleObstacle{Car: c, Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
		}
	}
	for tick := 0; tick < 60*60; tick++ {
		s.Lights.Update(1. / 60)
		before := b.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, s.World, s.Lights, obs, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.05 {
			t.Fatal("bus jumped")
		}
		if b.HitsHardBarrier(s.World, 52) {
			t.Fatal("hard barrier")
		}
		for _, ob := range obs {
			if sweepBodiesCollision(b.BodyParts(), ob, 0, 0, 0) {
				t.Fatal("hit neighbor")
			}
		}
		if b.Pos.Distance(start) > 250 {
			t.Logf("player cleared in %.2fs pos%v", float64(tick)/60, b.Pos)
			return
		}
	}
	t.Fatalf("player stuck %v heading%.3f rejoin%v merge%+v mid%+v hold%.2f road%v", b.Pos, b.Heading, ad.junctionRejoinActive, ad.approachMerge, ad.midLaneRecovery, ad.busTurnHoldDistance, ad.roadSafetyHold)
}

func TestSlot3FleetClearsStraddledJunctionExit(t *testing.T) {
	s := loadSlot3Blocks(t)
	var a eveningFleet
	for _, x := range s.Fleet {
		if x.Vehicle.ID == 110 {
			a = x
		}
	}
	b := a.State.Bus
	ad, e := a.State.Driver.Restore(s.World)
	if e != nil {
		t.Fatal(e)
	}
	ad.FleetBus = true
	var obs []VehicleObstacle
	for _, x := range s.Fleet {
		if x.Vehicle.ID != 110 && x.State.Bus.Pos.Distance(b.Pos) < 1000 {
			obs = append(obs, ObserveBus(x.State.Bus))
		}
	}
	for _, c := range s.Cars {
		if c.Pos.Distance(b.Pos) < 1000 {
			obs = append(obs, VehicleObstacle{Car: c, Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
		}
	}
	start := b.Pos
	ad.BeforeRecovery = func() { t.Fatal("unexpected fleet replan or relocation") }
	for tick := 0; tick < 60*60; tick++ {
		s.Lights.Update(1. / 60)
		before := b.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(b, a.Vehicle.Route, s.World, s.Lights, obs, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.05 {
			t.Fatal("bus jumped")
		}
		if b.HitsHardBarrier(s.World, 52) {
			t.Fatal("barrier")
		}
		for _, o := range obs {
			if sweepBodiesCollision(b.BodyParts(), o, 0, 0, 0) {
				t.Fatal("hit traffic")
			}
		}
		if b.Pos.Distance(start) > 300 {
			t.Logf("fleet bus cleared in %.2fs", float64(tick)/60)
			return
		}
	}
	t.Fatalf("fleet stuck %v heading%.3f rejoin%v hold%v merge%+v", b.Pos, b.Heading, ad.junctionRejoinActive, ad.roadSafetyHold, ad.approachMerge)
}

func TestRecoveredRoadOriginSurvivesLaneRebuildAndRestore(t *testing.T) {
	s := loadSlot3Blocks(t)
	var a eveningFleet
	for _, x := range s.Fleet {
		if x.Vehicle.ID == 110 {
			a = x
		}
	}
	ad, err := a.State.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	if ad.offRouteOrigin == nil {
		t.Fatal("legacy recovered road prefix not recognized")
	}
	origin := *ad.offRouteOrigin
	for i := 0; i < 2; i++ {
		ad.drivePath = nil
		ad.PrepareLeg(s.World, s.World.Stops[36], s.World.Stops[32], 52)
		if ad.pathOffRouteDistance(a.State.Bus) > 100 {
			t.Fatalf("current road approach dropped: %v", ad.drivePath[:2])
		}
		if ad.pathFromID != 36 || ad.pathToID != 32 || ad.offRouteOrigin == nil || ad.offRouteOrigin.Pos != origin.Pos || ad.offRouteOrigin.Heading != origin.Heading {
			t.Fatal("lost route identity or road origin")
		}
		snap := ad.Snapshot()
		ad, err = snap.Restore(s.World)
		if err != nil {
			t.Fatal(err)
		}
	}
	ad.PrepareLeg(s.World, s.World.Stops[32], s.World.Stops[24], 52)
	if ad.offRouteOrigin != nil {
		t.Fatal("old origin leaked into next leg")
	}
}

func TestJunctionRejoinRespectsCommittedManeuvers(t *testing.T) {
	for _, name := range []string{"doors", "service", "moving", "roundabout", "approach-merge", "lane-recovery", "turn-hold", "docked"} {
		t.Run(name, func(t *testing.T) {
			s := loadSlot3Blocks(t)
			b := s.Player.Bus
			ad, err := s.Player.Driver.Restore(s.World)
			if err != nil {
				t.Fatal(err)
			}
			f := autopilotFrame{bus: b, world: s.World, tls: s.Lights, laneWidth: 52, s: ad.LastPathS}
			switch name {
			case "doors":
				b.DoorsOpen = true
			case "service":
				b.AddPassengerService(2, 0)
			case "moving":
				f.speedAbs = 30
			case "roundabout":
				f.nearRoundabout = true
			case "approach-merge":
				ad.approachMerge = &sim.BusLaneMergeRequest{}
			case "lane-recovery":
				ad.midLaneRecovery = &sim.BusLaneMergeRequest{}
			case "turn-hold":
				ad.busTurnHoldDistance = 10
			case "docked":
				ad.State = AutopilotStateDocked
			}
			if _, _, _, active := ad.junctionLaneRejoinControls(f); active {
				t.Fatal("rejoin overrode committed maneuver")
			}
		})
	}
}

func TestSkippedArrivalReachesDistantPlatform(t *testing.T) {
	s := loadSlot3Blocks(t)
	for _, long := range []bool{false, true} {
		ad := NewAutopilotDriver()
		ad.BerserkMode = true
		ad.CanSkipTargetStop = true
		ad.articulatedPath = long
		ad.PrepareLeg(s.World, s.World.Stops[16], s.World.Stops[36], 52)
		last := ad.drivePath[len(ad.drivePath)-1]
		if math.Abs(last.Y-s.World.Stops[36].Pos.Y) > 1 {
			t.Fatalf("road-only path stopped before target: %v", last)
		}
		// Emulate the old cached path ending immediately after the corner.
		ad.drivePath = ad.drivePath[:len(ad.drivePath)-1]
		ad.cumLen = ad.cumLen[:len(ad.cumLen)-1]
		ad.pathSynced = true
		snap := ad.Snapshot()
		restored, e := snap.Restore(s.World)
		if e != nil {
			t.Fatal(e)
		}
		end := restored.drivePath[len(restored.drivePath)-1]
		if end.Distance(last) > 1 {
			t.Fatalf("cached skipped approach not repaired: %v want%v", end, last)
		}
	}
}
