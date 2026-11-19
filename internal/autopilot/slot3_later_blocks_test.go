package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

//go:embed testdata/slot3-junction-exit-20261116.json.gz
var slot3JunctionExit []byte

//go:embed testdata/slot3-berth-end-20261116.json.gz
var slot3BerthEnd []byte

func loadSlot3LaterBlock(t *testing.T, raw []byte) eveningBlocksScene {
	t.Helper()
	z, e := gzip.NewReader(bytes.NewReader(raw))
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
func TestSlot3ArticulatedWedgeRejectsUncheckedRejoin(t *testing.T) {
	s := loadSlot3LaterBlock(t, slot3JunctionExit)
	for _, a := range s.Fleet {
		if a.Vehicle.ID == 92 {
			ad, e := a.State.Driver.Restore(s.World)
			if e != nil {
				t.Fatal(e)
			}
			f := autopilotFrame{bus: a.State.Bus, world: s.World, tls: s.Lights, laneWidth: 52, s: ad.LastPathS}
			if _, _, _, active := ad.junctionLaneRejoinControls(f); active {
				t.Fatal("compact rejoin overrode articulated sweep")
			}
			return
		}
	}
	t.Fatal("fixture bus missing")
}
func TestSlot3FinishedCompactAlignmentBoards(t *testing.T) {
	for _, unsafe := range []bool{false, true} {
		s := loadSlot3LaterBlock(t, slot3BerthEnd)
		var a eveningFleet
		for _, x := range s.Fleet {
			if x.Vehicle.ID == 107 {
				a = x
			}
		}
		b := a.State.Bus
		ad, e := a.State.Driver.Restore(s.World)
		if e != nil {
			t.Fatal(e)
		}
		f := autopilotFrame{bus: b, world: s.World, tls: s.Lights, targetStop: s.World.Stops[28], laneWidth: 52}
		for _, x := range s.Fleet {
			if x.Vehicle.ID != 107 {
				f.obstacles = append(f.obstacles, ObserveBus(x.State.Bus))
			}
		}
		if unsafe {
			ob := ObserveBus(b)
			ob.Bus = nil
			f.obstacles = append(f.obstacles, ob)
		}
		controls, _ := ad.followStationDockPlan(f, ad.stationDock, true, false)
		if controls.isDocked == unsafe {
			t.Fatalf("boarding safety mismatch unsafe%v docked%v", unsafe, controls.isDocked)
		}
	}
}
func TestPendingExitReleasesOldApproach(t *testing.T) {
	ad, f, _ := loadStoppedSlot(t, 3)
	g := f.world.JunctionAt(*ad.busLaneJunction, 52)
	u := sim.UnitFromAngle(approachRoadHeading(f.bus.Heading))
	reach := g.HalfX
	if g.HalfY > reach {
		reach = g.HalfY
	}
	reach += f.bus.OverallLength() + 100
	f.bus.Pos = ad.busLaneJunction.Add(u.Mul(1.5 * reach))
	ad.trackPath(f.bus.Pos)
	f.s = ad.LastPathS
	ad.exitCruise = &sim.BusLaneMergeRequest{Junction: *ad.busLaneJunction, Heading: approachRoadHeading(f.bus.Heading)}
	ad.clearPassedBusLaneJunction(f)
	if ad.busLaneJunction != nil {
		t.Fatal("pending exit retained incoming merge")
	}
}
