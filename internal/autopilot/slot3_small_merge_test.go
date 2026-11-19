package autopilot

import (
	_ "embed"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/slot3-small-merge-20261117.json.gz
var slot3SmallMerge []byte

func TestSlot3SmallMergeResumesFromPhysicalOffset(t *testing.T) {
	s := loadSlot3LaterBlock(t, slot3SmallMerge)
	s.World.ConfigureCenterBRT(52)
	sim.StandardizeStopBays(s.World, 52)
	var a eveningFleet
	for _, x := range s.Fleet {
		if x.Vehicle.ID == 134 {
			a = x
		}
	}
	if a.State.Bus == nil {
		t.Fatal("missing bus")
	}
	b := a.State.Bus
	ad, err := a.State.Driver.Restore(s.World)
	if err != nil {
		t.Fatal(err)
	}
	ad.FleetBus, ad.DisallowOncomingOvertake = true, true
	f := autopilotFrame{bus: b, world: s.World, tls: s.Lights, laneWidth: 52, s: ad.LastPathS}
	for _, c := range s.Cars {
		f.obstacles = append(f.obstacles, VehicleObstacle{Car: c, Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor})
	}
	req := ad.approachMerge
	b.LaneMergeRequest = req
	right := sim.Vec2{X: -math.Sin(req.Heading), Y: math.Cos(req.Heading)}
	actual := b.Pos.Sub(ad.pointAt(f.s)).Dot(right)
	destination := req.Target.Sub(ad.pointAt(f.s)).Dot(right)
	if math.Abs(actual-destination) > 13 {
		t.Fatal("not a small merge")
	}
	if !requiredMergeLaneClear(f, req.Heading, actual, destination) {
		t.Fatal("stationary rear follower reserves completed gap")
	}
	if ad.advanceMidLaneRecovery(f, 1./60) || ad.midLaneRecovery != nil {
		t.Fatal("retreat kept ownership after required gap opened")
	}
	if math.Abs(ad.LaneOffset-actual) > .001 || math.Abs(ad.TargetLaneOffset-actual) > .001 {
		t.Fatal("resumed from old retreat offset")
	}
	for _, ob := range f.obstacles {
		if ob.Car != nil && ob.Car.ID == 2217 {
			ob.Speed = 50
			if stoppedJunctionFollowerClearsMerge(f, ob, req.Heading, actual, destination) {
				t.Fatal("moving rear car ignored")
			}
		}
	}
}
