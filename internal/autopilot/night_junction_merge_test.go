package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"testing"
)

//go:embed testdata/slot2-night-junction-20261116.json.gz
var nightJunctionScene []byte

func loadNightJunction(t *testing.T) eveningBlocksScene {
	t.Helper()
	z, e := gzip.NewReader(bytes.NewReader(nightJunctionScene))
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

func TestNightStraightJunctionMergeClearsBentRear(t *testing.T) {
	s := loadNightJunction(t)
	var lead, rear eveningFleet
	for _, a := range s.Fleet {
		if a.Vehicle.ID == 128 {
			lead = a
		}
		if a.Vehicle.ID == 160 {
			rear = a
		}
	}
	b := lead.State.Bus
	b.LaneMergeRequest = lead.State.Driver.ApproachMerge
	req := b.LaneMergeRequest
	f := autopilotFrame{bus: b, world: s.World, tls: s.Lights, laneWidth: 52}
	ob := ObserveBus(rear.State.Bus)
	f.obstacles = []VehicleObstacle{ob}
	right := sim.Vec2{X: -sim.UnitFromAngle(req.Heading).Y, Y: sim.UnitFromAngle(req.Heading).X}
	current := b.Pos.Sub(req.Target).Dot(right)
	t.Logf("physical gap %.2f, correction %.2f", stoppedFollowerGap(f, ob, req.Heading), current)
	if !requiredMergeLaneClear(f, req.Heading, current, 0) {
		t.Fatal("clear straight approach still waits for rear bus")
	}
	for _, name := range []string{"moving rear", "overlap", "wide move", "completed", "no intent", "opposing", "away from junction"} {
		t.Run(name, func(t *testing.T) {
			front, back := *b, *rear.State.Bus
			request := *req
			front.LaneMergeRequest = &request
			one := f
			one.bus = &front
			other := ObserveBus(&back)
			cur := current
			switch name {
			case "moving rear":
				other.Speed = 10
			case "overlap":
				back.Pos = front.Pos
				other = ObserveBus(&back)
			case "wide move":
				cur = 52
			case "completed":
				request.Joined = true
			case "no intent":
				front.LaneMergeRequest = nil
			case "opposing":
				back.Heading = req.Heading + 3.141592653589793
				back.ResetArticulation()
				other = ObserveBus(&back)
			case "away from junction":
				front.Pos.X -= 3000
				back.Pos.X -= 3000
				other = ObserveBus(&back)
			}
			if stoppedJunctionFollowerClearsMerge(one, other, req.Heading, cur, 0) {
				t.Fatal("unsafe clearance")
			}
		})
	}

}

func TestNightRetreatRepublishesMergeBeforeGapCheck(t *testing.T) {
	s := loadNightJunction(t)
	var lead, rear eveningFleet
	for _, a := range s.Fleet {
		if a.Vehicle.ID == 128 {
			lead = a
		}
		if a.Vehicle.ID == 160 {
			rear = a
		}
	}
	ad, e := lead.State.Driver.Restore(s.World)
	if e != nil {
		t.Fatal(e)
	}
	b := lead.State.Bus
	b.LaneMergeRequest = nil
	f := autopilotFrame{bus: b, world: s.World, tls: s.Lights, laneWidth: 52, s: ad.LastPathS, obstacles: []VehicleObstacle{ObserveBus(rear.State.Bus)}}
	if ad.midLaneRecovery == nil {
		t.Fatal("fixture requires pending retreat")
	}
	if ad.advanceMidLaneRecovery(f, 1./60) || ad.midLaneRecovery != nil || ad.busTurnHoldDistance != 0 {
		t.Fatal("open approach remained in retreat/stop hold")
	}
	if b.LaneMergeRequest != ad.approachMerge {
		t.Fatal("pending merge was not published")
	}
}
