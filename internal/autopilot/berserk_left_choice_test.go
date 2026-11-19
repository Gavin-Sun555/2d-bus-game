package autopilot

import (
	"encoding/json"
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func berserkLeftChoiceFixture(h float64) (*AutopilotDriver, autopilotFrame, sim.Vec2, sim.Vec2) {
	w, u, _, _ := wideLeftExitWorld(52, h, true)
	r := sim.Vec2{X: -u.Y, Y: u.X}
	p := u.Mul(-1800).Add(r.Mul(2.5 * 52))
	bus := sim.NewBus(p.X, p.Y, h)
	ad := NewAutopilotDriver()
	ad.BerserkMode = true
	ad.BerserkMaxSpeed = 1000
	ad.PrepareLeg(w, w.Stops[0], w.Stops[1], 52)
	center := sim.Vec2{}
	ad.busLaneJunction = &center
	ad.busLaneTurnChosen = true
	ad.holdLaneOffset(-52)
	ad.trackPath(bus.Pos)
	tls := sim.NewTrafficLightSystem()
	tls.BuildForWorld(w)
	f := autopilotFrame{bus: bus, world: w, tls: tls, laneWidth: 52, s: ad.LastPathS, remaining: 5000, targetStop: w.Stops[1]}
	f.obstacles = []VehicleObstacle{{Pos: bus.Pos.Add(u.Mul(300)).Add(r.Mul(52)), Heading: h, Length: 96, Width: 36}}
	return ad, f, u, r
}

func TestBerserkLeftTurnKeepsFasterLaneAcrossQueueChangesAndRestore(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		t.Run(fmt.Sprint(h), func(t *testing.T) {
			ad, f, u, r := berserkLeftChoiceFixture(h)
			ad.selectBerserkLeftTurn(f)
			if !ad.ordinaryBusLeft || ad.berserkLeftTurnChoice == nil {
				t.Fatal("stopped curb queue overrode empty inner left turn")
			}
			raw, err := json.Marshal(ad.Snapshot())
			if err != nil {
				t.Fatal(err)
			}
			var snapshot DriverSnapshot
			if err := json.Unmarshal(raw, &snapshot); err != nil {
				t.Fatal(err)
			}
			ad, err = snapshot.Restore(f.world)
			if err != nil {
				t.Fatal(err)
			}
			if ad.berserkLeftTurnChoice == nil {
				t.Fatal("save/load lost committed turn lane")
			}
			// The curb queue starts moving and a car joins the inner queue. A
			// feasible turn decision stays fixed instead of reversing a lane change.
			f.obstacles = []VehicleObstacle{{Pos: f.bus.Pos.Add(u.Mul(300)).Add(r.Mul(-104)), Heading: h, Length: 46, Width: 22}}
			for tick := 0; tick < 120; tick++ {
				ad.updateApproachLanePolicies(f, 1./60)
				if !ad.ordinaryBusLeft || ad.berserkLeftTurnChoice == nil || ad.TargetLaneOffset > 1 {
					t.Fatal("queue fluctuation reversed selected left lane")
				}
			}
			ad.resetLegManeuvers()
			if ad.berserkLeftTurnChoice != nil {
				t.Fatal("new leg inherited old turn choice")
			}
		})
	}
}

func TestBerserkLeftTurnAlsoChoosesFasterCurbLane(t *testing.T) {
	for _, speed := range []float64{0, 120} {
		t.Run(fmt.Sprint(speed), func(t *testing.T) {
			ad, f, u, r := berserkLeftChoiceFixture(0)
			f.obstacles = []VehicleObstacle{{Pos: f.bus.Pos.Add(u.Mul(300)).Add(r.Mul(-104)), Heading: 0, Speed: speed, Length: 96, Width: 36}}
			ad.selectBerserkLeftTurn(f)
			if ad.ordinaryBusLeft || ad.berserkLeftTurnChoice == nil {
				t.Fatal("did not keep faster empty curb-left lane")
			}
		})
	}
}

func TestBerserkLeftChoiceDoesNotReplaceIneligibleManeuvers(t *testing.T) {
	for _, name := range []string{"standard", "fleet", "entered junction", "turning body", "fallback", "no signals", "no world", "insufficient runway", "departure escape", "departing bay"} {
		t.Run(name, func(t *testing.T) {
			ad, f, u, _ := berserkLeftChoiceFixture(0)
			switch name {
			case "standard":
				ad.BerserkMode = false
			case "fleet":
				ad.FleetBus = true
			case "entered junction":
				ad.busJunctionEntered = true
			case "turning body":
				f.bus.Heading = .4
			case "fallback":
				ad.fallbackJunction = &sim.Vec2{}
			case "no signals":
				f.tls = nil
			case "no world":
				f.world = nil
			case "departure escape":
				ad.departureEscapeActive = true
			case "departing bay":
				f.bus.Pos = f.world.Stops[0].Pos
			case "insufficient runway":
				f.bus.Pos = f.bus.Pos.Add(u.Mul(1500))
			}
			before := f.bus.Pos
			ad.selectBerserkLeftTurn(f)
			if ad.berserkLeftTurnChoice != nil || ad.ordinaryBusLeft || f.bus.Pos != before {
				t.Fatal("ineligible maneuver was replaced")
			}
		})
	}
}
