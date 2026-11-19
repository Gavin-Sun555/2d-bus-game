package autopilot

import (
	_ "embed"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/slot3-boarding-window-20261117.json.gz
var slot3BoardingWindow []byte

func TestSlot3CompactBridgesBoardingWindowGap(t *testing.T) {
	for _, blocked := range []bool{false, true} {
		t.Run(map[bool]string{false: "clear lane", true: "blocked lane"}[blocked], func(t *testing.T) {
			s := loadSlot3LaterBlock(t, slot3BoardingWindow)
			s.World.ConfigureCenterBRT(52)
			sim.StandardizeStopBays(s.World, 52)
			var a eveningFleet
			var obs []VehicleObstacle
			for _, x := range s.Fleet {
				if x.Vehicle.ID == 98 {
					a = x
				} else {
					obs = append(obs, ObserveBus(x.State.Bus))
				}
			}
			b := a.State.Bus
			ad, err := a.State.Driver.Restore(s.World)
			if err != nil {
				t.Fatal(err)
			}
			stop := s.World.Stops[18]
			if stop.ContainsBusInOuterLane(b, 52) {
				t.Fatal("fixture is not between boarding windows")
			}
			if blocked {
				obs = append(obs, VehicleObstacle{Pos: b.Pos.Add(sim.Vec2{X: 83}), Heading: 0, Length: 46, Width: 22})
			}
			start := b.Pos
			served := false
			ad.BeforeRecovery = func() { t.Fatal("bus recovered instead of serving") }
			for tick := 0; tick < 40*60; tick++ {
				th, br, st, hb, doors, _ := ad.ComputeControls(b, a.Vehicle.Route, s.World, s.Lights, obs, 52, 1./60)
				before := b.Pos
				b.Update(1./60, th, br, st, hb, doors)
				if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.01 || b.HitsHardBarrier(s.World, 52) {
					t.Fatal("bus relocated or crossed barrier")
				}
				for _, ob := range obs {
					if sweepBodiesCollision(b.BodyParts(), ob, 0, 0, 0) {
						t.Fatal("crawl touched traffic")
					}
				}
				if b.DoorsOpen {
					if blocked {
						t.Fatal("blocked crawl reached boarding window")
					}
					if !stop.ContainsBus(b) && !stop.ContainsBusInOuterLane(b, 52) {
						t.Fatal("boarded between windows")
					}
					served = true
				}
				if served && b.Pos.Distance(start) > 300 {
					return
				}
				if blocked && tick == 180 {
					return
				}
			}
			t.Fatal("bus did not serve and leave", b.Pos, served)
		})
	}
}
