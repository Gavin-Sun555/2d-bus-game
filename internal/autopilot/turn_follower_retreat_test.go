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

//go:embed testdata/turn-follower-lock-20261111.json.gz
var turnFollowerScene []byte

type turnFollowerVehicle struct {
	ID     int
	Bus    *sim.Bus
	Driver DriverSnapshot
	Route  *sim.Route
	ad     *AutopilotDriver
}
type turnFollowerFixture struct {
	World  *sim.World
	Lights *sim.TrafficLightSystem
	Fleet  []turnFollowerVehicle
}

func loadTurnFollowerScene(t *testing.T) *turnFollowerFixture {
	t.Helper()
	gz, err := gzip.NewReader(bytes.NewReader(turnFollowerScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var s turnFollowerFixture
	if err = json.NewDecoder(gz).Decode(&s); err != nil {
		t.Fatal(err)
	}
	for i := range s.Fleet {
		v := &s.Fleet[i]
		v.ad, err = v.Driver.Restore(s.World)
		if err != nil {
			t.Fatal(err)
		}
		v.ad.BeforeRecovery = func() { t.Fatal("recovery relocated bus") }
	}
	return &s
}
func TestTurnFollowerPhysicalRetreatReleasesLeader(t *testing.T) {
	s := loadTurnFollowerScene(t)
	starts := map[int]sim.Vec2{}
	passengers := map[int]int{}
	reversed := false
	for _, v := range s.Fleet {
		starts[v.ID] = v.Bus.Pos
		passengers[v.ID] = v.Bus.PassengerCount
	}
	for tick := 0; tick < 3600; tick++ {
		s.Lights.Update(1. / 60)
		for i := range s.Fleet {
			v := &s.Fleet[i]
			if v.ID == 96 {
				continue
			}
			obs := []VehicleObstacle{}
			for j := range s.Fleet {
				if i != j {
					obs = append(obs, ObserveBus(s.Fleet[j].Bus))
				}
			}
			before := v.Bus.Pos
			th, br, st, hb, doors, _ := v.ad.ComputeControls(v.Bus, v.Route, s.World, s.Lights, obs, 52, 1./60)
			v.Bus.Update(1./60, th, br, st, hb, doors)
			if v.ID == 93 && v.Bus.Speed < 0 {
				reversed = true
			}
			if v.Bus.Pos.Distance(before) > math.Abs(v.Bus.Speed)/60+.05 {
				t.Fatal("recovery relocated bus")
			}
			if v.Bus.PassengerCount != passengers[v.ID] {
				t.Fatal("recovery lost passengers")
			}
			if v.Bus.CheckBoundaries(s.World, 52).Zone == sim.ZoneHardBarrier {
				t.Fatal("recovery crossed hard boundary")
			}
			for _, ob := range obs {
				for _, part := range ob.Bus.BodyParts() {
					if part.HalfLen > 0 {
						if hit, _, depth := v.Bus.CollidesBody(part); hit && depth > 1e-6 {
							t.Fatalf("bus%d hit another bus: %.4f", v.ID, depth)
						}
					}
				}
			}
			if tick == 180 {
				var err error
				v.ad, err = v.ad.Snapshot().Restore(s.World)
				if err != nil {
					t.Fatal(err)
				}
			}
		}
		if tick%600 == 0 {
			for _, v := range s.Fleet {
				if v.ID != 96 {
					t.Logf("tick%d bus%d pos%v speed%.1f head%.2f recovery%+v", tick, v.ID, v.Bus.Pos, v.Bus.Speed, v.Bus.Heading, v.ad.headonRecovery)
				}
			}
		}
		cleared := reversed
		for _, v := range s.Fleet {
			if v.ID != 96 && v.Bus.Pos.Distance(starts[v.ID]) < 500 {
				cleared = false
			}
		}
		if cleared {
			return
		}
	}
	t.Fatal("turning pair remained blocked")
}

func TestTurnFollowerRetreatRetainsSafetyGuards(t *testing.T) {
	for _, name := range []string{"rear occupied", "doors", "service", "moving follower", "moving leader", "straight queue", "leader doors", "no turn", "far junction"} {
		t.Run(name, func(t *testing.T) {
			s := loadTurnFollowerScene(t)
			var follower *turnFollowerVehicle
			var leader *sim.Bus
			for i := range s.Fleet {
				if s.Fleet[i].ID == 93 {
					follower = &s.Fleet[i]
				}
				if s.Fleet[i].ID == 67 {
					leader = s.Fleet[i].Bus
				}
			}
			b, ad := follower.Bus, follower.ad
			f := autopilotFrame{bus: b, world: s.World, tls: s.Lights, laneWidth: 52, s: ad.LastPathS}
			switch name {
			case "rear occupied":
				body := b.TrailerBody()
				f.obstacles = append(f.obstacles, VehicleObstacle{Pos: body.Center.Sub(sim.UnitFromAngle(body.Heading).Mul(body.HalfLen + 20)), Heading: body.Heading, Length: 32, Width: 24})
			case "doors":
				b.DoorsOpen = true
			case "service":
				b.AddPassengerService(1, 1)
			case "moving follower":
				b.Speed = 5
				f.speedAbs = 5
			case "moving leader":
				leader.Speed = 5
			case "straight queue":
				leader.Heading = 0
				leader.ResetArticulation()
			case "leader doors":
				leader.DoorsOpen = true
			case "no turn":
				ad.exitCruise = nil
			case "far junction":
				ad.exitCruise.Junction.X += 2000
			}
			for _, v := range s.Fleet {
				if v.ID != 93 {
					f.obstacles = append(f.obstacles, ObserveBus(v.Bus))
				}
			}
			if d := ad.turnFollowerRetreatDistance(f); d != 0 {
				t.Fatalf("unsafe or unrelated retreat planned: %.1f", d)
			}
		})
	}
}
