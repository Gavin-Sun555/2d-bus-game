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

//go:embed testdata/rear-berth-cruise-bus83-20261111.json.gz
var rearBerthCruiseScene []byte

func loadRearBerthCruise(t *testing.T) (*AutopilotDriver, autopilotFrame, *sim.Route) {
	t.Helper()
	gz, err := gzip.NewReader(bytes.NewReader(rearBerthCruiseScene))
	if err != nil {
		t.Fatal(err)
	}
	defer gz.Close()
	var scene rearDepartureFixture
	if err = json.NewDecoder(gz).Decode(&scene); err != nil {
		t.Fatal(err)
	}
	ad, err := scene.Driver.Restore(scene.World)
	if err != nil {
		t.Fatal(err)
	}
	f := autopilotFrame{bus: scene.Bus, world: scene.World, tls: scene.Lights, laneWidth: 52, s: ad.LastPathS, remaining: ad.LastRemaining, targetStop: scene.World.Stops[scene.Route.GetCurrentTargetStopID()]}
	for _, b := range scene.OtherBuses {
		f.obstacles = append(f.obstacles, ObserveBus(b))
	}
	return ad, f, scene.Route
}

func TestRearBerthDepartureRemainsEligibleWhileCruising(t *testing.T) {
	ad, f, route := loadRearBerthCruise(t)
	if ad.State != AutopilotStateCruising || f.bus.IsArticulated() || ad.dockDepartureRun <= f.s {
		t.Fatal("fixture lost ongoing ordinary-bus departure")
	}
	start, count := f.bus.Pos, f.bus.PassengerCount
	ad.BeforeRecovery = func() { t.Fatal("recovery repositioned bus") }
	for tick := 0; tick < 1800; tick++ {
		before := f.bus.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(f.bus, route, f.world, f.tls, f.obstacles, 52, 1./60)
		f.bus.Update(1./60, th, br, st, hb, doors)
		if f.bus.Pos.Distance(before) > math.Abs(f.bus.Speed)/60+.05 || f.bus.PassengerCount != count {
			t.Fatal("departure repositioned bus or lost passengers")
		}
		if f.bus.CheckBoundaries(f.world, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("departure crossed hard boundary")
		}
		for _, ob := range f.obstacles {
			for _, body := range ob.Bus.BodyParts() {
				if body.HalfLen > 0 {
					if hit, _, depth := f.bus.CollidesBody(body); hit && depth > 1e-6 {
						t.Fatalf("departure hit another bus at %.2fs depth%.4f", float64(tick)/60, depth)
					}
				}
			}
		}
		if tick == 60 {
			var err error
			ad, err = ad.Snapshot().Restore(f.world)
			if err != nil {
				t.Fatal(err)
			}
		}
		if f.bus.Pos.Distance(start) > 500 {
			return
		}
	}
	t.Fatalf("rear berth departure still blocked at %v", f.bus.Pos)
}

func TestRearBerthCruiseEscapeRetainsSafetyGuards(t *testing.T) {
	for _, name := range []string{"doors", "service", "moving", "no departure prefix", "moving neighbor", "actual body overlap"} {
		t.Run(name, func(t *testing.T) {
			ad, f, _ := loadRearBerthCruise(t)
			switch name {
			case "doors":
				f.bus.DoorsOpen = true
			case "service":
				f.bus.AddPassengerService(1, 1)
			case "moving":
				f.bus.Speed = 40
				f.speedAbs = 40
			case "no departure prefix":
				ad.dockDepartureRun = 0
			case "moving neighbor", "actual body overlap":
				for i := range f.obstacles {
					ob := &f.obstacles[i]
					if ob.Bus.Pos.Distance(f.bus.Pos) < 70 {
						if name == "moving neighbor" {
							ob.Speed = 2
							ob.Bus.Speed = 2
						} else {
							ob.Bus.Pos.X += 2
							*ob = ObserveBus(ob.Bus)
						}
					}
				}
			}
			if _, clear := ad.stationDepartureEscape(f, -1); clear || ad.departurePresteerValid {
				t.Fatal("unsafe or unrelated departure crawl accepted")
			}
		})
	}
}
