package autopilot

import (
	"bytes"
	"compress/gzip"
	_ "embed"
	"encoding/json"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

//go:embed testdata/slot2-station12-wedge-20261116.json.gz
var eveningStationWedge []byte

func TestEveningStationFollowerOpensReverseRoom(t *testing.T) {
	z, e := gzip.NewReader(bytes.NewReader(eveningStationWedge))
	if e != nil {
		t.Fatal(e)
	}
	defer z.Close()
	var s eveningBlocksScene
	if e = json.NewDecoder(z).Decode(&s); e != nil {
		t.Fatal(e)
	}
	drivers := map[int]*AutopilotDriver{}
	for _, a := range s.Fleet {
		if a.Vehicle.ID == 85 || a.Vehicle.ID == 94 {
			ad, e := a.State.Driver.Restore(s.World)
			if e != nil {
				t.Fatal(e)
			}
			ad.FleetBus = true
			ad.BeforeRecovery = func() { t.Fatal("bus relocated") }
			drivers[a.Vehicle.ID] = ad
		}
	}
	for tick := 0; tick < 60*90; tick++ {
		for _, a := range s.Fleet {
			ad := drivers[a.Vehicle.ID]
			if ad == nil {
				continue
			}
			b := a.State.Bus
			var obs []VehicleObstacle
			for _, o := range s.Fleet {
				if o.Vehicle.ID != a.Vehicle.ID && o.State.Bus.Pos.Distance(b.Pos) < 1000 {
					obs = append(obs, ObserveBus(o.State.Bus))
				}
			}
			for _, c := range s.Cars {
				if c.Pos.Distance(b.Pos) < 1000 {
					obs = append(obs, VehicleObstacle{Car: c, Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
				}
			}
			th, br, st, hb, doors, dock := ad.ComputeControls(b, a.Vehicle.Route, s.World, s.Lights, obs, 52, 1./60)
			before := b.Pos
			b.Update(1./60, th, br, st, hb, doors)
			if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.05 {
				t.Fatal("bus jumped")
			}
			if b.CheckBoundaries(s.World, 52).Zone == sim.ZoneHardBarrier {
				t.Fatal("hard boundary")
			}
			for _, ob := range obs {
				if sweepBodiesCollision(b.BodyParts(), ob, 0, 0, 0) {
					t.Fatalf("bus%d collision", a.Vehicle.ID)
				}
			}
			if a.Vehicle.ID == 94 && dock && b.DoorsOpen {
				t.Logf("bus94 boarded %.2fs", float64(tick)/60)
				return
			}
		}
	}
	t.Fatal("station arrival remains blocked")
}
