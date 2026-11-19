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

//go:embed testdata/slot2-night-station20-20261116.json.gz
var nightStation20 []byte

func TestNightStationArrivalYieldsToPinnedDeparture(t *testing.T) {
	z, e := gzip.NewReader(bytes.NewReader(nightStation20))
	if e != nil {
		t.Fatal(e)
	}
	defer z.Close()
	var s eveningBlocksScene
	if e = json.NewDecoder(z).Decode(&s); e != nil {
		t.Fatal(e)
	}
	drivers := map[int]*AutopilotDriver{}
	start := sim.Vec2{}
	for _, a := range s.Fleet {
		if a.Vehicle.ID == 161 || a.Vehicle.ID == 164 {
			ad, e := a.State.Driver.Restore(s.World)
			if e != nil {
				t.Fatal(e)
			}
			ad.FleetBus = true
			ad.BeforeRecovery = func() { t.Fatal("bus relocated") }
			drivers[a.Vehicle.ID] = ad
		}
		if a.Vehicle.ID == 164 {
			start = a.State.Bus.Pos
		}
	}
	// This controller-only fixture has static surrounding traffic; the game
	// regression covers complete departures with live service and traffic.
	departureClear, arrivalBoarded := false, false
	for tick := 0; tick < 60*30; tick++ {
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
				t.Fatal("position jumped")
			}
			if b.HitsHardBarrier(s.World, 52) {
				t.Fatal("hard barrier")
			}
			for _, o := range obs {
				if sweepBodiesCollision(b.BodyParts(), o, 0, 0, 0) {
					t.Fatalf("bus%d collision", a.Vehicle.ID)
				}
			}
			if a.Vehicle.ID == 164 && b.Pos.Distance(start) > 10 {
				departureClear = true
			}
			if a.Vehicle.ID == 161 && dock && b.DoorsOpen {
				arrivalBoarded = true
			}
		}
		if departureClear && arrivalBoarded {
			t.Logf("departure started and arrival boarded in %.2fs", float64(tick)/60)
			return
		}
	}
	for _, a := range s.Fleet {
		if a.Vehicle.ID == 164 {
			t.Logf("departure final%v displacement%.2f", a.State.Bus.Pos, a.State.Bus.Pos.Distance(start))
		}
	}
	t.Fatalf("station remains blocked: departureClear%v arrivalBoarded%v", departureClear, arrivalBoarded)
}
