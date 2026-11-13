package autopilot

import (
	_ "embed"
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

//go:embed testdata/red-queue-ai36-2026-11-08.json
var redQueueAI36Fixture []byte

func TestSavedAI36RedLightDoesNotTriggerLaneRecovery(t *testing.T) {
	var x struct {
		rearDepartureFixture
		Cars []*sim.TrafficCar
	}
	if err := json.Unmarshal(redQueueAI36Fixture, &x); err != nil {
		t.Fatal(err)
	}
	ad, err := x.Driver.Restore(x.World)
	if err != nil {
		t.Fatal(err)
	}
	var obs []VehicleObstacle
	for _, c := range x.Cars {
		obs = append(obs, VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width})
	}
	for _, b := range x.OtherBuses {
		obs = append(obs, ObserveBus(b))
	}
	start, health := x.Bus.Pos, x.Bus.Durability
	// This capture was emitted immediately before the old recovery's ten-second
	// deadline. Preserve that live timer rather than hiding it through Restore.
	ad.strandedMedianWait = 10
	ad.strandedMedianStart = start
	ad.BeforeRecovery = func() { t.Fatal("legally waiting bus was repositioned") }
	for tick := 0; tick < 3600; tick++ {
		x.Lights.Update(1. / 60)
		before := x.Bus.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(x.Bus, x.Route, x.World, x.Lights, obs, 52, 1./60)
		if ad.RecoverStrandedMedianApproach(x.Bus, x.World, obs, 52, 1./60, x.Lights) {
			t.Fatal("unnecessary lane recovery")
		}
		x.Bus.Update(1./60, th, br, st, hb, doors)
		if x.Bus.Pos.Distance(before) > math.Abs(x.Bus.Speed)/60+.05 || x.Bus.Durability < health {
			t.Fatal("relocation or damage")
		}
		for _, ob := range obs {
			if sweepBodiesCollision(x.Bus.BodyParts(), ob, 0, 0, 0) {
				t.Fatalf("touched queued traffic tick%d", tick)
			}
		}
		if x.Bus.Pos.Distance(start) > 1000 {
			t.Logf("departed naturally in %.2fs", float64(tick+1)/60)
			return
		}
	}
	t.Fatal("bus did not leave after signal changed")
}
