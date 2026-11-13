package autopilot

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// Slot 1's AI15 cleared the shelter with its tail but still faced the curb.
// The missed-stop crawl must remain armed until the powered section is aligned.
func TestMissedStationTaperBacksAndRejoinsWithoutReset(t *testing.T) {
	w := &sim.World{Roads: []sim.RoadSegment{{P1: sim.Vec2{X: 62400, Y: 8000}, P2: sim.Vec2{X: 62400, Y: 21600}, Lanes: 8}}}
	for id, y := range []float64{18000, 10800} {
		w.Stops = append(w.Stops, sim.NewBusStop(id, "stop", sim.Vec2{X: 62400, Y: y}, -math.Pi/2, 8, sim.RGBA(0, 0, 0, 255), 52))
	}
	b := sim.NewBus(62601.5083949308, 17822.094634570643, -.5663689354265389)
	b.SetModel(sim.BusArticulated)
	b.TrailerHeading = -1.3463
	other := sim.NewBus(62628.09438234965, 17921.850856015855, -1.5775678795782373)
	other.SetModel(sim.BusArticulated)
	other.TrailerHeading = -1.5356
	obs := []VehicleObstacle{ObserveBus(other)}
	ad := NewAutopilotDriver()
	ad.ContinuePastSkippedStop(b, w.Stops[0], 52)
	route := &sim.Route{StopIDs: []int{0, 1}, CurrentIndex: 1}
	b.BoardPassengers(2, []int{1})
	count := b.PassengerCount
	start := b.Pos
	reversed := false
	for tick := 0; tick < 600; tick++ {
		before := b.Pos
		th, br, st, hb, doors, _ := ad.ComputeControls(b, route, w, nil, obs, 52, 1./60)
		b.Update(1./60, th, br, st, hb, doors)
		reversed = reversed || b.Speed < 0
		if b.Pos.Distance(before) > math.Abs(b.Speed)/60+.001 {
			t.Fatal("recovery relocated bus")
		}
		if b.CheckBoundaries(w, 52).Zone == sim.ZoneHardBarrier {
			t.Fatal("recovery crossed boundary")
		}
		for _, part := range other.BodyParts() {
			if part.HalfLen > 0 {
				if hit, _, _ := b.CollidesBody(part); hit {
					t.Fatal("recovery hit waiting follower")
				}
			}
		}
		if b.PassengerCount != count {
			t.Fatal("recovery lost passengers")
		}
		if tick == 30 {
			var err error
			ad, err = ad.Snapshot().Restore(w)
			if err != nil {
				t.Fatal(err)
			}
		}
	}
	if !reversed || b.Pos.Distance(start) < 200 {
		t.Fatalf("taper remained blocked: reverse=%v travel=%.1f", reversed, b.Pos.Distance(start))
	}
}
