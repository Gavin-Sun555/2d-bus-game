package game

import (
	"fmt"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Replay 2026-11-12 slot 1 congestion captures. Articulated fleet buses on
// ordinary six-lane roads followed the bare taper toward the inner left-turn
// lane, put their front beside a car there and could never swing the trailer
// past it, stalling the whole inner-lane queue. Put the bus back in the curb
// lane before its wedge point: it must request the merge, wait in its own
// lane for a real gap, and complete the left turn without contact.
func TestSavedSlot1WideRoadLeftMergeDoesNotWedge(t *testing.T) {
	for _, tc := range []struct {
		capture string
		bus     int
		wedge   sim.Vec2
		done    func(sim.Vec2) bool
	}{
		{"congestion-20261112-084510.830653000-000001-car173699.json", 87, sim.Vec2{X: 30508, Y: 28878}, func(p sim.Vec2) bool { return p.X > 31300 || p.Y < 28500 }},
		{"congestion-20261112-090645.602152000-000005-car177219.json", 34, sim.Vec2{X: -31884, Y: 28878}, func(p sim.Vec2) bool { return p.X > -31100 || p.Y < 28500 }},
	} {
		// From farther back, bus 34 can end up waiting a long time in its own
		// lane beside the stationary left-turn queue; that is a wait, not a wedge.
		for _, back := range []float64{250, 450} {
			t.Run(fmt.Sprintf("bus%d/back%.0f", tc.bus, back), func(t *testing.T) {
				p := firstFixture(filepath.Join("..", "dist", "debug-cars-2026-11-12", tc.capture))
				if p == "" {
					t.Skip("2026-11-12 car congestion capture not found")
				}
				g := loadSaveForTest(t, p, 1280, 720)
				sim.SeedSimulation(3)
				var bus *traffic.AIBus
				for _, ab := range g.AIBusMgr.Buses {
					if ab.ID == tc.bus {
						bus = ab
					}
				}
				if bus == nil {
					t.Fatalf("capture is missing bus %d", tc.bus)
				}
				bus.Bus.Pos = tc.wedge.Sub(sim.Vec2{X: back})
				bus.Bus.Heading, bus.Bus.SteeringAngle, bus.Bus.Speed = 0, 0, 80
				bus.Bus.ResetArticulation()
				bus.Bus.JunctionCorridor = nil
				bus.Driver.Reset()
				health := bus.Bus.Durability
				for tick := 0; tick < 60*90; tick++ {
					if err := g.Update(); err != nil {
						t.Fatal(err)
					}
					if bus.Bus.Durability < health {
						t.Fatalf("bus %d damaged at tick %d", tc.bus, tick)
					}
					if tc.done(bus.Bus.Pos) {
						return
					}
				}
				t.Fatalf("bus %d did not complete the left turn: pos%v heading%.2f speed%.1f", tc.bus, bus.Bus.Pos, bus.Bus.Heading, bus.Bus.Speed)
			})
		}
	}
}

// Replay the 2026-11-12 bus 39 capture. Two articulated buses of one line
// queued for the same inner left-turn lane: bus 39 waited in the middle lane
// for its merge while bus 38 behind it had already angled into that lane.
// Bus 38 granted the rear slot, then dropped it once it was moving while
// still angled and crawled beside bus 39's trailer, so each waited on the
// other. The follower must keep the slot until the leader has merged.
func TestSavedSlot1QueuedFleetLeftMergeDoesNotInterlock(t *testing.T) {
	p := firstFixture(filepath.Join("..", "dist", "debug-2026-11-12-pm", "congestion-20261112-105713.557696000-000010-bus39.json"))
	if p == "" {
		t.Skip("2026-11-12 bus 39 congestion capture not found")
	}
	g := loadSaveForTest(t, p, 1280, 720)
	buses := map[int]*traffic.AIBus{}
	for _, ab := range g.AIBusMgr.Buses {
		if ab.ID == 38 || ab.ID == 39 {
			buses[ab.ID] = ab
		}
	}
	if len(buses) != 2 {
		t.Fatal("capture is missing bus 38 or 39")
	}
	health := map[int]float64{38: buses[38].Bus.Durability, 39: buses[39].Bus.Durability}
	turned := map[int]bool{}
	for tick := 0; tick < 60*60 && len(turned) < 2; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		for id, ab := range buses {
			if ab.Bus.Durability < health[id] {
				t.Fatalf("bus %d damaged at tick %d", id, tick)
			}
			if ab.Bus.Pos.Y < 28500 {
				turned[id] = true
			}
		}
	}
	for id, ab := range buses {
		if !turned[id] {
			t.Errorf("bus %d did not complete the left turn: pos%v heading%.2f speed%.1f", id, ab.Bus.Pos, ab.Bus.Heading, ab.Bus.Speed)
		}
	}
}
