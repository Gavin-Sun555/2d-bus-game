package game

import (
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestSavedSlot1FourLaneLeftQueue(t *testing.T) {
	path := os.Getenv("BUS_SLOT1_FOUR_LANE_SAVE")
	if path == "" {
		t.Skip("BUS_SLOT1_FOUR_LANE_SAVE not set")
	}
	t.Setenv("BUS_SLOT1_LEFT_EXIT_SAVE", path)
	g := loadSlot1LeftExitReplay(t)
	var leader *sim.Bus
	for _, a := range g.AIBusMgr.Buses {
		if a.ID == 31 {
			leader = a.Bus
		}
	}
	if leader == nil {
		t.Fatal("missing AI31")
	}
	var junction *sim.TrafficIntersection
	for _, it := range g.TrafficLights.Intersections {
		if it.Center == (sim.Vec2{X: 62400, Y: 14400}) {
			junction = it
			break
		}
	}
	if junction == nil {
		t.Fatal("missing South Ring junction")
	}
	stopLine := junction.Center.X - junction.GetStopLineDistForHeading(0, 52)
	green, entry := -1, -1
	parallel := false
	parallelTurnSpeed := 0.
	peakMidTurnCarSpeed := 0.
	minMidTurnSpeed := math.Inf(1)
	leftCars := map[int]bool{}
	start, durability := leader.Pos, leader.Durability
	for tick := 0; tick < 60*90; tick++ {
		before := leader.Pos
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		if leader.Durability < durability || leader.Pos.Distance(before) > math.Abs(leader.Speed)/60+.1 {
			t.Fatalf("contact or reposition t=%.2f pos=%v", float64(tick)/60, leader.Pos)
		}
		leftGreen := junction.GetSignalForHeading(0, true) == sim.SignalGreen
		if green < 0 && leftGreen {
			green = tick
		}
		if entry < 0 {
			if green >= 0 && !leftGreen {
				t.Fatalf("AI31 missed the first left green at %v", leader.Pos)
			}
			nose := leader.Pos.X + math.Cos(leader.Heading)*leader.Length/2
			if nose >= stopLine {
				if !leftGreen {
					t.Fatal("AI31 crossed the stop line before the left green")
				}
				entry = tick
			}
		}
		if leftGreen {
			for _, car := range g.TrafficMgr.Cars {
				turn := car.JunctionTurn
				if turn == nil || turn.LaneChange || turn.Center != junction.Center ||
					math.Cos(turn.EntryHeading) < .98 || math.Sin(turn.ExitHeading) > -.98 {
					continue
				}
				leftCars[car.ID] = true
				if entry >= 0 && car.Speed > 5 && leader.Speed > 5 {
					parallel = true
				}
			}
		}
		for _, car := range g.TrafficMgr.Cars {
			turn := car.JunctionTurn
			if leftCars[car.ID] && turn != nil && !turn.LaneChange && turn.Center == junction.Center &&
				car.Heading < -.3 && car.Heading > -1.25 {
				peakMidTurnCarSpeed = math.Max(peakMidTurnCarSpeed, car.Speed)
				if leader.Heading < -.3 && leader.Heading > -1.25 {
					parallelTurnSpeed = math.Max(parallelTurnSpeed, math.Min(car.Speed, leader.Speed))
				}
			}
		}
		if leader.Heading < -.3 && leader.Heading > -1.25 {
			minMidTurnSpeed = math.Min(minMidTurnSpeed, leader.Speed)
		}
		if leader.Pos.Y < start.Y-750 {
			if !parallel {
				t.Fatalf("no simultaneous left turns on first green; %d cars entered", len(leftCars))
			}
			if tick >= 25*60 || math.IsInf(minMidTurnSpeed, 1) || minMidTurnSpeed < 75 || parallelTurnSpeed < 80 {
				t.Fatalf("slow parallel turn: clear %.2fs, bus mid-turn minimum %.1f, shared mid-turn speed %.1f", float64(tick+1)/60, minMidTurnSpeed, parallelTurnSpeed)
			}
			t.Logf("shared mid-turn speed %.1f", parallelTurnSpeed)
			t.Logf("bus mid-turn minimum %.1f, car peak %.1f", minMidTurnSpeed, peakMidTurnCarSpeed)
			t.Logf("%d small cars turned on the first left green", len(leftCars))
			t.Logf("first green %.2fs, entered %.2fs, cleared %.2fs", float64(green+1)/60, float64(entry+1)/60, float64(tick+1)/60)
			return
		}
	}
	t.Fatalf("AI31 remained in queue at %v", leader.Pos)
}
