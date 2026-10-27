package game

import (
	"bufio"
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"math"
	"os"
	"testing"
)

// Fixed seed, warm-up and frame count keep before/after profiles comparable.
// This measures simulation updates; Draw/GPU work is measured by release-check.
func BenchmarkArticulatedCongestedCity(b *testing.B) {
	sim.SeedSimulation(20261023)
	g := NewGame(1280, 720)
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	for tick := 0; tick < 600; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.ReportAllocs()
	b.ResetTimer()
	for tick := 0; tick < b.N; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.StopTimer()
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
	b.ReportMetric(g.Bus.Durability, "health")
}

// Optional deterministic trajectory trace for profiling changes. No user save
// is read or written. Compare hashes from separate before/after builds.
func TestArticulatedPerformanceTrace(t *testing.T) {
	path := os.Getenv("BUS_PERF_TRACE")
	if path == "" {
		t.Skip("BUS_PERF_TRACE not set")
	}
	sim.SeedSimulation(20261023)
	g := saveTestGame(t)
	if os.Getenv("BUS_PERF_FLEET") == "48" {
		g.SetTransitLineCount(6)
		for i := 0; i < 6; i++ {
			g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
		}
	}
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 40, MaxKPH: 60}})
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	f, err := os.Create(path)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	writer := bufio.NewWriter(f)
	defer writer.Flush()
	for tick := 0; tick < 1800; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		h := sha256.New()
		put := func(values ...float64) {
			for _, v := range values {
				var b [8]byte
				binary.LittleEndian.PutUint64(b[:], math.Float64bits(v))
				h.Write(b[:])
			}
		}
		busState := func(b *sim.Bus, r *sim.Route, d *autopilot.AutopilotDriver) {
			put(b.Pos.X, b.Pos.Y, b.Heading, b.Speed, b.TailHeading(), b.SteeringAngle, b.DoorProgress, b.Durability, float64(b.PassengerCount), float64(r.CurrentIndex), float64(r.TotalTrips), float64(d.State), d.LastTargetSpeed, d.LastPathS, d.LaneOffset)
		}
		busState(g.Bus, g.Route, g.AutopilotDriver)
		for _, ab := range g.AIBusMgr.Buses {
			busState(ab.Bus, ab.Route, ab.Driver)
		}
		for _, c := range g.TrafficMgr.Cars {
			put(float64(c.ID), c.Pos.X, c.Pos.Y, c.Speed, c.Heading, c.TargetSpeed, float64(c.LaneIndex))
			if c.JunctionTurn != nil {
				put(float64(c.JunctionTurn.Segment), c.JunctionTurn.Progress)
			}
		}
		put(float64(g.Cash))
		for _, s := range g.World.Stops {
			put(float64(s.WaitingPassengers))
		}
		fmt.Fprintf(writer, "%d %x\n", tick, h.Sum(nil))
	}
}

func BenchmarkArticulatedMaximumFleet(b *testing.B) {
	sim.SeedSimulation(20261024)
	g := NewGame(1280, 720)
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.TrafficDensity = 4
	g.EnterCity(sim.MapThemeMetropolis)
	g.SetTransitLineCount(6)
	for i := 0; i < 6; i++ {
		g.SetLineFleet(i, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 30, MaxKPH: 80}})
	}
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	for tick := 0; tick < 300; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.ReportAllocs()
	b.ResetTimer()
	for tick := 0; tick < b.N; tick++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.StopTimer()
	var hits, misses uint64
	hits, misses = g.Bus.PosePredictionStats()
	for _, ab := range g.AIBusMgr.Buses {
		h, m := ab.Bus.PosePredictionStats()
		hits, misses = hits+h, misses+m
	}
	if hits+misses > 0 {
		b.ReportMetric(100*float64(hits)/float64(hits+misses), "pose-hit%")
	}
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
	b.ReportMetric(g.Bus.Durability, "health")
}
