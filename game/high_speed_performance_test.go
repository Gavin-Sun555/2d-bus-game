package game

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"math"
	"os"
	"runtime"
	"slices"
	"testing"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

// Full production Update profiling; opt-in, synthetic dispatch, no user saves.
func TestHighSpeedGamePerformance(t *testing.T) {
	out := os.Getenv("BUS_HIGH_OUTPUT")
	if out == "" {
		t.Skip("BUS_HIGH_OUTPUT not set")
	}
	sim.SeedSimulation(5)
	g := NewGame(1280, 720)
	g.Settings.AutopilotEnabled = true
	g.Settings.AutopilotBerserk = true
	g.Settings.AutopilotBerserkMaxSpeed = 120
	g.Settings.TrafficDensity = 4
	g.EnterCity(sim.MapThemeMetropolis)
	g.Settings.IsOpen = false
	g.SetTransitLineCount(1)
	if os.Getenv("BUS_HIGH_LONG_ROUTE") == "1" {
		g.Route.StopIDs = []int{199, 213, 199}
		g.Route.CurrentIndex = 0
	}
	g.SetLineFleet(0, traffic.LineFleetSettings{BusCount: 8, Model: sim.BusArticulated, PlayerModel: sim.BusArticulated, SpeedRange: sim.SpeedRange{MinKPH: 80, MaxKPH: 80}})
	g.Bus.SetModel(sim.BusArticulated)
	g.ResetBusesToRouteStart()
	type sample struct {
		Tick    int
		Group   string
		KPH, MS float64
	}
	samples := make([]sample, 0, 7200)
	hash := sha256.New()
	var b [8]byte
	put := func(v float64) {
		binary.LittleEndian.PutUint64(b[:], math.Float64bits(v))
		hash.Write(b[:])
	}
	var before, after runtime.MemStats
	for tick := 0; tick < 7500; tick++ {
		if tick == 300 && os.Getenv("BUS_HIGH_LANE_SCENE") == "1" {
			g.Route.StopIDs = []int{199, 213, 199}
			g.Route.CurrentIndex = 1
			g.Bus.Pos = sim.Vec2{X: -11000, Y: 5582}
			g.Bus.Heading = 0
			g.Bus.Speed = sim.SpeedKPH(120)
			g.Bus.SteeringAngle = 0
			g.Bus.DoorsOpen = false
			g.Bus.DoorProgress = 0
			g.Bus.ResetArticulation()
			g.AutopilotDriver.PrepareInitialLeg(g.Bus, g.World, g.Route, 52)
			lead := g.AIBusMgr.Buses[0]
			lead.Route.StopIDs = []int{199, 213, 199}
			lead.Route.CurrentIndex = 1
			lead.Bus.Pos = sim.Vec2{X: -10200, Y: 5582}
			lead.Bus.Heading = 0
			lead.Bus.Speed = sim.SpeedKPH(40)
			lead.Bus.SteeringAngle = 0
			lead.Bus.DoorsOpen = false
			lead.Bus.DoorProgress = 0
			lead.Bus.ResetArticulation()
			lead.Driver.PrepareInitialLeg(lead.Bus, g.World, lead.Route, 52)
		}
		if tick == 300 {
			runtime.GC()
			runtime.ReadMemStats(&before)
		}
		speed := g.Bus.Speed * 3.6 / 20
		group := "straight"
		if g.World.InsideJunction(g.Bus.Pos, 52, g.Bus.Length/2) {
			group = "junction"
		}
		if g.AutopilotDriver.OvertakeState == 1 || math.Abs(g.AutopilotDriver.TargetLaneOffset-g.AutopilotDriver.LaneOffset) > .5 {
			group = "lane-change"
		}
		start := time.Now()
		err := g.Update()
		ms := float64(time.Since(start).Nanoseconds()) / 1e6
		if err != nil {
			t.Fatal(err)
		}
		if tick >= 300 {
			samples = append(samples, sample{tick, group, speed, ms})
		}
		busHash := func(b *sim.Bus) {
			put(b.Pos.X)
			put(b.Pos.Y)
			put(b.Heading)
			put(b.TailHeading())
			put(b.Speed)
			put(b.Durability)
		}
		busHash(g.Bus)
		for _, ab := range g.AIBusMgr.Buses {
			busHash(ab.Bus)
		}
		for _, c := range g.TrafficMgr.Cars {
			put(c.Pos.X)
			put(c.Pos.Y)
			put(c.Heading)
			put(c.Speed)
		}
		for _, it := range g.TrafficLights.Intersections {
			put(float64(it.Phase))
			put(it.Timer)
		}
		put(float64(g.Route.CurrentIndex))
		put(float64(g.Bus.PassengerCount))
		put(float64(g.Cash))
	}
	runtime.ReadMemStats(&after)
	type stats struct {
		N                         int
		Mean, P50, P99, P999, Max float64
		Over16, Over33            int
	}
	groups := map[string][]float64{}
	for _, s := range samples {
		groups["all"] = append(groups["all"], s.MS)
		if s.KPH >= 110 && s.KPH <= 120.01 {
			groups["110-120/"+s.Group] = append(groups["110-120/"+s.Group], s.MS)
		}
	}
	summary := map[string]stats{}
	for group, values := range groups {
		s := stats{N: len(values)}
		for _, ms := range values {
			s.Mean += ms
			if ms > 16.667 {
				s.Over16++
			}
			if ms > 33.334 {
				s.Over33++
			}
		}
		s.Mean /= float64(s.N)
		slices.Sort(values)
		s.P50 = values[(s.N-1)/2]
		s.P99 = values[(s.N-1)*99/100]
		s.P999 = values[(s.N-1)*999/1000]
		s.Max = values[s.N-1]
		summary[group] = s
	}
	data, _ := json.MarshalIndent(map[string]any{"groups": summary, "hash": hex.EncodeToString(hash.Sum(nil)), "bytesPerTick": float64(after.TotalAlloc-before.TotalAlloc) / 7200, "allocsPerTick": float64(after.Mallocs-before.Mallocs) / 7200, "health": g.Bus.Durability, "samples": samples}, "", "  ")
	if err := os.WriteFile(out, data, 0644); err != nil {
		t.Fatal(err)
	}
}
