package game

import (
	"fmt"
	"math"
	"testing"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
)

func presentationTestGame() *Game {
	b := sim.NewBus(0, 0, 0)
	b.Speed = 1000
	c := &sim.TrafficCar{ID: 1, Speed: 500, Length: 46, Width: 22}
	ai := sim.NewBus(200, 100, 0)
	ai.Speed = 800
	return &Game{State: StatePlaying, World: &sim.World{}, Bus: b, Camera: sim.NewCamera(1280, 720), Settings: &ui.Settings{TrafficDensity: 4}, TrafficMgr: &traffic.TrafficManager{Cars: []*sim.TrafficCar{c}}, AIBusMgr: &traffic.AIBusManager{Buses: []*traffic.AIBus{{ID: 1, Bus: ai}}}}
}

func TestPresentationRunsEveryCatchUpUpdate(t *testing.T) {
	g := saveTestGame(t)
	g.State = StatePlaying
	g.Settings.IsOpen = false
	wrapper := WithGraphicsWarmup(g)
	wrapper.ready = true
	before := g.World.Clock.ElapsedMinutes
	for i := 0; i < 20; i++ {
		if err := wrapper.Update(); err != nil {
			t.Fatal(err)
		}
	}
	if delta := g.World.Clock.ElapsedMinutes - before; math.Abs(delta-20./3600) > 1e-9 {
		t.Fatalf("catch-up lost simulation time: %.9f minutes", delta)
	}
}

func TestPresentationKeepsConstantSpeedAcrossDisplayRates(t *testing.T) {
	for _, fps := range []int{30, 50, 60, 90, 120, 144} {
		for _, phase := range []float64{-.45, 0, .45} {
			t.Run(fmt.Sprintf("%dhz/phase%.2f", fps, phase), func(t *testing.T) {
				g := presentationTestGame()
				var p presentationFrames
				var clock presentationClock
				p.capture(g, 0)
				origin := time.Unix(1, 0)
				tick := 0
				lastPos, lastTime := 0., time.Duration(0)
				for frame := 0; frame < fps*2; frame++ {
					elapsed := time.Duration(float64(frame) / float64(fps) * float64(time.Second))
					needed := max(1, int(math.Round(float64(elapsed)/float64(simulationFrame)+phase))+1)
					for tick < needed {
						at := clock.advance(origin.Add(time.Duration(tick) * simulationFrame))
						tick++
						seconds := at.Seconds()
						g.Bus.Pos.X = 1000 * seconds
						g.Camera.Pos.X = g.Bus.Pos.X
						g.AIBusMgr.Buses[0].Bus.Pos.X = 200 + 800*seconds
						g.TrafficMgr.Cars[0].Pos.X = 500 * seconds
						p.capture(g, at)
					}
					// Include varying simulation/draw preparation costs, as on a real frame.
					drawTime := elapsed + time.Duration(2+frame%5)*time.Millisecond
					if !p.prepare(g, clock.displayTime(origin.Add(drawTime))) {
						t.Fatal("presentation unavailable")
					}
					if frame > fps/4 {
						want := 1000 * (drawTime - lastTime).Seconds()
						if math.Abs((p.player.Pos.X-lastPos)-want) > 1e-5 {
							t.Fatalf("display motion disagrees with 180 km/h: delta%.8f want%.8f", p.player.Pos.X-lastPos, want)
						}
						if math.Abs(p.camera.Pos.X-p.player.Pos.X) > 1e-6 || p.player.Speed != 1000 {
							t.Fatal("camera, vehicle and speed disagree")
						}
					}
					lastPos, lastTime = p.player.Pos.X, drawTime
				}
			})
		}
	}
}

func TestPresentationCopiesNeverChangeLiveVehicles(t *testing.T) {
	g := presentationTestGame()
	g.Bus.SetModel(sim.BusArticulated)
	g.Bus.ResetArticulation()
	var p presentationFrames
	p.capture(g, 0)
	g.Bus.Pos.X = 10
	g.Bus.Heading = .1
	g.Bus.TrailerHeading = .02
	g.Camera.Pos.X = 10
	g.TrafficMgr.Cars[0].Pos.X = 5
	p.capture(g, simulationFrame)
	before, car, camera := *g.Bus, *g.TrafficMgr.Cars[0], *g.Camera
	if !p.prepare(g, simulationFrame/2) || math.Abs(p.player.Pos.X-5) > 1e-9 {
		t.Fatal("failed to interpolate pose")
	}
	if g.Bus.Pos != before.Pos || g.Bus.Heading != before.Heading || g.Bus.TrailerHeading != before.TrailerHeading || g.Bus.Speed != before.Speed || *g.Camera != camera || g.TrafficMgr.Cars[0].Pos != car.Pos {
		t.Fatal("presentation altered physics")
	}
	g.Bus.Pos.X = 2000
	if !p.prepare(g, simulationFrame/2) || p.player.Pos != g.Bus.Pos {
		t.Fatal("relocation animated through the map")
	}
	g.TrafficMgr.Cars[0].ID++
	if !p.prepare(g, simulationFrame/2) || p.traffic.Cars[0].Pos != g.TrafficMgr.Cars[0].Pos {
		t.Fatal("replacement car interpolated from old identity")
	}
	g.Settings.IsOpen = true
	if p.prepare(g, simulationFrame/2) {
		t.Fatal("interpolated paused UI")
	}
}

func TestPresentationWrapsAnglesAndReusesBuffers(t *testing.T) {
	v := interpolatePose(vehiclePose{heading: math.Pi - .1}, vehiclePose{heading: -math.Pi + .1}, .5)
	if math.Abs(sim.NormalizeAngle(v.heading-math.Pi)) > 1e-9 {
		t.Fatal("rotation took long way round")
	}
	g := presentationTestGame()
	var p presentationFrames
	for i := 0; i < 10; i++ {
		p.capture(g, time.Duration(i)*simulationFrame)
		p.prepare(g, time.Duration(i)*simulationFrame)
	}
	if n := testing.AllocsPerRun(50, func() { p.capture(g, 10*simulationFrame); p.prepare(g, 10*simulationFrame) }); n != 0 {
		t.Fatalf("steady presentation allocates %.1f objects", n)
	}
}

func TestPresentationClockRecoversAfterSuspension(t *testing.T) {
	var c presentationClock
	origin := time.Unix(1, 0)
	for i := 0; i < 60; i++ {
		c.advance(origin.Add(time.Duration(i) * simulationFrame))
	}
	// An OS suspension causes a capped catch-up batch. It is presentation,
	// not simulation, that must be re-anchored at the end of that batch.
	for i := 0; i < 15; i++ {
		c.advance(origin.Add(3 * time.Second))
	}
	now := origin.Add(3100 * time.Millisecond)
	got := c.displayTime(now)
	want := 75*simulationFrame - simulationFrame*3/2
	if got != want || c.ticks != 75 {
		t.Fatalf("bad suspension recovery: time%v ticks%d", got, c.ticks)
	}
	c.advance(now.Add(simulationFrame))
	if next := c.displayTime(now.Add(simulationFrame)); next-got != simulationFrame {
		t.Fatal("presentation stayed out of phase after suspension")
	}
}
func TestPresentationResetReleasesPreviousCity(t *testing.T) {
	g := presentationTestGame()
	w := WithGraphicsWarmup(g)
	w.frames.capture(g, 0)
	w.clock.advance(time.Now())
	w.resetPresentation()
	if w.frames.count != 0 || !w.clock.origin.IsZero() {
		t.Fatal("kept old presentation clock")
	}
	for _, s := range w.frames.snapshots {
		if s.world != nil || s.player.bus != nil || s.cars != nil || s.buses != nil {
			t.Fatal("retained prior city after switching modes")
		}
	}
}
