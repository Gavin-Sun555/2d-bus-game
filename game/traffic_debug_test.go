package game

import (
	"bytes"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func debugFiles(t *testing.T, g *Game) []string {
	t.Helper()
	paths, err := filepath.Glob(filepath.Join(g.trafficDebug.dir, "congestion-*.json"))
	if err != nil {
		t.Fatal(err)
	}
	return paths
}

func debugTestGame(t *testing.T) *Game {
	t.Helper()
	g := saveTestGame(t)
	for _, a := range g.AIBusMgr.Buses {
		a.Driver.State = autopilot.AutopilotStateDocked
	}
	g.TrafficMgr.Cars = g.TrafficMgr.Cars[:2]
	a, b := g.TrafficMgr.Cars[0], g.TrafficMgr.Cars[1]
	id := b.ID
	// Queue both cars on car a's own road just before one junction. Snapshots
	// are de-duplicated by nearest junction, so a random spawn between two
	// junctions could otherwise split the pair into separate regions.
	forward := sim.UnitFromAngle(a.Heading)
	best, along := math.Inf(1), 0.
	for _, inter := range g.TrafficLights.Intersections {
		offRoad := math.Abs(inter.Center.Y - a.Pos.Y)
		if !a.IsEW {
			offRoad = math.Abs(inter.Center.X - a.Pos.X)
		}
		if d := inter.Center.Distance(a.Pos); offRoad <= 200 && d < best {
			best, along = d, inter.Center.Sub(a.Pos).Dot(forward)
		}
	}
	if math.IsInf(best, 1) {
		t.Fatal("no junction on the test car's road")
	}
	a.Pos = a.Pos.Add(forward.Mul(along - 200))
	*b = *a
	b.ID, b.Pos = id, a.Pos.Sub(forward.Mul(70))
	a.Speed, b.Speed = 0, 0
	return g
}

func TestTrafficDebugThresholdDedupAndReplay(t *testing.T) {
	g := debugTestGame(t)
	g.updateTrafficDebug(1000)
	if g.trafficDebug != nil || g.TrafficMgr.BeforeRecovery != nil {
		t.Fatal("debug enabled without flag")
	}
	if err := g.EnableDebug(); err != nil {
		t.Fatal(err)
	}
	car := g.TrafficMgr.Cars[0]
	timeout := g.TrafficLights.DeadlockTimeoutAt(car.Pos)
	g.updateTrafficDebug(timeout)
	if len(debugFiles(t, g)) != 0 {
		t.Fatal("saved before exceeding two full cycles")
	}
	g.updateTrafficDebug(.1)
	paths := debugFiles(t, g)
	if len(paths) != 1 {
		t.Fatalf("expected one snapshot for shared location, got %d", len(paths))
	}
	e, err := savefile.Read[SaveMetadata](paths[0])
	if err != nil {
		t.Fatal(err)
	}
	if e.Metadata.Source != "debug" || e.Metadata.Debug == nil || e.Metadata.Debug.Reason != "two-signal-cycles" || e.Metadata.Debug.StationarySeconds <= timeout {
		t.Fatal("missing congestion evidence")
	}
	g.updateTrafficDebug(timeout)
	if len(debugFiles(t, g)) != 1 {
		t.Fatal("repeatedly saved the same stationary episode")
	}
	before := g.trafficDebug
	g.savePathOverride = paths[0]
	if err = g.LoadGame(); err != nil {
		t.Fatalf("debug snapshot cannot load: %v", err)
	}
	if g.trafficDebug != before {
		t.Fatal("loading a save disabled debug mode")
	}
}

func TestTrafficDebugCapturesBeforeCarResetAndKeepsManualSlot(t *testing.T) {
	g := debugTestGame(t)
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	manual, err := os.ReadFile(g.savePathOverride)
	if err != nil {
		t.Fatal(err)
	}
	if err = g.EnableDebug(); err != nil {
		t.Fatal(err)
	}
	g.updateTrafficDebug(.1)
	car := *g.TrafficMgr.Cars[0]
	g.TrafficMgr.DespawnAndResetCar(0, g.World, 52, g.Bus)
	paths := debugFiles(t, g)
	if len(paths) != 1 {
		t.Fatal("reset did not preserve pre-recovery scene")
	}
	s, err := readSave(paths[0])
	if err != nil {
		t.Fatal(err)
	}
	if len(s.Cars) == 0 || s.Cars[0].Car.ID != car.ID || s.Cars[0].Car.Pos != car.Pos {
		t.Fatal("debug save contains post-reset vehicle instead of original")
	}
	after, err := os.ReadFile(g.savePathOverride)
	if err != nil || !bytes.Equal(manual, after) {
		t.Fatal("debug capture changed manual slot")
	}
}

func TestTrafficDebugExcludesServiceAndResetsMovingEpisodes(t *testing.T) {
	g := debugTestGame(t)
	if err := g.EnableDebug(); err != nil {
		t.Fatal(err)
	}
	for _, car := range g.TrafficMgr.Cars {
		car.Speed = 30
	}
	g.updateTrafficDebug(1000)
	if len(debugFiles(t, g)) != 0 {
		t.Fatal("captured moving cars or normal passenger service")
	}
	car := g.TrafficMgr.Cars[0]
	car.Speed = 0
	timeout := g.TrafficLights.DeadlockTimeoutAt(car.Pos)
	g.updateTrafficDebug(timeout / 2)
	car.Pos.X += 3
	g.updateTrafficDebug(timeout/2 + .1)
	if len(debugFiles(t, g)) != 0 {
		t.Fatal("combined separate stops into one congestion interval")
	}
}

func TestTrafficDebugRetentionOnlyPrunesOwnedFiles(t *testing.T) {
	g := debugTestGame(t)
	if err := g.EnableDebug(); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < MaxTrafficDebugSaves+1; i++ {
		path := filepath.Join(g.trafficDebug.dir, "congestion-000-"+string(rune('A'+i))+".json")
		if err := os.WriteFile(path, []byte("fixture"), 0600); err != nil {
			t.Fatal(err)
		}
	}
	other := filepath.Join(g.trafficDebug.dir, "notes.txt")
	if err := os.WriteFile(other, []byte("keep"), 0600); err != nil {
		t.Fatal(err)
	}
	g.updateTrafficDebug(.1)
	g.captureTrafficDebug(sim.TrafficRecoveryEvent{Kind: "car", ID: 1, Pos: g.TrafficMgr.Cars[0].Pos, Reason: "deadlock-reset"})
	if len(debugFiles(t, g)) != MaxTrafficDebugSaves {
		t.Fatal("debug retention limit failed")
	}
	if _, err := os.Stat(other); err != nil {
		t.Fatal("pruned unrelated file")
	}
}

func TestTrafficDebugPersistsModeSwitch(t *testing.T) {
	g := debugTestGame(t)
	if err := g.EnableDebug(); err != nil {
		t.Fatal(err)
	}
	d := g.trafficDebug
	g.SelectMode(ModeCareer)
	if g.trafficDebug != d {
		t.Fatal("career switch disabled debug")
	}
	g.SelectMode(ModeFast)
	if g.trafficDebug != d {
		t.Fatal("fast mode switch disabled debug")
	}
}

func TestTrafficDebugRetriesFailedWrite(t *testing.T) {
	g := debugTestGame(t)
	if err := g.EnableDebug(); err != nil {
		t.Fatal(err)
	}
	dir := g.trafficDebug.dir
	bad := filepath.Join(t.TempDir(), "file")
	if err := os.WriteFile(bad, []byte("not a directory"), 0600); err != nil {
		t.Fatal(err)
	}
	g.trafficDebug.dir = bad
	timeout := g.TrafficLights.DeadlockTimeoutAt(g.TrafficMgr.Cars[0].Pos)
	g.updateTrafficDebug(timeout + .1)
	if g.trafficDebug.retryAt <= g.trafficDebug.seconds {
		t.Fatal("write failure was not throttled for retry")
	}
	g.trafficDebug.dir = dir
	g.updateTrafficDebug(6)
	if len(debugFiles(t, g)) != 1 {
		t.Fatal("congestion snapshot did not retry after write failure")
	}
}
