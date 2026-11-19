package game

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"os"
	"path/filepath"
	"sort"
	"sync"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/savefile"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

const MaxTrafficDebugSaves = 20

type debugVehicleKey struct {
	Kind string
	ID   int
}
type debugStopWatch struct {
	Anchor   sim.Vec2
	Seconds  float64
	Timeout  float64
	Seen     uint64
	Reported bool
}
type debugRegion struct {
	X, Y   int
	Reason string
}
type trafficDebugState struct {
	dir      string
	world    *sim.World
	seconds  float64
	tick     uint64
	sequence uint64
	retryAt  float64
	pending  int
	done     chan debugWrite
	watches  map[debugVehicleKey]*debugStopWatch
	regions  map[debugRegion]float64
}

type CongestionDebugVehicle struct {
	Kind           string
	ID             int
	Position       sim.Vec2
	Heading, Speed float64
	DriverState    string  `json:",omitempty"`
	TargetSpeed    float64 `json:",omitempty"`
	DeadlockTime   float64 `json:",omitempty"`
}
type CongestionDebugReport struct {
	Reason                 string
	VehicleKind            string
	VehicleID              int
	Position               sim.Vec2
	Intersection           *sim.Vec2 `json:",omitempty"`
	StationarySeconds      float64
	FullSignalCycleSeconds float64
	SimulationSeconds      float64
	Nearby                 []CongestionDebugVehicle
}

// EnableDebug enables congestion snapshots without changing manual/autosave
// slots. Timers use simulation time and the nearest junction's complete cycle.
func (g *Game) EnableDebug() error {
	path, err := g.saveSlotPath(0)
	if err != nil {
		return err
	}
	dir := filepath.Join(filepath.Dir(path), "debug-saves")
	if err = os.MkdirAll(dir, 0700); err != nil {
		return err
	}
	g.trafficDebug = &trafficDebugState{dir: dir}
	log.Printf("Traffic debug enabled: %s", dir)
	return nil
}

func (g *Game) updateTrafficDebug(dt float64) {
	d := g.trafficDebug
	if d == nil || g.World == nil || dt <= 0 {
		return
	}
	if d.world != g.World {
		d.world, d.seconds = g.World, 0
		d.watches = make(map[debugVehicleKey]*debugStopWatch)
		d.regions = make(map[debugRegion]float64)
	}
	d.finishDebugWrites(false)
	d.seconds += dt
	d.tick++
	if g.TrafficMgr != nil {
		g.TrafficMgr.BeforeRecovery = g.captureTrafficDebug
		for _, car := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			g.watchTrafficStop(debugVehicleKey{"car", car.ID}, car.Pos, car.Speed, false, dt)
		}
	}
	if g.AIBusMgr != nil {
		g.AIBusMgr.BeforeRecovery = g.captureTrafficDebug
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus == nil || a.Driver == nil {
				continue
			}
			if a.Driver.BeforeRecovery == nil {
				bus := a
				a.Driver.BeforeRecovery = func() {
					g.captureTrafficDebug(sim.TrafficRecoveryEvent{Kind: "bus", ID: bus.ID, Pos: bus.Bus.Pos, Reason: "local-recovery"})
				}
			}
			servicing := a.Bus.DoorsOpen || a.Bus.PassengerServiceRemaining() > 0 || a.Driver.State == autopilot.AutopilotStateDocked || a.Bus.TerminalReplacementHold
			g.watchTrafficStop(debugVehicleKey{"bus", a.ID}, a.Bus.Pos, a.Bus.Speed, servicing, dt)
		}
	}
	for key, watch := range d.watches {
		if watch.Seen != d.tick {
			delete(d.watches, key)
		}
	}
}

func (g *Game) watchTrafficStop(key debugVehicleKey, pos sim.Vec2, speed float64, servicing bool, dt float64) {
	d := g.trafficDebug
	if servicing || math.Abs(speed) >= 3 {
		delete(d.watches, key)
		return
	}
	w := d.watches[key]
	if w == nil || pos.Distance(w.Anchor) > 2 {
		w = &debugStopWatch{Anchor: pos, Timeout: g.TrafficLights.DeadlockTimeoutAt(pos)}
		d.watches[key] = w
	}
	w.Seen = d.tick
	w.Seconds += dt
	if !w.Reported && w.Seconds > w.Timeout && d.seconds >= d.retryAt {
		w.Reported = g.captureTrafficDebugEvent(sim.TrafficRecoveryEvent{Kind: key.Kind, ID: key.ID, Pos: pos, Reason: "two-signal-cycles"})
	}
}

func (g *Game) captureTrafficDebug(event sim.TrafficRecoveryEvent) {
	g.captureTrafficDebugEvent(event)
}

func (g *Game) captureTrafficDebugEvent(event sim.TrafficRecoveryEvent) bool {
	d := g.trafficDebug
	if d == nil || g.World == nil || d.world != g.World || d.seconds < d.retryAt {
		return false
	}
	timeout := g.TrafficLights.DeadlockTimeoutAt(event.Pos)
	report := &CongestionDebugReport{Reason: event.Reason, VehicleKind: event.Kind, VehicleID: event.ID, Position: event.Pos, FullSignalCycleSeconds: timeout / 2, SimulationSeconds: d.seconds}
	if w := d.watches[debugVehicleKey{event.Kind, event.ID}]; w != nil {
		report.StationarySeconds = w.Seconds
	}
	nearest := math.Inf(1)
	if g.TrafficLights != nil {
		for _, inter := range g.TrafficLights.Intersections {
			if distance := event.Pos.Distance(inter.Center); distance < nearest {
				p := inter.Center
				report.Intersection, nearest = &p, distance
			}
		}
	}
	regionPos := event.Pos
	if report.Intersection != nil {
		regionPos = *report.Intersection
	}
	region := debugRegion{int(math.Floor(regionPos.X / 500)), int(math.Floor(regionPos.Y / 500)), event.Reason}
	if until, ok := d.regions[region]; ok && d.seconds < until {
		return true
	}
	if g.TrafficMgr != nil {
		for _, car := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			if car.Pos.Distance(event.Pos) < 1200 {
				report.Nearby = append(report.Nearby, CongestionDebugVehicle{Kind: "car", ID: car.ID, Position: car.Pos, Heading: car.Heading, Speed: car.Speed, DeadlockTime: car.DeadlockTimer})
			}
		}
	}
	if g.AIBusMgr != nil {
		for _, a := range g.AIBusMgr.Buses {
			if a.Bus != nil && a.Bus.Pos.Distance(event.Pos) < 1200 {
				v := CongestionDebugVehicle{Kind: "bus", ID: a.ID, Position: a.Bus.Pos, Heading: a.Bus.Heading, Speed: a.Bus.Speed, DeadlockTime: a.DeadlockTimer}
				if a.Driver != nil {
					v.DriverState, v.TargetSpeed = a.Driver.State.String(), a.Driver.LastTargetSpeed
				}
				report.Nearby = append(report.Nearby, v)
			}
		}
	}
	snapshot := g.snapshot()
	snapshot.Camera.Pos, snapshot.Camera.TargetPos = event.Pos, event.Pos
	metadata := g.saveMetadata("debug", event.Reason)
	metadata.Debug = report
	now := time.Now()
	d.sequence++
	job := debugWrite{
		event: event, region: region, metadata: metadata, now: now,
		path: filepath.Join(d.dir, fmt.Sprintf("congestion-%s-%06d-%s%d.json", now.Format("20060102-150405.000000000"), d.sequence, event.Kind, event.ID)),
	}
	// Encoding and writing a full save takes tens of milliseconds. Detach
	// the snapshot from live state and finish off the simulation thread, as
	// interval autosaves do.
	job.snapshot = detachedSaveSnapshot(snapshot)
	d.regions[region] = d.seconds + timeout
	d.pending++
	if d.done == nil {
		d.done = make(chan debugWrite, 64)
	}
	dir, done := d.dir, d.done
	go func() {
		job.err = writeDebugSnapshot(dir, job)
		job.snapshot = gameSave{}
		done <- job
	}()
	return true
}

type debugWrite struct {
	event    sim.TrafficRecoveryEvent
	region   debugRegion
	snapshot gameSave
	metadata *SaveMetadata
	now      time.Time
	path     string
	err      error
}

var debugRetention sync.Mutex

func writeDebugSnapshot(dir string, job debugWrite) error {
	payload, err := json.Marshal(job.snapshot)
	if err != nil {
		return err
	}
	encoded, err := savefile.Encode(payload, "Traffic debug: "+job.event.Reason, job.metadata, job.now)
	if err != nil {
		return err
	}
	debugRetention.Lock()
	defer debugRetention.Unlock()
	if err = savefile.Write(job.path, encoded); err != nil {
		return err
	}
	files, _ := filepath.Glob(filepath.Join(dir, "congestion-*.json"))
	sort.Strings(files)
	for len(files) > MaxTrafficDebugSaves {
		if err := os.Remove(files[0]); err != nil {
			log.Printf("Traffic debug retention: %v", err)
			break
		}
		files = files[1:]
	}
	return nil
}

// finishDebugWrites applies completed background writes. A failed write
// re-arms its vehicle and region so the snapshot is retried. wait blocks
// until every write in flight has finished.
func (d *trafficDebugState) finishDebugWrites(wait bool) {
	for d.pending > 0 {
		var job debugWrite
		if wait {
			job = <-d.done
		} else {
			select {
			case job = <-d.done:
			default:
				return
			}
		}
		d.pending--
		event := job.event
		if job.err != nil {
			d.retryAt = d.seconds + 5
			delete(d.regions, job.region)
			if w := d.watches[debugVehicleKey{event.Kind, event.ID}]; w != nil {
				w.Reported = false
			}
			log.Printf("Traffic debug save failed: %v", job.err)
			continue
		}
		log.Printf("Traffic debug saved (%s %s%d at %.1f,%.1f): %s", event.Reason, event.Kind, event.ID, event.Pos.X, event.Pos.Y, job.path)
	}
}

// FlushDebug waits for congestion snapshots still being written.
func (g *Game) FlushDebug() {
	if g.trafficDebug != nil {
		g.trafficDebug.finishDebugWrites(true)
	}
}
