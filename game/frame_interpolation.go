package game

import (
	"math"
	"time"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/traffic"
)

type vehiclePose struct {
	pos                                   sim.Vec2
	heading, tail, speed, steering, doors float64
}

func busPresentation(b *sim.Bus) vehiclePose {
	return vehiclePose{b.Pos, b.Heading, b.TailHeading(), b.Speed, b.SteeringAngle, b.DoorProgress}
}
func carPresentation(c *sim.TrafficCar) vehiclePose {
	return vehiclePose{pos: c.Pos, heading: c.VisualHeading(), speed: c.Speed}
}
func presentationAngle(a, b, t float64) float64 { return a + sim.NormalizeAngle(b-a)*t }
func interpolatePose(a, b vehiclePose, t float64) vehiclePose {
	// Replacements, resets and network wrapping are discontinuities, not trips
	// to animate through the city. The simulation alone decides their position.
	if a.pos.Distance(b.pos) > math.Max(50, (math.Max(math.Abs(a.speed), math.Abs(b.speed))/60)*3+1) {
		return b
	}
	return vehiclePose{a.pos.Add(b.pos.Sub(a.pos).Mul(t)), presentationAngle(a.heading, b.heading, t), presentationAngle(a.tail, b.tail, t), sim.Lerp(a.speed, b.speed, t), sim.Lerp(a.steering, b.steering, t), sim.Lerp(a.doors, b.doors, t)}
}

type busPoseRecord struct {
	bus   *sim.Bus
	model sim.BusModel
	pose  vehiclePose
}
type carPoseRecord struct {
	car  *sim.TrafficCar
	id   int
	pose vehiclePose
}
type presentationSnapshot struct {
	at            time.Duration
	world         *sim.World
	player        busPoseRecord
	camera        sim.Camera
	buses         []busPoseRecord
	cars          []carPoseRecord
	busIndex      map[*sim.Bus]int
	carIndex      map[*sim.TrafficCar]int
	busIndexBuilt bool
	carIndexBuilt bool
}

func (s *presentationSnapshot) capture(g *Game, at time.Duration) {
	s.at, s.world = at, g.World
	if s.busIndexBuilt {
		clear(s.busIndex)
		s.busIndexBuilt = false
	}
	if s.carIndexBuilt {
		clear(s.carIndex)
		s.carIndexBuilt = false
	}
	s.player = busPoseRecord{g.Bus, g.Bus.Model, busPresentation(g.Bus)}
	s.camera = *g.Camera
	n := len(g.AIBusMgr.Buses)
	if cap(s.buses) < n {
		s.buses = make([]busPoseRecord, n)
	} else {
		s.buses = s.buses[:n]
	}
	for i, b := range g.AIBusMgr.Buses {
		s.buses[i] = busPoseRecord{b.Bus, b.Bus.Model, busPresentation(b.Bus)}
	}
	cars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	if cap(s.cars) < len(cars) {
		s.cars = make([]carPoseRecord, len(cars))
	} else {
		s.cars = s.cars[:len(cars)]
	}
	for i, c := range cars {
		s.cars[i] = carPoseRecord{c, c.ID, carPresentation(c)}
	}
}

func (s *presentationSnapshot) findBus(live *sim.Bus, hint int) (busPoseRecord, bool) {
	if hint < len(s.buses) {
		if rec := s.buses[hint]; rec.bus == live && rec.model == live.Model {
			return rec, true
		}
	}
	if !s.busIndexBuilt {
		if s.busIndex == nil {
			s.busIndex = make(map[*sim.Bus]int, len(s.buses))
		} else {
			clear(s.busIndex)
		}
		for i, rec := range s.buses {
			s.busIndex[rec.bus] = i
		}
		s.busIndexBuilt = true
	}
	if idx, ok := s.busIndex[live]; ok {
		if rec := s.buses[idx]; rec.bus == live && rec.model == live.Model {
			return rec, true
		}
	}
	return busPoseRecord{}, false
}

func (s *presentationSnapshot) findCar(live *sim.TrafficCar, hint int) (carPoseRecord, bool) {
	if hint < len(s.cars) {
		if rec := s.cars[hint]; rec.car == live && rec.id == live.ID {
			return rec, true
		}
	}
	if !s.carIndexBuilt {
		if s.carIndex == nil {
			s.carIndex = make(map[*sim.TrafficCar]int, len(s.cars))
		} else {
			clear(s.carIndex)
		}
		for i, rec := range s.cars {
			s.carIndex[rec.car] = i
		}
		s.carIndexBuilt = true
	}
	if idx, ok := s.carIndex[live]; ok {
		if rec := s.cars[idx]; rec.car == live && rec.id == live.ID {
			return rec, true
		}
	}
	return carPoseRecord{}, false
}

type presentationFrames struct {
	snapshots   [5]presentationSnapshot
	count, next int
	camera      sim.Camera
	player      sim.Bus
	busValues   []sim.Bus
	fleetValues []traffic.AIBus
	carValues   []sim.TrafficCar
	fleet       traffic.AIBusManager
	traffic     traffic.TrafficManager
}

func (p *presentationFrames) capture(g *Game, at time.Duration) {
	p.snapshots[p.next].capture(g, at)
	p.next = (p.next + 1) % len(p.snapshots)
	p.count = min(p.count+1, len(p.snapshots))
}
func (p *presentationFrames) pair(at time.Duration) (*presentationSnapshot, *presentationSnapshot, float64) {
	first := (p.next - p.count + len(p.snapshots)) % len(p.snapshots)
	a := &p.snapshots[first]
	if at <= a.at {
		return a, a, 0
	}
	for i := 1; i < p.count; i++ {
		b := &p.snapshots[(first+i)%len(p.snapshots)]
		if at <= b.at {
			return a, b, float64(at-a.at) / float64(b.at-a.at)
		}
		a = b
	}
	return a, a, 0
}
func renderBus(dst, src *sim.Bus, a, b busPoseRecord, t float64) {
	*dst = *src
	if a.bus != src || b.bus != src || a.model != src.Model || b.model != src.Model {
		return
	}
	v := interpolatePose(a.pose, b.pose, t)
	// A relocation after the selected historical pair snaps to the live pose.
	if b.pose.pos.Distance(src.Pos) > math.Max(50, math.Abs(src.Speed)/60*4+1) {
		return
	}
	dst.Pos, dst.Heading, dst.TrailerHeading = v.pos, v.heading, v.tail
	dst.Speed, dst.SteeringAngle, dst.DoorProgress = v.speed, v.steering, v.doors
}
func presentationActive(g *Game) bool {
	return g.State == StatePlaying && !g.IsPlanMode && g.Settings != nil && !g.Settings.IsOpen &&
		g.World != nil && g.Bus != nil && g.Camera != nil && g.TrafficMgr != nil && g.AIBusMgr != nil
}
func (p *presentationFrames) prepare(g *Game, at time.Duration) bool {
	if p.count < 2 || !presentationActive(g) {
		return false
	}
	a, b, t := p.pair(at)
	if a.world != g.World || b.world != g.World {
		return false
	}
	p.camera = *g.Camera
	if a.camera.Mode == g.Camera.Mode && b.camera.Mode == g.Camera.Mode && a.camera.Pos.Distance(b.camera.Pos) < 500 {
		p.camera.Pos = a.camera.Pos.Add(b.camera.Pos.Sub(a.camera.Pos).Mul(t))
		p.camera.Zoom = sim.Lerp(a.camera.Zoom, b.camera.Zoom, t)
		p.camera.Rotation = presentationAngle(a.camera.Rotation, b.camera.Rotation, t)
		p.camera.ShakeOffset = a.camera.ShakeOffset.Add(b.camera.ShakeOffset.Sub(a.camera.ShakeOffset).Mul(t))
	}
	renderBus(&p.player, g.Bus, a.player, b.player, t)
	n := len(g.AIBusMgr.Buses)
	if cap(p.busValues) < n {
		p.busValues = make([]sim.Bus, n)
		p.fleetValues = make([]traffic.AIBus, n)
		p.fleet.Buses = make([]*traffic.AIBus, n)
	} else {
		p.busValues = p.busValues[:n]
		p.fleetValues = p.fleetValues[:n]
		p.fleet.Buses = p.fleet.Buses[:n]
	}
	for i, live := range g.AIBusMgr.Buses {
		p.busValues[i] = *live.Bus
		aBus, aOk := a.findBus(live.Bus, i)
		bBus, bOk := b.findBus(live.Bus, i)
		if aOk && bOk {
			renderBus(&p.busValues[i], live.Bus, aBus, bBus, t)
		} else if bOk {
			renderBus(&p.busValues[i], live.Bus, bBus, bBus, 0)
		}
		p.fleetValues[i] = *live
		p.fleetValues[i].Bus = &p.busValues[i]
		p.fleet.Buses[i] = &p.fleetValues[i]
	}
	cars := g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity)
	if cap(p.carValues) < len(cars) {
		p.carValues = make([]sim.TrafficCar, len(cars))
		p.traffic.Cars = make([]*sim.TrafficCar, len(cars))
	} else {
		p.carValues = p.carValues[:len(cars)]
		p.traffic.Cars = p.traffic.Cars[:len(cars)]
	}
	for i, live := range cars {
		p.carValues[i] = *live
		aCar, aOk := a.findCar(live, i)
		bCar, bOk := b.findCar(live, i)
		if bOk && bCar.pose.pos.Distance(live.Pos) <= math.Max(50, math.Abs(live.Speed)/60*4+1) {
			v := bCar.pose
			if aOk {
				v = interpolatePose(aCar.pose, bCar.pose, t)
			}
			p.carValues[i].Pos, p.carValues[i].Heading, p.carValues[i].RoadCurveYaw = v.pos, v.heading, 0
		}
		p.traffic.Cars[i] = &p.carValues[i]
	}
	return true
}
