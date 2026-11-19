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
	at     time.Duration
	world  *sim.World
	player busPoseRecord
	camera sim.Camera
	buses  []busPoseRecord
	cars   []carPoseRecord
}

func (s *presentationSnapshot) capture(g *Game, at time.Duration) {
	s.at, s.world = at, g.World
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
		if i < len(a.buses) && i < len(b.buses) {
			renderBus(&p.busValues[i], live.Bus, a.buses[i], b.buses[i], t)
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
		if i < len(a.cars) && i < len(b.cars) && a.cars[i].car == live && b.cars[i].car == live && a.cars[i].id == live.ID && b.cars[i].id == live.ID {
			v := interpolatePose(a.cars[i].pose, b.cars[i].pose, t)
			if b.cars[i].pose.pos.Distance(live.Pos) <= math.Max(50, math.Abs(live.Speed)/60*4+1) {
				p.carValues[i].Pos, p.carValues[i].Heading, p.carValues[i].RoadCurveYaw = v.pos, v.heading, 0
			}
		}
		p.traffic.Cars[i] = &p.carValues[i]
	}
	return true
}
