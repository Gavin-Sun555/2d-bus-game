package traffic

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type TrafficManager struct {
	Cars         []*sim.TrafficCar
	pendingExits []sim.Vec2
	nextCarID    int
	AIBusMgr     *AIBusManager
}

const MaxTrafficCars = sim.MaxTrafficCars

func NewTrafficManager(initialLaneWidth float64) *TrafficManager {
	tm := &TrafficManager{
		Cars: make([]*sim.TrafficCar, 0),
	}
	tm.initializeCars(MaxTrafficCars, initialLaneWidth)
	return tm
}

// GetActiveCars returns the slice of cars active under the given density level
func (tm *TrafficManager) GetActiveCars(densityLevel int) []*sim.TrafficCar {
	if tm == nil {
		return nil
	}
	activeCount := sim.TrafficCarCount(densityLevel)
	if activeCount > len(tm.Cars) {
		activeCount = len(tm.Cars)
	}
	return tm.Cars[:activeCount]
}

var carPalette = []color.RGBA{
	sim.RGBA(239, 68, 68, 255),   // Crimson
	sim.RGBA(59, 130, 246, 255),  // Cobalt Blue
	sim.RGBA(16, 185, 129, 255),  // Emerald
	sim.RGBA(248, 250, 252, 255), // Pearl White
	sim.RGBA(71, 85, 105, 255),   // Slate Gray
	sim.RGBA(168, 85, 247, 255),  // Violet
	sim.RGBA(234, 179, 8, 255),   // Taxi Yellow
}

func (tm *TrafficManager) initializeCars(count int, laneWidth float64) {
	w := sim.NewWorld(laneWidth, sim.MapThemeMetropolis)
	tm.BuildForWorld(w, count, laneWidth)
}

// BuildForWorld adapts civilian AI traffic to the active city map's road network
func (tm *TrafficManager) BuildForWorld(w *sim.World, count int, laneWidth float64) {
	if w == nil {
		return
	}
	tm.Cars = make([]*sim.TrafficCar, 0)
	tm.pendingExits = nil
	tm.nextCarID = count

	type spawnDef struct {
		pos     sim.Vec2
		h       float64
		rc      float64
		ew      bool
		laneIdx int
	}
	var defs []spawnDef

	for _, r := range w.Roads {
		dir := r.P2.Sub(r.P1)
		lenR := dir.Length()
		if lenR < 300 {
			continue
		}
		unitDir := dir.Mul(1.0 / lenR)
		isEW := math.Abs(r.P1.Y-r.P2.Y) < 1.0
		var roadCoord float64
		if isEW {
			roadCoord = r.P1.Y
		} else {
			roadCoord = r.P1.X
		}

		numLanesPerDir := r.GeneralTrafficLanes()

		step := 220.0
		for dist := 260.0; dist <= lenR-260.0; dist += step {
			pt := r.P1.Add(unitDir.Mul(dist))

			nearInter := false
			for _, ipt := range w.Intersections {
				if pt.Distance(ipt) < 220 {
					nearInter = true
					break
				}
			}
			if nearInter {
				continue
			}

			nearRb := false
			for _, rb := range w.Roundabouts {
				if pt.Distance(rb.Center) < rb.YieldLineDistance()+60 {
					nearRb = true
					break
				}
			}
			if nearRb {
				continue
			}

			nearStop := false
			for _, st := range w.Stops {
				if pt.Distance(st.Pos) < 160 {
					nearStop = true
					break
				}
			}
			if nearStop {
				continue
			}

			for lane := 0; lane < numLanesPerDir; lane++ {
				offset := (float64(lane) + 0.5) * laneWidth
				if isEW {
					defs = append(defs,
						spawnDef{pos: sim.Vec2{X: pt.X, Y: roadCoord + offset}, h: 0, rc: roadCoord, ew: true, laneIdx: lane},
						spawnDef{pos: sim.Vec2{X: pt.X, Y: roadCoord - offset}, h: math.Pi, rc: roadCoord, ew: true, laneIdx: lane},
					)
				} else {
					defs = append(defs,
						spawnDef{pos: sim.Vec2{X: roadCoord - offset, Y: pt.Y}, h: math.Pi / 2, rc: roadCoord, ew: false, laneIdx: lane},
						spawnDef{pos: sim.Vec2{X: roadCoord + offset, Y: pt.Y}, h: -math.Pi / 2, rc: roadCoord, ew: false, laneIdx: lane},
					)
				}
			}
		}
	}

	if count > len(defs) {
		count = len(defs)
	}

	stride := 1.0
	if count > 0 {
		stride = float64(len(defs)) / float64(count)
	}
	for i := 0; i < count; i++ {
		idx := int(float64(i)*stride) % len(defs)
		sd := defs[idx]

		cType := sim.CarSedan
		cColor := carPalette[i%len(carPalette)]
		if i%4 == 0 {
			cType = sim.CarTaxi
			cColor = sim.RGBA(234, 179, 8, 255)
		} else if i%3 == 0 {
			cType = sim.CarSUV
		}

		car := &sim.TrafficCar{
			ID:          i,
			Pos:         sd.pos,
			Heading:     sd.h,
			Speed:       130 + sim.RandomFloat64()*30,
			TargetSpeed: 150 + sim.RandomFloat64()*40,
			Type:        cType,
			Color:       cColor,
			Length:      46,
			Width:       22,
			RoadCenter:  sd.rc,
			IsEW:        sd.ew,
			LaneIndex:   sd.laneIdx,
		}
		if cType == sim.CarSUV {
			car.Length = 50
			car.Width = 24
		}
		tm.Cars = append(tm.Cars, car)
	}
}

// Update updates all AI cars according to traffic rules, bus proximity, traffic density and customizable lane width
func (tm *TrafficManager) Update(dt float64, bus *sim.Bus, tls *sim.TrafficLightSystem, densityLevel int, laneWidth float64, worldOptional ...*sim.World) {
	activeCount := len(tm.GetActiveCars(densityLevel))

	var w *sim.World
	if len(worldOptional) > 0 && worldOptional[0] != nil {
		w = worldOptional[0]
	}

	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]
		tm.StepCar(dt, car, i, bus, tls, activeCount, laneWidth, w)
	}

	// Deadlock prevention: when two non-player vehicles are jammed motionless for > 1 traffic light cycle, disappear
	tm.ResolveDeadlocks(dt, bus, activeCount, laneWidth, w, tls)
	tm.updateBoundaryTraffic(bus, densityLevel, laneWidth, w)
}
