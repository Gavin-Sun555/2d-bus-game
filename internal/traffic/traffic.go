package traffic

import (
	"image/color"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type TrafficManager struct {
	BeforeRecovery                   func(sim.TrafficRecoveryEvent) `json:"-"`
	separations                      BusCarSeparations
	disableBusCarIndex               bool
	recoveryQueries                  recoveryQueries
	disableRecoveryQueries           bool
	laneOrder                        laneOrderIndex
	disableLaneOrder                 bool
	followingIndices                 []int
	followingCandidates              []*sim.TrafficCar
	busPerceptionActive              bool
	observedBuses                    []*sim.Bus
	observedMerges                   []busMergeObservation
	mergeStorage                     [1 + MaxFleetBuses]busMergeObservation
	mergeOwnBodies, mergeOtherBodies []sim.OBB
	observedParked                   [1 + MaxFleetBuses]bool
	// StepCar is sequential; the perception list is borrowed for that call only.
	busStorage                    [1 + MaxFleetBuses]*sim.Bus
	roundaboutReservations        []roundaboutReservation
	disableJunctionBroadPhase     bool
	disableJunctionCandidates     bool
	junctionCandidates            []int
	junctionExitCandidates        []int
	junctionCandidateSets         []junctionCandidateSet
	junctionCommitted             []int
	junctionRegistered            []bool
	junctionMaxHorizon            float64
	disableJunctionReuse          bool
	junctionScratch               []carJunctionScratch
	nearbyJunctionIDs             []int
	nearbyJunctions               []*sim.TrafficIntersection
	disableJunctionIndex          bool
	spatial                       carSpatialIndex
	spatialActive, disableSpatial bool
	nearbyIndices                 []int
	nearbyCars                    []*sim.TrafficCar
	localCarsReady                bool
	Cars                          []*sim.TrafficCar
	CarSpeedRange                 sim.SpeedRange
	pendingExits                  []sim.Vec2
	populationCapacity            int
	spawnSites                    *trafficSpawnPool
	nextCarID                     int
	AIBusMgr                      *AIBusManager
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
	tm.junctionScratch = nil
	tm.pendingExits = nil
	tm.roundaboutReservations = nil
	tm.nextCarID = count
	tm.spawnSites = newTrafficSpawnPool(w, laneWidth)
	// take reorders the pool it draws from. Dispatch from a copy so the live
	// replacement pool keeps the canonical order a restored manager rebuilds.
	dispatch := tm.spawnSites.clone()
	remaining := dispatch.counts()
	count = min(count, remaining[0]+remaining[1]+remaining[2])
	tm.populationCapacity = min(count, MaxTrafficCars)

	for i := 0; i < count; i++ {
		site := dispatch.take(&remaining)
		car := site.car(i)
		car.Speed = 130 + sim.RandomFloat64()*30
		if i%4 == 0 {
			car.Type = sim.CarTaxi
			car.Color = sim.RGBA(234, 179, 8, 255)
		} else if i%3 == 0 {
			car.Type = sim.CarSUV
			car.Length, car.Width = 50, 24
		}
		tm.assignCarSpeed(car)
		tm.Cars = append(tm.Cars, car)
	}
}

type trafficSpawnSite struct {
	pos       sim.Vec2
	h         float64
	rc        float64
	ew        bool
	laneIdx   int
	roadClass int
}

func (s trafficSpawnSite) car(id int) *sim.TrafficCar {
	return &sim.TrafficCar{
		ID: id, Pos: s.pos, Heading: s.h, RoadCenter: s.rc, IsEW: s.ew, LaneIndex: s.laneIdx,
		Color: carPalette[id%len(carPalette)], Length: 46, Width: 22,
	}
}

func trafficSpawnSites(w *sim.World, laneWidth float64) []trafficSpawnSite {
	defs := []trafficSpawnSite{}

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
		class := trafficRoadClass(r)

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

			for lane := r.FirstGeneralTrafficLane(); lane < r.FirstGeneralTrafficLane()+numLanesPerDir; lane++ {
				offset := (float64(lane) + 0.5) * laneWidth
				if isEW {
					defs = append(defs,
						trafficSpawnSite{pos: sim.Vec2{X: pt.X, Y: roadCoord + offset}, h: 0, rc: roadCoord, ew: true, laneIdx: lane, roadClass: class},
						trafficSpawnSite{pos: sim.Vec2{X: pt.X, Y: roadCoord - offset}, h: math.Pi, rc: roadCoord, ew: true, laneIdx: lane, roadClass: class},
					)
				} else {
					defs = append(defs,
						trafficSpawnSite{pos: sim.Vec2{X: roadCoord - offset, Y: pt.Y}, h: math.Pi / 2, rc: roadCoord, ew: false, laneIdx: lane, roadClass: class},
						trafficSpawnSite{pos: sim.Vec2{X: roadCoord + offset, Y: pt.Y}, h: -math.Pi / 2, rc: roadCoord, ew: false, laneIdx: lane, roadClass: class},
					)
				}
			}
		}
	}

	return defs
}

// Update updates all AI cars according to traffic rules, bus proximity, traffic density and customizable lane width
func (tm *TrafficManager) Update(dt float64, bus *sim.Bus, tls *sim.TrafficLightSystem, densityLevel int, laneWidth float64, worldOptional ...*sim.World) {
	activeCount := len(tm.GetActiveCars(densityLevel))

	var w *sim.World
	if len(worldOptional) > 0 && worldOptional[0] != nil {
		w = worldOptional[0]
	}

	if len(tm.junctionScratch) < activeCount {
		tm.junctionScratch = append(tm.junctionScratch, make([]carJunctionScratch, activeCount-len(tm.junctionScratch))...)
	}
	if w != nil {
		w.PrepareJunctionGeometry(laneWidth)
	}
	if tls != nil {
		tls.PrepareJunctionIndex(laneWidth)
	}
	tm.updateRoundaboutReservations(bus, w, activeCount, laneWidth)
	tm.spatialActive = !tm.disableSpatial && activeCount > 64
	if tm.spatialActive {
		tm.spatial.build(tm.Cars[:activeCount])
		tm.prepareJunctionCandidates(activeCount)
		if tls != nil && len(tm.junctionCandidateSets) < len(tls.Intersections) {
			tm.junctionCandidateSets = make([]junctionCandidateSet, len(tls.Intersections))
		}
		if !tm.disableLaneOrder {
			tm.laneOrder.build(tm.Cars[:activeCount], laneWidth)
		}
	}
	tm.prepareBusPerception(bus, w)
	tm.busPerceptionActive = true
	for i := 0; i < activeCount; i++ {
		car := tm.Cars[i]
		tm.StepCar(dt, car, i, bus, tls, activeCount, laneWidth, w)
		if tm.spatialActive {
			tm.spatial.move(i, car.Pos)
			tm.laneOrder.observe(i, car)
			tm.observeJunctionCandidate(i, car)
		}
	}
	recoveryIndexReady := tm.spatialActive
	tm.spatialActive = false
	tm.laneOrder.ready = false
	tm.busPerceptionActive = false

	// Recycle cars that cannot move after two complete signal cycles.
	tm.resolveDeadlocks(dt, bus, activeCount, laneWidth, w, tls, recoveryIndexReady)
	tm.updateBoundaryTraffic(bus, densityLevel, laneWidth, w)
}

func (tm *TrafficManager) assignCarSpeed(car *sim.TrafficCar) {
	r := tm.CarSpeedRange.Normalize(60, sim.TrafficMaxSpeedKPH)
	car.MaxSpeedKPH = r.Sample()
	car.TargetSpeed = sim.SpeedKPH(car.MaxSpeedKPH)
	car.CanOvertake = car.MaxSpeedKPH > r.MinKPH
	car.Speed = math.Min(car.Speed, car.TargetSpeed)
}
