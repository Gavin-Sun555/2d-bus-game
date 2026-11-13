package sim

import (
	"math"
	"sort"
)

const (
	RailTrainLength    = 414.
	freightTrainLength = 480.
	RailTrackCount     = 2
	railSpeedScale     = .18 // km/h per world unit per second
	railBrakeTime      = 1.7
	railDwell          = 6.
	railHeadway        = 150.
	railJitter         = 30.
	railWarning        = 5.
	railReopen         = 1.5
)

type RailService uint8

const (
	RailHighSpeed RailService = iota
	RailConventionalPassenger
	RailFreight
)

type Railway struct {
	Conventional                 bool
	StartX, EndX                 float64
	Y, HalfWidth, TrackSpacing   float64
	StationStopID                int
	StationCenter                Vec2
	StationApproach              Vec2
	StationHeading, StationDepth float64
	Crossings                    []RailCrossing
}

type RailCrossing struct {
	Center          Vec2
	RoadHalfWidth   float64
	Level           bool
	ViaductHalfSpan float64
	// First closing time and closure duration, one per track.
	windows        [RailTrackCount][2]float64
	freightWindows [RailTrackCount][2]float64
}

type RailTrain struct {
	Pos     Vec2
	Heading float64
	Service RailService
	Speed   float64 // World units per second.
}

type RailGateState struct {
	Red      bool
	Occupied bool
	// Lowered runs from zero (raised) to one (horizontal).
	Lowered float64
}

func (w *World) Railway() *Railway {
	if len(w.railways) == 0 {
		return nil
	}
	return w.railways[0]
}

func (w *World) Railways() []*Railway { return w.railways }

func (t RailTrain) Length() float64 {
	if t.Service == RailFreight {
		return freightTrainLength
	}
	return RailTrainLength
}

func (r *Railway) cruiseSpeed() float64 {
	if r.Conventional {
		return 150 / railSpeedScale
	}
	return 300 / railSpeedScale
}

func (r *Railway) TrackY(track int) float64 {
	return r.Y + (1-2*float64(track))*r.TrackSpacing
}

func (w *World) rebuildRailway(lw float64) {
	w.railways = nil
	// Authored stop IDs keep the railway attached even if the player renames
	// the bus stop or switches its platform into the median.
	id, depth, trackY, setback := 5, 310., -.255, 0.
	switch w.Theme {
	case MapThemeMetropolis:
	case MapThemeRiverValley:
		// Leave room for a level crossing beyond the broad junction mouths.
		id, depth, trackY, setback = 4, 320, -.22, 190
	default:
		return
	}
	if id >= len(w.Stops) || len(w.Roads) == 0 {
		return
	}
	stop := w.Stops[id]
	u := UnitFromAngle(stop.Heading)
	right := Vec2{X: -u.Y, Y: u.X}
	edge := float64(stop.RoadLanes)*lw/2 + 10 + ShoulderWidth + 32
	if !stop.CenterBRT {
		edge = math.Max(edge, stop.ShelterPos.Sub(stop.RoadCenter).Dot(right)+42)
	}
	center := stop.RoadCenter.Add(right.Mul(edge + depth/2 + setback))
	r := &Railway{
		Y: center.Sub(right.Mul(trackY * depth)).Y, HalfWidth: depth * .225, TrackSpacing: depth * .145,
		StationStopID: id, StationCenter: center, StationHeading: stop.Heading + math.Pi, StationDepth: depth,
		StationApproach: stop.RoadCenter.Add(right.Mul(float64(stop.RoadLanes)*lw/2 + ShoulderWidth + 10)),
	}
	w.buildRailCorridor(r, lw)
	w.railways = []*Railway{r}
	if w.Theme != MapThemeMetropolis {
		return
	}
	for _, stationStop := range w.Stops {
		if !w.isMetropolisConventionalStop(stationStop) || math.Cos(stationStop.Heading) >= 0 {
			continue
		}
		// The westbound platform is on the north side of the central avenue.
		// Put the concourse between the bus stop and the east-west tracks.
		const depth = 240.
		edge := float64(stationStop.RoadLanes)*lw/2 + 10 + ShoulderWidth
		setback := math.Max(edge+32, stationStop.RoadCenter.Y-stationStop.ShelterPos.Y+42)
		center := stationStop.RoadCenter.Sub(Vec2{Y: setback + depth/2})
		conventional := &Railway{
			Conventional: true,
			Y:            center.Y - depth/4, HalfWidth: 61, TrackSpacing: 38,
			StationStopID: stationStop.ID, StationCenter: center, StationDepth: depth,
			StationApproach: stationStop.RoadCenter.Sub(Vec2{Y: edge}),
		}
		w.buildRailCorridor(conventional, lw)
		w.railways = append(w.railways, conventional)
		break
	}
}

func (w *World) buildRailCorridor(r *Railway, lw float64) {
	r.StartX, r.EndX = math.Inf(1), math.Inf(-1)
	type roadCrossing struct {
		x, half, clearance float64
		lanes              int
	}
	byX := make(map[float64]roadCrossing)
	for _, road := range w.Roads {
		r.StartX = math.Min(r.StartX, math.Min(road.P1.X, road.P2.X))
		r.EndX = math.Max(r.EndX, math.Max(road.P1.X, road.P2.X))
		if math.Abs(road.P1.X-road.P2.X) >= 1 {
			continue
		}
		lo, hi := math.Min(road.P1.Y, road.P2.Y), math.Max(road.P1.Y, road.P2.Y)
		if r.Y < lo || r.Y > hi {
			continue
		}
		x := road.P1.X
		c := byX[x]
		c.x = x
		c.half = math.Max(c.half, road.GetWidth(lw)/2)
		c.lanes = max(c.lanes, road.LaneCount())
		c.clearance = math.Max(c.clearance, math.Min(r.Y-lo, hi-r.Y))
		byX[x] = c
	}
	r.StartX -= 1400
	r.EndX += 1400
	roads := make([]roadCrossing, 0, len(byX))
	for _, road := range byX {
		roads = append(roads, road)
	}
	sort.Slice(roads, func(i, j int) bool { return roads[i].x < roads[j].x })
	eligible := 0
	const articulatedQueueRoom = 192. // Space for one complete articulated bus between gate and yield line.
	for _, road := range roads {
		p := Vec2{X: road.x, Y: r.Y}
		// Keep major arterials and cramped approaches grade-separated.
		level := false
		nearRoundabout := false
		for _, rb := range w.Roundabouts {
			if math.Abs(p.X-rb.Center.X) <= road.half &&
				math.Abs(p.Y-rb.Center.Y) <= rb.YieldLineDistance()+r.HalfWidth+RailGateMargin+articulatedQueueRoom {
				nearRoundabout = true
				break
			}
		}
		if !nearRoundabout && road.lanes <= 4 && road.clearance > r.HalfWidth+55 && !w.InsideJunction(p, lw, r.HalfWidth+75) {
			level = eligible%2 == 0
			eligible++
		}
		crossing := RailCrossing{Center: p, RoadHalfWidth: road.half, Level: level}
		if !level {
			// The crossing can overlap the wider junction apron even where
			// the rail centerline sits outside the junction's asphalt.
			clear := road.half + ShoulderWidth
			for _, junction := range w.Intersections {
				if math.Abs(junction.X-p.X) > 1 || w.IsAtRoundabout(junction) {
					continue
				}
				g := w.JunctionAt(junction, lw)
				if math.Abs(junction.Y-p.Y) < g.HalfY+ShoulderWidth+r.HalfWidth+35 {
					clear = math.Max(clear, g.HalfX+ShoulderWidth)
				}
			}
			crossing.ViaductHalfSpan = clear + 108
		}
		r.Crossings = append(r.Crossings, crossing)
	}
	for i := range r.Crossings {
		c := &r.Crossings[i]
		if !c.Level {
			continue
		}
		for track := range c.windows {
			distance := r.distanceFromEntry(track, c.Center.X)
			margin := c.RoadHalfWidth + ShoulderWidth + RailTrainLength/2
			entry := r.arrivalTime(track, distance-margin)
			exit := r.arrivalTime(track, distance+margin)
			c.windows[track] = railGateWindow(r.firstDeparture(track), entry, exit)
			if r.Conventional {
				margin = c.RoadHalfWidth + ShoulderWidth + freightTrainLength/2
				speed := r.cruiseSpeed()
				c.freightWindows[track] = railGateWindow(r.firstDeparture(track), (distance-margin)/speed, (distance+margin)/speed)
			}
		}
	}
}

func railGateWindow(first, entry, exit float64) [2]float64 {
	return [2]float64{first + entry - railWarning, exit - entry + railWarning}
}

func (r *Railway) distanceFromEntry(track int, x float64) float64 {
	if track == 0 {
		return x - r.StartX
	}
	return r.EndX - x
}

func (r *Railway) arrivalTime(track int, distance float64) float64 {
	speed := r.cruiseSpeed()
	stop := r.distanceFromEntry(track, r.StationCenter.X)
	halfBrake := speed * railBrakeTime / 2
	switch {
	case distance <= stop-halfBrake:
		return distance / speed
	case distance < stop:
		return (stop-halfBrake)/speed + railBrakeTime - math.Sqrt(2*(stop-distance)*railBrakeTime/speed)
	case distance < stop+halfBrake:
		return (stop-halfBrake)/speed + railBrakeTime + railDwell + math.Sqrt(2*(distance-stop)*railBrakeTime/speed)
	default:
		return (distance+2*halfBrake)/speed + railDwell
	}
}

func (r *Railway) firstDeparture(track int) float64 {
	speed := r.cruiseSpeed()
	distance := r.distanceFromEntry(track, r.StationCenter.X)
	arrival := (distance-speed*railBrakeTime/2)/speed + railBrakeTime
	first := 6.
	if r.Conventional {
		first += 24
	}
	return first + float64(track)*railHeadway/2 - arrival
}

// Jitter each slot independently within 30 seconds. Consecutive trains in
// one direction stay 120–180 seconds apart, including across a save/load.
// Conventional services use a slightly smaller jitter to allow for the
// difference in travel time between a stopping passenger and a through freight.
func railServiceTime(first float64, schedule int, service int64) float64 {
	slot := first + float64(service)*railHeadway
	if service == 0 {
		return slot
	}
	hash := uint64(service) + uint64(schedule+1)*0x9e3779b97f4a7c15
	hash = (hash ^ (hash >> 30)) * 0xbf58476d1ce4e5b9
	hash = (hash ^ (hash >> 27)) * 0x94d049bb133111eb
	hash ^= hash >> 31
	jitter := railJitter
	if schedule >= RailTrackCount {
		jitter -= railBrakeTime + railDwell
	}
	return slot + float64(hash>>11)*(jitter/(1<<53))
}

func latestRailService(elapsed, first float64, track int) int64 {
	service := int64(math.Floor((elapsed - first) / railHeadway))
	if railServiceTime(first, track, service) > elapsed {
		service--
	}
	return service
}

func (r *Railway) ForEachTrain(elapsed float64, draw func(RailTrain)) {
	cruise := r.cruiseSpeed()
	length := r.EndX - r.StartX
	for track := range RailTrackCount {
		schedule := track
		if r.Conventional {
			schedule += RailTrackCount
		}
		stop := r.distanceFromEntry(track, r.StationCenter.X)
		approach := stop/cruise - railBrakeTime/2
		arrival := approach + railBrakeTime
		depart := arrival + railDwell
		first := r.firstDeparture(track)
		duration := r.arrivalTime(track, length)
		for service := latestRailService(elapsed, first, schedule); ; service-- {
			age := elapsed - railServiceTime(first, schedule, service)
			if age > duration {
				break
			}
			distance, speed := age*cruise, cruise
			kind := RailHighSpeed
			if r.Conventional {
				kind = RailConventionalPassenger
			}
			if r.Conventional && service%2 != 0 {
				kind = RailFreight
				if distance > length {
					continue
				}
			} else {
				switch {
				case age < approach:
				case age < arrival:
					left := arrival - age
					speed = cruise * left / railBrakeTime
					distance = stop - speed*left/2
				case age < depart:
					distance, speed = stop, 0
				case age < depart+railBrakeTime:
					away := age - depart
					speed = cruise * away / railBrakeTime
					distance = stop + speed*away/2
				default:
					distance = (age - railBrakeTime - railDwell) * cruise
				}
			}
			train := RailTrain{Pos: Vec2{X: r.StartX + distance, Y: r.TrackY(track)}, Speed: speed, Service: kind}
			if track != 0 {
				train.Pos.X = r.EndX - distance
				train.Heading = math.Pi
			}
			draw(train)
		}
	}
}

func (c RailCrossing) Gate(elapsed float64) RailGateState {
	var state RailGateState
	if !c.Level {
		return state
	}
	for track, window := range c.windows {
		if freight := c.freightWindows[track]; freight[1] > 0 {
			state.includeWindow(elapsed, track+RailTrackCount, window, 0)
			state.includeWindow(elapsed, track+RailTrackCount, freight, 1)
		} else {
			state.includeWindow(elapsed, track, window, -1)
		}
	}
	return state
}

func (s *RailGateState) includeWindow(elapsed float64, schedule int, window [2]float64, parity int64) {
	service := latestRailService(elapsed, window[0], schedule)
	if parity >= 0 && service&1 != parity {
		service--
	}
	phase := elapsed - railServiceTime(window[0], schedule, service)
	if phase >= window[1]+railReopen {
		return
	}
	s.Red = true
	s.Occupied = s.Occupied || phase >= railWarning && phase <= window[1]
	s.Lowered = math.Max(s.Lowered, Clamp(math.Min(phase/1.2, (window[1]+railReopen-phase)/railReopen), 0, 1))
}
