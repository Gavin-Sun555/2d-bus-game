package traffic

import (
	"math"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

type initialRun struct {
	start, end, direction sim.Vec2
	at, length            float64
	target                int
}

// Only new/replanned services use this dispatcher. Live vehicles are never
// repositioned by count/speed/model edits, and save loading does not call it.
func distributeInitialLine(buses []*AIBus, w *sim.World, lw float64, occupied []*sim.Bus, cars []*sim.TrafficCar) {
	if len(buses) == 0 {
		return
	}
	route := buses[0].Route
	runs, total := initialRouteRuns(route, w, lw)
	others := append([]*sim.Bus(nil), occupied...)
	for slot, b := range buses {
		ideal := total * (float64(slot) + .5) / float64(len(buses))
		placed := false
		if total > 0 {
			type choice struct {
				run       initialRun
				at, delta float64
			}
			choices := make([]choice, 0, len(runs))
			for _, run := range runs {
				lo, hi := run.at+b.Bus.OverallLength()-b.Bus.Length/2+24, run.at+run.length-b.Bus.Length/2-24
				if hi < lo {
					continue
				}
				at := sim.Clamp(ideal, lo, hi)
				gap := func(s float64) float64 { d := math.Abs(s - ideal); return math.Min(d, total-d) }
				for _, offset := range []float64{-total, total} {
					s := sim.Clamp(ideal+offset, lo, hi)
					if gap(s) < gap(at) {
						at = s
					}
				}
				choices = append(choices, choice{run, at, gap(at)})
			}
			sort.SliceStable(choices, func(i, j int) bool { return choices[i].delta < choices[j].delta })
			for _, c := range choices {
				for shift := 0.; shift <= 1024 && !placed; shift += 32 {
					for _, sign := range []float64{1, -1} {
						if shift == 0 && sign < 0 {
							continue
						}
						along := c.at - c.run.at + shift*sign
						if along < b.Bus.OverallLength()-b.Bus.Length/2+24 || along > c.run.length-b.Bus.Length/2-24 {
							continue
						}
						candidate := *b.Bus
						candidate.Pos = c.run.start.Add(c.run.direction.Mul(along))
						candidate.Heading = math.Atan2(c.run.direction.Y, c.run.direction.X)
						candidate.ResetArticulation()
						if !safeInitialSpawn(&candidate, w, lw, others, cars) {
							continue
						}
						b.Route.CurrentIndex = c.run.target
						*b.Bus = candidate
						b.Bus.Speed, b.Bus.SteeringAngle = 0, 0
						b.Bus.DoorsOpen, b.Bus.DoorProgress = false, 0
						placed = b.Driver.PrepareInitialLeg(b.Bus, w, b.Route, lw)
						if placed {
							break
						}
					}
				}
				if placed {
					break
				}
			}
		}
		if !placed {
			// Degenerate/very short routes may have no safe straight roadway.
			// Keep the existing physical station queue instead of overlapping.
			b.Route.CurrentIndex = 0
			stop := w.Stops[b.Route.StopIDs[0]]
			queueObstacles := append([]*sim.Bus(nil), others...)
			for _, car := range cars {
				if car == nil {
					continue
				}
				body := sim.NewBus(car.Pos.X, car.Pos.Y, car.Heading)
				body.Length, body.Width = car.Length, car.Width
				body.Speed = car.Speed
				queueObstacles = append(queueObstacles, body)
			}
			sim.PlaceBusInStationQueue(b.Bus, stop, queueObstacles, lw)
			b.Driver.Reset()
			b.Driver.QueueAtStation(stop.ID)
		}
		others = append(others, b.Bus)
	}
}

func initialRouteRuns(route *sim.Route, w *sim.World, lw float64) ([]initialRun, float64) {
	if route == nil || w == nil || len(route.StopIDs) < 2 {
		return nil, 0
	}
	n := len(route.StopIDs)
	closed := route.StopIDs[0] == route.StopIDs[n-1]
	if closed {
		n--
	}
	if n < 2 {
		return nil, 0
	}
	var runs []initialRun
	total := 0.
	ghost := sim.NewBus(0, 0, 0)
	ghost.SetModel(sim.BusArticulated)
	for from := 0; from < n; from++ {
		target := (from + 1) % n
		if closed && from == n-1 {
			target = n
		}
		r := *route
		r.CurrentIndex = target
		driver := autopilot.NewAutopilotDriver()
		if !driver.PrepareInitialLeg(ghost, w, &r, lw) {
			continue
		}
		path := driver.DrivePath()
		for i := 1; i < len(path); i++ {
			d := path[i].Sub(path[i-1])
			length := d.Length()
			if length < 1e-6 {
				continue
			}
			u := d.Mul(1 / length)
			if math.Abs(u.X) < 1e-7 || math.Abs(u.Y) < 1e-7 {
				if len(runs) > 0 && runs[len(runs)-1].target == target && runs[len(runs)-1].end.Distance(path[i-1]) < 1e-5 && runs[len(runs)-1].direction.Dot(u) > 1-1e-10 {
					last := &runs[len(runs)-1]
					last.end = path[i]
					last.length += length
				} else {
					runs = append(runs, initialRun{path[i-1], path[i], u, total, length, target})
				}
			}
			total += length
		}
	}
	return runs, total
}

func safeInitialSpawn(bus *sim.Bus, w *sim.World, lw float64, others []*sim.Bus, cars []*sim.TrafficCar) bool {
	if w.InsideJunction(bus.Pos, lw, bus.OverallLength()+32) || bus.CheckBoundaries(w, lw).Zone != sim.ZoneRoad {
		return false
	}
	for _, rb := range w.Roundabouts {
		if bus.Pos.Distance(rb.Center) < rb.Radius+bus.OverallLength()+64 {
			return false
		}
	}
	u := sim.UnitFromAngle(bus.Heading)
	right := sim.Vec2{X: -u.Y, Y: u.X}
	rear := bus.OverallLength() - bus.Length/2
	clear := func(pos sim.Vec2, h, length, width, speed, brake float64) bool {
		d := pos.Sub(bus.Pos)
		reach := bus.OverallLength() + length + math.Abs(speed)*math.Abs(speed)/(2*brake) + 80
		if d.Dot(d) > reach*reach {
			return true
		}
		body := sim.OBB{Center: pos, Heading: h, HalfLen: length/2 + 12, HalfWid: width/2 + 4}
		if hit, _, _ := bus.CollidesBody(body); hit {
			return false
		}
		yaw := h - bus.Heading
		if d.Dot(u) < 0 && math.Cos(yaw) > .9 && math.Abs(d.Dot(right)) < (bus.Width+width)/2+8 {
			frontExtent := length/2*math.Abs(math.Cos(yaw)) + width/2*math.Abs(math.Sin(yaw))
			gap := -d.Dot(u) - rear - frontExtent
			if gap < speed*speed/(2*brake)+math.Max(24, math.Abs(speed)*.2) {
				return false
			}
		}
		return true
	}
	for _, other := range others {
		if other == nil {
			continue
		}
		body := other.BodyEnvelope()
		if !clear(body.Center, body.Heading, body.HalfLen*2, body.HalfWid*2, other.Speed, 135) {
			return false
		}
	}
	for _, car := range cars {
		if car != nil && !clear(car.Pos, car.Heading, car.Length, car.Width, car.Speed, 220) {
			return false
		}
	}
	return true
}
