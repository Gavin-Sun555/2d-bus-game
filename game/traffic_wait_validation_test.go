package game

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"testing"
)

// A progress test must distinguish a blocked controller from a legal queue.
// Only a stationary, physically connected queue ending at an actual red
// signal qualifies. A green leader or a perpendicular queue never excuses it.
func (g *Game) waitingInRedQueue() bool {
	if g.TrafficLights == nil || math.Abs(g.Bus.Speed) > 1 {
		return false
	}
	position, heading, length, width := g.Bus.Pos, g.Bus.Heading, g.Bus.Length, g.Bus.Width
	lane := -1
	for depth := 0; depth < 16; depth++ {
		u := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
		for _, it := range g.TrafficLights.Intersections {
			d := it.Center.Sub(position)
			gap := d.Dot(u) - it.GetStopLineDistForHeading(heading, g.Settings.GetLaneWidth()) - length/2
			if gap < -8 || gap > 40 || math.Abs(d.X*u.Y-d.Y*u.X) > it.ApproachRoadHalfWidth(heading, g.Settings.GetLaneWidth()) {
				continue
			}
			if lane >= 0 {
				left := lane == 0 && it.ApproachHasDedicatedLanesForHeading(heading)
				right := lane == 3 && it.ApproachLanes[it.GetApproachIndexForHeading(heading)] >= 8
				if !right && it.GetSignalForHeading(heading, left) == sim.SignalRed {
					return true
				}
			} else {
				diagnosis := g.AutopilotDriver.DiagnoseStop(g.Bus, g.Route, g.World, g.TrafficLights, g.obstacles, g.Settings.GetLaneWidth())
				if diagnosis.Limits["signal"] == 0 {
					return true
				}
			}
		}
		var leader *sim.TrafficCar
		best := math.Inf(1)
		for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			if math.Abs(c.Speed) > 1 || math.Cos(c.Heading-heading) < .98 {
				continue
			}
			d := c.Pos.Sub(position)
			along := d.Dot(u)
			if along <= length/2 || along > length/2+c.Length/2+100 || math.Abs(d.X*u.Y-d.Y*u.X) > (width+c.Width)/2+5 {
				continue
			}
			if along < best {
				best, leader = along, c
			}
		}
		if leader == nil {
			return false
		}
		position, heading, length, width, lane = leader.Pos, leader.Heading, leader.Length, leader.Width, leader.LaneIndex
	}
	return false
}

func TestRedQueueWaitValidationRequiresRealRedLeader(t *testing.T) {
	g := NewGame(1280, 720)
	g.Bus = sim.NewBus(-450, 26, 0)
	it := &sim.TrafficIntersection{IsMajor: true, ApproachLanes: [4]int{4, 4, 4, 4}, ApproachHasRoad: [4]bool{true, true, true, true}, ApproachIs4Lane: [4]bool{true, true, true, true}, Phase: sim.PhaseNSGreen}
	g.TrafficLights = &sim.TrafficLightSystem{Intersections: []*sim.TrafficIntersection{it}}
	line := it.GetStopLineDistForHeading(0, 52)
	front := &sim.TrafficCar{Pos: sim.Vec2{X: -line - 23 - 14, Y: 26}, Length: 46, Width: 22, LaneIndex: 0}
	g.Bus.Pos.X = front.Pos.X - 110
	g.TrafficMgr.Cars = []*sim.TrafficCar{front}
	if !g.waitingInRedQueue() {
		t.Fatal("real red queue not recognized")
	}
	it.Phase = sim.PhaseEWLeftGreen
	if g.waitingInRedQueue() {
		t.Fatal("green-light deadlock excused as legal red wait")
	}
	it.Phase = sim.PhaseNSGreen
	front.Heading = math.Pi / 2
	if g.waitingInRedQueue() {
		t.Fatal("perpendicular queue excused a blocked controller")
	}
}
