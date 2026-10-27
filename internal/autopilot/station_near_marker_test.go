package autopilot

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// The saved articulated bus was 3.77px short and 0.55px off the front marker.
// Its tiny local docking curve failed the trailer constraint before the
// already-arrived check could run. Reproduce that pose without personal saves.
func TestNearFrontMarkerCompletesDockingAndDeparture(t *testing.T) {
	for _, h := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		for model := sim.BusStandard; model < sim.BusModelCount; model++ {
			for _, berserk := range []bool{false, true} {
				for _, side := range []float64{-1, 1} {
					t.Run(fmt.Sprintf("h%.2f/model%d/berserk%v/side%.0f", h, model, berserk, side), func(t *testing.T) {
						const lw = 52.
						u := sim.Vec2{X: math.Cos(h), Y: math.Sin(h)}
						r := sim.Vec2{X: -u.Y, Y: u.X}
						w := &sim.World{Roads: []sim.RoadSegment{{P1: u.Mul(-3000), P2: u.Mul(3000), Lanes: 8, Type: sim.RoadBusBoulevard8Lane}}}
						w.RebuildIntersections()
						for i, x := range []float64{-1500, 0, 1500} {
							w.Stops = append(w.Stops, sim.NewBusStop(i, "test", u.Mul(x), h, 8, sim.RGBA(0, 0, 0, 255), lw))
						}
						stop := w.Stops[1]
						p := stop.FrontBerthPosition().Sub(u.Mul(3.773272562084)).Add(r.Mul(side * .545664083551))
						b := sim.NewBus(p.X, p.Y, h+.01975619484518721)
						b.SetModel(model)
						b.TrailerHeading = h + .053198839658695034
						ad := NewAutopilotDriver()
						ad.BerserkMode = berserk
						route := &sim.Route{StopIDs: []int{0, 1, 2}, CurrentIndex: 1}
						opened, closed, advanced := false, false, false
						for tick := 0; tick < 600; tick++ {
							th, br, st, hb, d, dock := ad.ComputeControls(b, route, w, nil, nil, lw, 1./60)
							b.Update(1./60, th, br, st, hb, d)
							if tick == 0 {
								if !dock || !b.DoorsOpen || b.Pos.Distance(p) > 1e-9 {
									t.Fatalf("arrival not accepted without snapping: dock%v doors%v pos%v", dock, b.DoorsOpen, b.Pos)
								}
								restored, err := ad.Snapshot().Restore(w)
								if err != nil {
									t.Fatal(err)
								}
								ad = restored
							}
							opened = opened || b.DoorsOpen
							closed = closed || (opened && !b.DoorsOpen && b.DoorProgress <= .05)
							advanced = advanced || route.GetCurrentTargetStopID() == 2
							if result := b.CheckBoundaries(w, lw); result.Zone == sim.ZoneHardBarrier {
								t.Fatalf("barrier tick%d %s", tick, result.ObstacleName)
							}
							if closed && advanced && b.Pos.Sub(p).Dot(u) > 60 {
								return
							}
						}
						t.Fatalf("door cycle or departure stuck: open%v closed%v advanced%v pos%v", opened, closed, advanced, b.Pos)
					})
				}
			}
		}
	}
}

func TestNearMarkerCompletionKeepsArrivalGuards(t *testing.T) {
	for _, name := range []string{"occupied", "too-fast", "misaligned", "too-far", "skip", "departing"} {
		t.Run(name, func(t *testing.T) {
			stop := sim.NewBusStop(0, "test", sim.Vec2{}, 0, 8, sim.RGBA(0, 0, 0, 255), 52)
			p := stop.FrontBerthPosition()
			b := sim.NewBus(p.X, p.Y, 0)
			b.SetModel(sim.BusArticulated)
			ad := NewAutopilotDriver()
			ad.BerserkMode = true
			f := autopilotFrame{bus: b, targetStop: stop, laneWidth: 52, remaining: 4}
			switch name {
			case "occupied":
				other := sim.NewBus(p.X, p.Y, 0)
				f.obstacles = []VehicleObstacle{ObserveBus(other)}
			case "too-fast":
				b.Speed = 30
				f.speedAbs = 30
			case "misaligned":
				b.Heading = .9 // Outside the permissible service heading window.
			case "too-far":
				b.Pos.X -= 350 // Both sections must actually be alongside the platform.
			case "skip":
				ad.CanSkipTargetStop = true
			case "departing":
				ad.State = AutopilotStateDeparting
			}
			in, _ := ad.stationDockControls(f)
			if in.isDocked || in.toggleDoors || ad.State == AutopilotStateDocked {
				t.Fatalf("unsafe arrival accepted: %+v", in)
			}
		})
	}
}
