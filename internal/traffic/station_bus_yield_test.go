package traffic

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestCurbCarsYieldOnlyToActiveSafeStationMerge(t *testing.T) {
	for _, lw := range []float64{42, 52, 78} {
		for _, lanes := range []int{2, 4, 6, 8} {
			for _, heading := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
				for _, scenario := range []string{"yield", "already alongside", "inner lane", "opposite", "parked", "expired", "no request"} {
					if lanes == 2 && scenario == "inner lane" {
						continue
					}
					t.Run(fmt.Sprintf("width%.0f/lanes%d/h%.2f/%s", lw, lanes, heading, scenario), func(t *testing.T) {
						forward := sim.Vec2{X: math.Cos(heading), Y: math.Sin(heading)}
						right := sim.Vec2{X: -forward.Y, Y: forward.X}
						stop := sim.NewBusStop(0, "stop", sim.Vec2{}, heading, lanes, sim.RGBA(0, 0, 0, 255), lw, 10)
						w := &sim.World{Stops: []*sim.BusStop{stop}}
						bus := sim.NewBus(stop.Pos.X, stop.Pos.Y, heading)
						bus.StationMergeRequest = &sim.BusStationMergeRequest{StopID: 0, Departing: true, Remaining: .3}
						curb := (float64(lanes/2) - .5) * lw
						pos := forward.Mul(bus.Pos.Dot(forward) - 300).Add(right.Mul(curb))
						car := &sim.TrafficCar{Pos: pos, Heading: heading, Speed: 80, Length: 48, Width: 22, LaneIndex: lanes/2 - 1}
						switch scenario {
						case "already alongside":
							car.Pos = forward.Mul(bus.Pos.Dot(forward)).Add(right.Mul(curb))
						case "inner lane":
							car.LaneIndex--
							car.Pos = car.Pos.Sub(right.Mul(lw))
						case "opposite":
							car.Heading = heading + math.Pi
						case "parked":
							bus.DoorsOpen = true
						case "expired":
							bus.StationMergeRequest.Remaining = 0
						case "no request":
							bus.StationMergeRequest = nil
						}
						f := carFrame{car: car, buses: []*sim.Bus{bus}, world: w, laneWidth: lw}
						speed := (&TrafficManager{}).stationBusYieldSpeed(f, 90)
						if scenario == "yield" {
							if speed > 90 || car.YieldingToBus != bus || !car.IsBraking {
								t.Fatal("curb car did not create a station gap")
							}
							bus.StationMergeRequest = nil
							if (&TrafficManager{}).stationBusYieldSpeed(f, 90) != 90 || car.YieldingToBus != nil {
								t.Fatal("car did not resume after bus finished merge")
							}
						} else if speed != 90 || car.YieldingToBus != nil {
							t.Fatal("unrelated or too-close traffic was stopped")
						}
					})
				}
			}
		}
	}
}
