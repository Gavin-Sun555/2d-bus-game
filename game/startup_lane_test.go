package game

import (
	"fmt"
	"math"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestBerserkStartupDepartureStaysOnOwnSide(t *testing.T) {
	for _, density := range []int{1, 2, 3, 4} {
		for _, width := range []int{0, 1, 2, 3} {
			for _, seed := range []int64{5, 500, 503} {
				t.Run(fmt.Sprintf("density%d/width%d/seed%d", density, width, seed), func(t *testing.T) {
					sim.SeedSimulation(seed)
					g := NewGame(1280, 720)
					g.Settings.LaneWidthLevel = width
					g.Settings.AutopilotEnabled = false
					g.Settings.AutopilotBerserk = true
					g.Settings.AutopilotBerserkMaxSpeed = 120
					g.Settings.TrafficDensity = density
					g.Settings.AIBusCount = 4
					g.EnterCity(sim.MapThemeMetropolis)
					for tick := 0; tick < 240; tick++ {
						g.Update()
					}
					g.Settings.AutopilotEnabled = true
					g.AutopilotDriver.Reset()
					stop := g.World.Stops[g.Route.StopIDs[0]]
					forward := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
					right := sim.Vec2{X: -forward.Y, Y: forward.X}
					start := g.Bus.Pos
					for tick := 0; tick < 720; tick++ {
						if err := g.Update(); err != nil {
							t.Fatal(err)
						}
						along := g.Bus.Pos.Sub(start).Dot(forward)
						yaw := sim.NormalizeAngle(g.Bus.Heading - stop.Heading)
						clearance := g.Bus.Pos.Sub(stop.RoadCenter).Dot(right) - g.Bus.Length/2*math.Abs(math.Sin(yaw)) - g.Bus.Width/2*math.Abs(math.Cos(yaw))
						if clearance < 0 {
							t.Fatalf("crossed centerline t%.2f along%.1f pos%v yaw%.2f v%.1f clearance%.1f offset%.1f target%.1f path%v", float64(tick)/60, along, g.Bus.Pos, yaw, g.Bus.Speed, clearance, g.AutopilotDriver.LaneOffset, g.AutopilotDriver.LastTargetSpeed, g.AutopilotDriver.DrivePath()[:4])
						}
						if along > 800 {
							return
						}
					}
					t.Fatalf("failed to depart, pos%v v%.1f", g.Bus.Pos, g.Bus.Speed)
				})
			}
		}
	}
}
