package game

import (
	"math"
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

// Run against a copy of a local save; never writes to it.
func TestSavedStationQueueReplay(t *testing.T) {
	path := os.Getenv("BUS_STATION_REPLAY_SAVE")
	if path == "" {
		t.Skip("BUS_STATION_REPLAY_SAVE not set")
	}
	g := NewGame(1280, 720)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	start := g.Bus.Pos
	target := g.Route.GetCurrentTargetStopID()
	stop := g.World.Stops[target]
	u := sim.Vec2{X: math.Cos(stop.Heading), Y: math.Sin(stop.Heading)}
	initialAIHealth := map[int]float64{}
	for _, a := range g.AIBusMgr.Buses {
		initialAIHealth[a.ID] = a.Bus.Durability
	}
	followerID := -1
	for _, a := range g.AIBusMgr.Buses {
		if a.Route.GetCurrentTargetStopID() == target && a.Bus.Pos.Distance(stop.Pos) < 500 {
			followerID = a.ID
			break
		}
	}
	entryTime, boardingTime := -1., -1.
	frontTime := 0.
	peak := 0.
	for tick := 0; tick < 2400; tick++ {
		if err := g.Update(); err != nil {
			t.Fatal(err)
		}
		peak = math.Max(peak, g.Bus.Speed)
		if frontTime == 0 && g.Bus.Pos.Sub(start).Dot(u) > 620 {
			frontTime = float64(tick) / 60
		}
		if tick%60 == 0 && tick < 660 {
			snap := g.AutopilotDriver.Snapshot()
			t.Logf("tick%d along%.1f lat%.1f speed%.1f target%.1f steer%.3f bend%.3f smooth%v run%.1f state%v shoulder%v", tick, g.Bus.Pos.Sub(start).Dot(u), g.Bus.Pos.Sub(start).Dot(sim.Vec2{X: -u.Y, Y: u.X}), g.Bus.Speed, snap.Public.LastTargetSpeed, g.Bus.SteeringAngle, sim.NormalizeAngle(g.Bus.Heading-g.Bus.TailHeading()), snap.SmoothDockDeparture, snap.DockDepartureRun, snap.Public.State, g.Bus.OnShoulder)
			for _, a := range g.AIBusMgr.Buses {
				if a.Bus.Pos.Distance(stop.Pos) < 500 {
					t.Logf("follower%d along%.1f lat%.1f speed%.1f state%v", a.ID, a.Bus.Pos.Sub(stop.Pos).Dot(u), a.Bus.Pos.Sub(stop.Pos).Dot(sim.Vec2{X: -u.Y, Y: u.X}), a.Bus.Speed, a.Driver.State)
				}
			}
		}
		for _, a := range g.AIBusMgr.Buses {
			if a.ID == followerID {
				if entryTime < 0 && math.Abs(a.Bus.Pos.Sub(stop.Pos).Dot(sim.Vec2{X: -u.Y, Y: u.X})) < 10 {
					entryTime = float64(tick) / 60
				}
				if boardingTime < 0 && a.Driver.DockedStopID == target && a.Bus.DoorsOpen {
					boardingTime = float64(tick) / 60
				}
			}
			if a.Bus.Durability < initialAIHealth[a.ID] {
				t.Fatalf("AI%d health lost", a.ID)
			}
		}
		if g.Bus.Durability < 100 {
			t.Fatalf("player health%.1f", g.Bus.Durability)
		}
	}
	t.Logf("620px including doors %.2fs peak%.1f follower%d entered%.2fs boarded%.2fs", frontTime, peak, followerID, entryTime, boardingTime)
	if followerID < 0 || entryTime < 0 || entryTime > 3 || boardingTime < 0 || boardingTime > 8 {
		t.Fatal("rear bus did not enter and board promptly")
	}
	if frontTime == 0 || frontTime > 8 {
		t.Fatalf("slow departure %.2fs", frontTime)
	}
}
