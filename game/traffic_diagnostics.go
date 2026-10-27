package game

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/autopilot"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"math"
	"os"
	"path/filepath"
	"time"
)

type greenWaitReport struct {
	CapturedAt        time.Time
	Map               sim.MapTheme
	BusID, TargetStop int
	Position          sim.Vec2
	Heading           float64
	Junction          sim.TrafficIntersection
	Diagnosis         autopilot.StopDiagnosis
}

// Retain the latest incident for each vehicle. Write only once per continuous
// wait, outside the physics/controller packages; no vehicle is forced to move.
func (g *Game) recordGreenWaits(dt float64) {
	if g.AIBusMgr == nil || g.TrafficLights == nil {
		return
	}
	if g.greenWaitTimers == nil {
		g.greenWaitTimers = make(map[int]float64)
	}
	for _, ab := range g.AIBusMgr.Buses {
		b := ab.Bus
		var intersection *sim.TrafficIntersection
		if b != nil && ab.Driver != nil && ab.Route != nil && math.Abs(b.Speed) < 1 && !b.DoorsOpen && ab.Driver.State != autopilot.AutopilotStateDocked {
			h := math.Round(b.Heading/(math.Pi/2)) * (math.Pi / 2)
			u := sim.UnitFromAngle(h)
			for _, it := range g.TrafficLights.Intersections {
				delta := it.Center.Sub(b.Pos)
				gap := delta.Dot(u) - it.GetStopLineDistForHeading(h, g.Settings.GetLaneWidth()) - b.Length/2
				if gap >= -5 && gap <= 60 && math.Abs(delta.X*u.Y-delta.Y*u.X) < it.ApproachRoadHalfWidth(h, g.Settings.GetLaneWidth()) && it.GetSignalForHeading(h, false) == sim.SignalGreen {
					intersection = it
					break
				}
			}
		}
		if intersection == nil {
			g.greenWaitTimers[ab.ID] = 0
			continue
		}
		previous := g.greenWaitTimers[ab.ID]
		g.greenWaitTimers[ab.ID] += dt
		if previous >= 1.5 || g.greenWaitTimers[ab.ID] < 1.5 {
			continue
		}
		obs := g.AIBusMgr.GetObstacles(ab.ID)
		obs = append(obs, autopilot.ObserveBus(g.Bus))
		for _, c := range g.TrafficMgr.GetActiveCars(g.Settings.TrafficDensity) {
			obs = append(obs, autopilot.VehicleObstacle{Pos: c.Pos, Heading: c.Heading, Speed: c.Speed, Length: c.Length, Width: c.Width, JunctionTurn: c.JunctionTurn, JunctionCorridor: c.JunctionCorridor})
		}
		report := greenWaitReport{CapturedAt: time.Now(), Map: g.SelectedMapTheme, BusID: ab.ID, TargetStop: ab.Route.GetCurrentTargetStopID(), Position: b.Pos, Heading: b.Heading, Junction: *intersection, Diagnosis: ab.Driver.DiagnoseStop(b, ab.Route, g.World, g.TrafficLights, obs, g.Settings.GetLaneWidth())}
		if g.greenWaitReports == nil {
			g.greenWaitReports = make(map[int]greenWaitReport)
		}
		g.greenWaitReports[ab.ID] = report
		if directory, err := os.UserCacheDir(); err == nil {
			_ = writeGreenWaitReports(filepath.Join(directory, "2d-bus-game", "green-wait.json"), g.greenWaitReports)
		}
	}
}
func writeGreenWaitReports(path string, reports map[int]greenWaitReport) error {
	data, err := json.MarshalIndent(reports, "", "  ")
	if err != nil {
		return err
	}
	if err = os.MkdirAll(filepath.Dir(path), 0755); err != nil {
		return err
	}
	temporary := path + ".tmp"
	if err = os.WriteFile(temporary, data, 0600); err != nil {
		return err
	}
	return os.Rename(temporary, path)
}
