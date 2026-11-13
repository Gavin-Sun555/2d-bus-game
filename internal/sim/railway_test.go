package sim

import (
	"math"
	"testing"
)

func TestRailwayConnectsAuthoredStationsPastCityLimits(t *testing.T) {
	for _, theme := range []MapTheme{MapThemeMetropolis, MapThemeRiverValley} {
		for _, large := range []bool{false, true} {
			for _, brt := range []bool{false, true} {
				for _, lw := range []float64{42, 52, 78} {
					w := &World{LargeGrid: large, CenterBRT: brt}
					w.BuildMap(theme, lw)
					lines := w.Railways()
					wantLines := 1
					if theme == MapThemeMetropolis {
						wantLines = 2
					}
					if len(lines) != wantLines || lines[0].Conventional || RailTrackCount != 2 {
						t.Fatalf("%s: unexpected railway layout", theme.ShortName())
					}
					r := lines[0]
					if theme == MapThemeMetropolis {
						local := lines[1]
						stop := w.Stops[local.StationStopID]
						if !local.Conventional || !w.isMetropolisConventionalStop(stop) ||
							math.Cos(stop.Heading) >= 0 || math.Abs(local.StationCenter.X-stop.RoadCenter.X) > 1 ||
							local.StationCenter.Distance(stop.RoadCenter) > 550 || math.Abs(r.Y-local.Y) < 1000 {
							t.Fatal("conventional railway is not at the separate Railway Station Stop")
						}
						if StripStopDirection(stop.Name) != metropolisConventionalStopName ||
							StripStopDirection(w.Stops[r.StationStopID].Name) != metropolisHighSpeedStopName {
							t.Fatal("railway bus stops do not have their respective station names")
						}
					}
					if r == nil || math.Abs(w.Stops[r.StationStopID].RoadCenter.X-r.StationCenter.X) > 1 {
						t.Fatalf("%s: railway misses its station", theme.ShortName())
					}
					for _, road := range w.Roads {
						if r.StartX >= math.Min(road.P1.X, road.P2.X) || r.EndX <= math.Max(road.P1.X, road.P2.X) {
							t.Fatalf("%s: railway ends inside the road network", theme.ShortName())
						}
					}
					level, elevated := false, false
					for _, crossing := range r.Crossings {
						level = level || crossing.Level
						elevated = elevated || !crossing.Level
					}
					if !level || !elevated {
						t.Fatalf("%s large=%t BRT=%t width=%.0f: missing a crossing type", theme.ShortName(), large, brt, lw)
					}
					docked := false
					r.ForEachTrain(8, func(train RailTrain) {
						docked = docked || math.Abs(train.Pos.X-r.StationCenter.X) < .001
					})
					if !docked {
						t.Fatalf("%s: scheduled train did not stop at the platform", theme.ShortName())
					}
				}
			}
		}
	}
}

func TestRailGateProtectsTracksAcrossEntireTrainSchedule(t *testing.T) {
	w := &World{LargeGrid: true}
	w.BuildMap(MapThemeMetropolis, 52)
	// The large map's conventional railway is grade-separated. Add a quiet
	// road away from the ring to exercise conventional and freight gate timing.
	y := w.Railways()[1].Y
	w.Roads = append(w.Roads, RoadSegment{P1: Vec2{X: 1800, Y: y - 300}, P2: Vec2{X: 1800, Y: y + 300}, Lanes: 2})
	w.rebuildRailway(52)
	for _, r := range w.Railways() {
		verifyRailGateSchedule(t, w, r)
	}
}

func TestConventionalRailCrossingNearCentralRoundaboutIsElevated(t *testing.T) {
	w := &World{LargeGrid: true}
	w.BuildMap(MapThemeMetropolis, 52)
	for _, crossing := range w.Railways()[1].Crossings {
		if crossing.Center.X != 0 {
			continue
		}
		if crossing.Level || crossing.ViaductHalfSpan <= crossing.RoadHalfWidth {
			t.Fatal("the crossing immediately before the central roundabout needs a viaduct")
		}
		return
	}
	t.Fatal("missing crossing on the central avenue")
}

func verifyRailGateSchedule(t *testing.T, w *World, r *Railway) {
	t.Helper()
	for _, crossing := range r.Crossings {
		if !crossing.Level {
			continue
		}
		occupied, reopened := false, false
		for tick := 0; tick < 900*60; tick++ {
			now := float64(tick) / 60
			gate := crossing.Gate(now)
			r.ForEachTrain(now, func(train RailTrain) {
				if math.Abs(train.Pos.X-crossing.Center.X) <= crossing.RoadHalfWidth+ShoulderWidth+train.Length()/2 {
					occupied = true
					if !gate.Red || !gate.Occupied || gate.Lowered != 1 {
						t.Errorf("train at crossing with unprotected gate at %.3fs: %+v", now, gate)
					}
				}
			})
			reopened = reopened || occupied && !gate.Red && gate.Lowered == 0
		}
		if !occupied || !reopened {
			t.Fatal("crossing did not see a train and reopen during a full schedule")
		}
		now := crossing.windows[0][0] + 2
		tls := &TrafficLightSystem{RailElapsed: now}
		gateY := r.Y - r.HalfWidth - RailGateMargin
		body := OBB{Center: Vec2{X: crossing.Center.X - 15, Y: gateY - 20}, Heading: math.Pi / 2, HalfLen: 48, HalfWid: 18}
		before := body.Center.Sub(Vec2{Y: 40})
		shift, _ := w.RailGateCorrection(tls, before, body)
		if shift >= 0 || body.Center.Y+shift+body.HalfLen > gateY {
			t.Fatal("closed barrier let a road vehicle enter")
		}
		return
	}
	t.Fatal("no level crossing to verify")
}

func TestRailServiceIntervalsVaryWithinTwoToThreeMinutes(t *testing.T) {
	for track := 0; track < 2*RailTrackCount; track++ {
		const first = -37.
		intervals := make(map[int]bool)
		for service := int64(-10); service < 100; service++ {
			current := railServiceTime(first, track, service)
			next := railServiceTime(first, track, service+1)
			gap := next - current
			if gap < 120 || gap > 180 {
				t.Fatalf("track %d service %d: %.2fs interval", track, service, gap)
			}
			if got := latestRailService(next-.01, first, track); got != service {
				t.Fatalf("track %d at %.2fs: found service %d, want %d", track, next-.01, got, service)
			}
			intervals[int(gap)] = true
		}
		if len(intervals) < 10 {
			t.Fatalf("track %d: only %d distinct intervals", track, len(intervals))
		}
	}
}

func TestViaductSpansWideJunctionApron(t *testing.T) {
	const lw = 52.
	w := &World{LargeGrid: true, CenterBRT: true}
	w.BuildMap(MapThemeMetropolis, lw)
	r := w.Railway()
	widened := false
	for _, crossing := range r.Crossings {
		if crossing.Level {
			continue
		}
		for _, center := range w.Intersections {
			if math.Abs(center.X-crossing.Center.X) > 1 || w.IsAtRoundabout(center) {
				continue
			}
			junction := w.JunctionAt(center, lw)
			if math.Abs(center.Y-r.Y) > junction.HalfY+ShoulderWidth+r.HalfWidth ||
				junction.HalfX <= crossing.RoadHalfWidth {
				continue
			}
			widened = true
			if crossing.ViaductHalfSpan-47 <= junction.HalfX+ShoulderWidth {
				t.Fatalf("bridge pier falls inside the junction apron at %v", crossing.Center)
			}
		}
	}
	if !widened {
		t.Fatal("fixture has no viaduct over a widened junction apron")
	}
}

func TestHighSpeedRailUsesBothTracks(t *testing.T) {
	w := NewWorld(52, MapThemeMetropolis)
	r := w.Railway()
	var cruise, stationStop bool
	var directions [RailTrackCount]bool
	for now := 0.; now < 600; now += .25 {
		r.ForEachTrain(now, func(train RailTrain) {
			speed := train.Speed * railSpeedScale
			if train.Service != RailHighSpeed || speed < 0 || speed > 300+1e-6 {
				t.Fatalf("unexpected train class %d at %.2f km/h", train.Service, speed)
			}
			track := 0
			if train.Heading != 0 {
				track = 1
			}
			if train.Pos.Y != r.TrackY(track) {
				t.Fatal("train departed from its directional track")
			}
			directions[track] = true
			cruise = cruise || math.Abs(speed-300) < 1e-6
			stationStop = stationStop || train.Speed == 0 && train.Pos.X == r.StationCenter.X
		})
	}
	if !cruise || !stationStop || directions != [RailTrackCount]bool{true, true} {
		t.Fatal("high-speed trains must cruise, stop at the station, and run in both directions")
	}
}
