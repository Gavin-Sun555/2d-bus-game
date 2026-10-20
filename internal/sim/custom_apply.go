package sim

import (
	"strings"
)

// ApplyCustomMapToWorld converts CustomMapData into World entities
func ApplyCustomMapToWorld(w *World, data *CustomMapData, laneWidth float64) {
	if w == nil || data == nil {
		return
	}

	w.Theme = MapThemeCustom
	w.Roads = make([]RoadSegment, 0, len(data.Roads))
	w.Buildings = make([]Building, 0, len(data.Buildings))
	w.Trees = make([]Tree, 0, len(data.Trees))
	w.Stops = make([]*BusStop, 0, len(data.Stops))
	w.Rivers = make([]River, 0, len(data.Rivers))
	w.Waters = make([]WaterBody, 0, len(data.Waters))
	w.Bridges = make([]Bridge, 0, len(data.Bridges))
	w.Roundabouts = make([]Roundabout, 0, len(data.Roundabouts))

	// 1. Waters
	for _, wd := range data.Waters {
		col := parseHexColor(wd.Color, RGBA(14, 116, 144, 210))
		shim := parseHexColor(wd.Shimmer, RGBA(103, 232, 249, 180))
		w.Waters = append(w.Waters, WaterBody{
			IsOcean:  wd.IsOcean,
			HasBeach: wd.HasBeach,
			IsLake:   wd.IsLake,
			Center:   Vec2{X: wd.Center[0], Y: wd.Center[1]},
			Width:    wd.Width,
			Height:   wd.Height,
			Color:    col,
			Shimmer:  shim,
			Name:     wd.Name,
		})
	}

	// 2. Rivers
	for _, rd := range data.Rivers {
		w.Rivers = append(w.Rivers, River{
			Y:      rd.Y,
			Height: rd.Height,
		})
	}

	// 3. Roads
	for _, r := range data.Roads {
		lanes := r.Lanes
		if lanes != 2 && lanes != 4 && lanes != 6 && lanes != 8 {
			lanes = 4
		}
		rType := RoadAvenue4Lane
		if lanes == 2 {
			rType = RoadStreet2Lane
		} else if lanes == 6 {
			rType = RoadExpressway6Lane
		}
		if lanes == 8 {
			rType = RoadBoulevard8Lane
			if r.BusLane {
				rType = RoadBusBoulevard8Lane
			}
		}
		w.Roads = append(w.Roads, RoadSegment{
			P1:       Vec2{X: r.P1[0], Y: r.P1[1]},
			P2:       Vec2{X: r.P2[0], Y: r.P2[1]},
			Lanes:    lanes,
			Type:     rType,
			HasLines: true,
			Name:     r.Name,
		})
	}

	// 4. Bridges
	for _, b := range data.Bridges {
		isCable := (strings.ToLower(b.Style) == "cable" || strings.ToLower(b.Style) == "suspension")
		isArch := (strings.ToLower(b.Style) == "arch")
		isOverpass := (strings.ToLower(b.Style) == "overpass")
		w.Bridges = append(w.Bridges, Bridge{
			Pos:        Vec2{X: b.Pos[0], Y: b.Pos[1]},
			Heading:    b.Heading,
			Length:     b.Length,
			Width:      b.Width,
			Name:       b.Name,
			IsCable:    isCable,
			IsArch:     isArch,
			IsOverpass: isOverpass,
		})
	}

	// 5. Roundabouts
	for _, rb := range data.Roundabouts {
		rbType := RoundaboutCivic
		switch strings.ToLower(rb.Type) {
		case "lighthouse":
			rbType = RoundaboutLighthouse
		case "island":
			rbType = RoundaboutIsland
		}
		w.Roundabouts = append(w.Roundabouts, Roundabout{
			Center:      Vec2{X: rb.Center[0], Y: rb.Center[1]},
			Radius:      rb.Radius,
			InnerRadius: rb.InnerRadius,
			Name:        rb.Name,
			Type:        rbType,
		})
	}

	// 6. Bus Stops
	for i, s := range data.Stops {
		stopID := s.ID
		if stopID < 0 {
			stopID = i
		}
		lanes := s.Lanes
		if lanes != 2 && lanes != 4 && lanes != 6 && lanes != 8 {
			lanes = 4
		}
		accent := parseHexColor(s.Color, RGBA(59, 130, 246, 255))
		cap := s.Capacity
		if cap <= 0 {
			cap = 10
		}
		w.Stops = append(w.Stops, NewBusStop(
			stopID,
			s.Name,
			Vec2{X: s.Pos[0], Y: s.Pos[1]},
			s.Heading,
			lanes,
			accent,
			laneWidth,
			cap,
		))
	}
	w.MatchStopRoadLanes(laneWidth)

	// 7. Buildings
	for _, b := range data.Buildings {
		col := parseHexColor(b.Color, RGBA(30, 41, 59, 255))
		roof := parseHexColor(b.Roof, RGBA(51, 65, 85, 255))
		w.Buildings = append(w.Buildings, Building{
			Pos:   Vec2{X: b.Pos[0], Y: b.Pos[1]},
			W:     b.Width,
			H:     b.Height,
			Color: col,
			Roof:  roof,
			Name:  b.Name,
		})
	}

	// 8. Trees
	for _, tr := range data.Trees {
		col := parseHexColor(tr.Color, RGBA(22, 101, 52, 255))
		rad := tr.Radius
		if rad <= 0 {
			rad = 15
		}
		w.Trees = append(w.Trees, Tree{
			Pos:    Vec2{X: tr.Pos[0], Y: tr.Pos[1]},
			Radius: rad,
			Color:  col,
		})
	}

	// Compute intersections dynamically
	w.RebuildIntersections()
}
