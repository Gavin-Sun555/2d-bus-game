package sim

import (
	"fmt"
	"math"
	"strings"
)

type landmarkName struct{ zh, en string }

var blockLandmarkAreas = [...]landmarkName{
	{"松林", "Pinewood"}, {"银杏", "Ginkgo"}, {"青石", "Bluestone"}, {"云锦", "Cloudbrocade"},
	{"长宁", "Changning"}, {"桂花", "Osmanthus"}, {"明德", "Mingde"}, {"槐荫", "Locust Grove"},
	{"白鹭", "Egret"}, {"景明", "Jingming"}, {"东篱", "Eastgarden"}, {"新桥", "Newbridge"},
	{"榆林", "Elmwood"}, {"芳草", "Meadow"}, {"晴川", "Clearwater"}, {"锦绣", "Brocade"},
	{"翠竹", "Green Bamboo"}, {"柳湾", "Willow Bend"}, {"玉泉", "Jade Spring"}, {"晨光", "Morning Light"},
	{"石榴", "Pomegranate"}, {"杏花", "Apricot Blossom"}, {"兰亭", "Orchid Pavilion"}, {"望月", "Moonrise"},
	{"棠溪", "Tang Creek"}, {"桃源", "Peach Valley"}, {"和乐", "Hele"}, {"澄明", "Chengming"},
	{"星河", "Star River"}, {"紫藤", "Wisteria"}, {"瑞安", "Ruian"}, {"朝阳", "Sunrise"},
}

var blockLandmarks = [...]landmarkName{
	{"图书馆", "Library"}, {"社区卫生中心", "Health Centre"}, {"农贸市场", "Market"}, {"中学", "Secondary School"},
	{"邮局", "Post Office"}, {"体育馆", "Sports Hall"}, {"文化广场", "Cultural Square"}, {"公园", "Park"},
	{"剧院", "Theatre"}, {"少年宫", "Youth Centre"}, {"花卉市场", "Flower Market"}, {"美术馆", "Art Gallery"},
	{"书城", "Book Centre"}, {"社区服务中心", "Community Centre"}, {"商业街", "Shopping Street"}, {"居民区", "Residences"},
}

func stopDirectionLabel(heading float64) (string, string) {
	u := UnitFromAngle(heading)
	if math.Abs(u.X) > .5 {
		if u.X > 0 {
			return "东行", "Eastbound"
		}
		return "西行", "Westbound"
	}
	if u.Y < 0 {
		return "北行", "Northbound"
	}
	return "南行", "Southbound"
}

func splitBilingualStopName(name string) (zh, en string, hasEn bool) {
	if idx := strings.Index(name, " ("); idx > 0 && strings.HasSuffix(name, ")") {
		return strings.TrimSpace(name[:idx]), strings.TrimSpace(name[idx+2 : len(name)-1]), true
	}
	return strings.TrimSpace(name), "", false
}

// StripStopDirection returns the base station name without any cardinal direction suffix.
func StripStopDirection(name string) string {
	zh, en, hasEn := splitBilingualStopName(name)
	isBlock := strings.HasPrefix(zh, "街区")
	for _, dir := range []string{"东行", "西行", "南行", "北行"} {
		if strings.HasSuffix(zh, "·"+dir+"站") {
			zh = strings.TrimSuffix(zh, "·"+dir+"站")
			if isBlock {
				zh += "站"
			}
			break
		}
		if strings.HasSuffix(zh, "·"+dir) {
			zh = strings.TrimSuffix(zh, "·"+dir)
			break
		}
	}
	for _, dir := range []string{"Eastbound", "Westbound", "Southbound", "Northbound"} {
		if strings.HasSuffix(en, " "+dir) {
			en = strings.TrimSuffix(en, " "+dir)
			break
		}
	}
	if hasEn {
		return fmt.Sprintf("%s (%s)", zh, en)
	}
	return zh
}

// WithStopDirection appends the cardinal travel direction of heading to a station name.
func WithStopDirection(name string, heading float64) string {
	base := StripStopDirection(name)
	zh, en, hasEn := splitBilingualStopName(base)
	dirZh, dirEn := stopDirectionLabel(heading)
	if strings.HasPrefix(zh, "街区") && strings.HasSuffix(zh, "站") {
		zh = strings.TrimSuffix(zh, "站") + "·" + dirZh + "站"
	} else {
		zh = zh + "·" + dirZh
	}
	if hasEn {
		return fmt.Sprintf("%s (%s %s)", zh, en, dirEn)
	}
	return zh
}

func shouldShareMajorLandmarkName(theme MapTheme, id int) bool {
	_ = theme
	switch id {
	case 0, 3, 4, 5, 7, 11:
		return true
	default:
		return false
	}
}

func directionalBlockStopName(id int, heading float64) string {
	direction, en := stopDirectionLabel(heading)
	return fmt.Sprintf("街区%d·%s站 (Block %d %s)", id+1, direction, id+1, en)
}

func blockStopName(theme MapTheme, id int, heading float64) string {
	if (id+int(theme))%5 == 0 {
		return directionalBlockStopName(id, heading)
	}
	// Permute the combinations by station ID. This varies the names without
	// consuming simulation randomness or changing a name when the city reloads.
	index := (id*173 + int(theme)*59) % (len(blockLandmarkAreas) * len(blockLandmarks))
	area, place := blockLandmarkAreas[index/len(blockLandmarks)], blockLandmarks[index%len(blockLandmarks)]
	return fmt.Sprintf("%s%s (%s %s)", area.zh, place.zh, area.en, place.en)
}

func (w *World) syncPairedStopNames(lw float64) {
	if w == nil || len(w.Stops) < 2 {
		return
	}
	paired := make([]bool, len(w.Stops))
	pairStops := func(a, b *BusStop) {
		if a == nil || b == nil || a == b {
			return
		}
		primary, secondary := a, b
		if b.ID < a.ID {
			primary, secondary = b, a
		}
		if primary.CenterBRT || secondary.CenterBRT {
			base := StripStopDirection(primary.Name)
			primary.Name = WithStopDirection(base, primary.Heading)
			secondary.Name = WithStopDirection(base, secondary.Heading)
			primary.cachedInfoText = ""
			secondary.cachedInfoText = ""
			return
		}
		if w.Theme != MapThemeCustom && primary.ID < 12 && secondary.ID >= 12 && shouldShareMajorLandmarkName(w.Theme, primary.ID) {
			base := StripStopDirection(primary.Name)
			primary.Name = WithStopDirection(base, primary.Heading)
			secondary.Name = WithStopDirection(base, secondary.Heading)
			primary.cachedInfoText = ""
			secondary.cachedInfoText = ""
			return
		}
		if StripStopDirection(primary.Name) == StripStopDirection(secondary.Name) {
			base := StripStopDirection(primary.Name)
			primary.Name = WithStopDirection(base, primary.Heading)
			secondary.Name = WithStopDirection(base, secondary.Heading)
			primary.cachedInfoText = ""
			secondary.cachedInfoText = ""
		}
	}

	for _, block := range w.EligibleBlockStops(lw) {
		h0 := NormalizeAngle(block.Heading)
		h1 := NormalizeAngle(block.Heading + math.Pi)
		ew := math.Abs(math.Cos(h0)) > .5
		var s0, s1 *BusStop
		for _, s := range w.Stops {
			along, cross := s.RoadCenter.Y, s.RoadCenter.X
			coord := block.Center.X
			if ew {
				along, cross = s.RoadCenter.X, s.RoadCenter.Y
				coord = block.Center.Y
			}
			if math.Abs(cross-coord) < 1 && along > block.Start && along < block.End {
				if math.Cos(s.Heading-h0) > .99 && s0 == nil {
					s0 = s
				} else if math.Cos(s.Heading-h1) > .99 && s1 == nil {
					s1 = s
				}
			}
		}
		if s0 != nil && s1 != nil {
			if s0.ID >= 0 && s0.ID < len(paired) {
				paired[s0.ID] = true
			}
			if s1.ID >= 0 && s1.ID < len(paired) {
				paired[s1.ID] = true
			}
			pairStops(s0, s1)
		}
	}

	maxAlongGap := 260.0 * w.GridScale()
	for i, s0 := range w.Stops {
		if i < len(paired) && paired[i] {
			continue
		}
		ew := math.Abs(math.Cos(s0.Heading)) > .5
		var best *BusStop
		bestDist := maxAlongGap
		for j, s1 := range w.Stops {
			if i == j || (j < len(paired) && paired[j]) {
				continue
			}
			if math.Cos(s0.Heading-s1.Heading) > -0.98 {
				continue
			}
			cross := math.Abs(s0.RoadCenter.X - s1.RoadCenter.X)
			along := math.Abs(s0.RoadCenter.Y - s1.RoadCenter.Y)
			if ew {
				cross = math.Abs(s0.RoadCenter.Y - s1.RoadCenter.Y)
				along = math.Abs(s0.RoadCenter.X - s1.RoadCenter.X)
			}
			if cross < 2.0 && along < bestDist {
				best = s1
				bestDist = along
			}
		}
		if best != nil {
			if i < len(paired) {
				paired[i] = true
			}
			if best.ID >= 0 && best.ID < len(paired) {
				paired[best.ID] = true
			}
			pairStops(s0, best)
		}
	}
}

// RefreshGeneratedStopNames upgrades built-in stop labels in older saves.
func (w *World) RefreshGeneratedStopNames() {
	if w.Theme == MapThemeCustom {
		return
	}
	w.refreshMetropolisRailStopNames()
	for _, stop := range w.Stops {
		if stop.ID >= 48 && stop.Name == directionalBlockStopName(stop.ID, stop.Heading) {
			stop.Name = blockStopName(w.Theme, stop.ID, stop.Heading)
			stop.cachedInfoText = ""
		}
	}
}
