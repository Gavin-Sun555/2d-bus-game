package sim

import (
	"fmt"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
)

func (r *Route) GetRouteDescription(world *World) string {
	if !r.cachedDescDirty && r.cachedDesc != "" {
		return r.cachedDesc
	}
	if len(r.StopIDs) == 0 {
		r.cachedDesc = "未规划站点 (点击站点图标添加)"
		r.cachedDescDirty = false
		return r.cachedDesc
	}
	desc := ""
	for i, id := range r.StopIDs {
		prefix := " -> "
		if i == 0 {
			prefix = ""
		}
		if i == r.CurrentIndex {
			desc += fmt.Sprintf("%s[%s ★]", prefix, world.Stops[id].Name)
		} else {
			desc += fmt.Sprintf("%s%s", prefix, world.Stops[id].Name)
		}
	}
	r.cachedDesc = desc
	r.cachedDescDirty = false
	return desc
}

// GetFormattedRouteLines generates wrapped lines of route stops that strictly fit within maxWidth.
// Long routes are wrapped onto multiple lines with clear arrow indicators (➔) and sequence numbering.
func (r *Route) GetFormattedRouteLines(world *World, maxWidth float64, fontSize float64) []string {
	if world == nil || len(r.StopIDs) == 0 {
		return []string{fitRouteLine(locale.T("未规划站点 (点击地图站点图标添加)", "No stations planned (Click station icons to add)"), maxWidth, fontSize)}
	}

	prefix := locale.Tf("当前路线 (%d站): ", "Active Route (%d stops): ", len(r.StopIDs))

	type stopItem struct {
		display string
		index   int
	}

	isClosed := (len(r.StopIDs) >= 2 && r.StopIDs[0] == r.StopIDs[len(r.StopIDs)-1])
	items := make([]stopItem, 0, len(r.StopIDs))

	for i, stopID := range r.StopIDs {
		name := fmt.Sprintf("#%d", stopID)
		if stopID >= 0 && stopID < len(world.Stops) {
			name = world.Stops[stopID].CleanName()
		}
		if i == r.CurrentIndex {
			name = fmt.Sprintf("[%s ★]", name)
		} else if isClosed && i == len(r.StopIDs)-1 {
			name = fmt.Sprintf("%s(终点)", name)
		}
		items = append(items, stopItem{
			display: fmt.Sprintf("%d.%s", i+1, name),
			index:   i,
		})
	}

	maxLines := 3
	lines := make([]string, 0, 3)
	currentLine := prefix

	for i := 0; i < len(items); i++ {
		seg := items[i].display
		var candidate string
		if currentLine == prefix || strings.HasSuffix(currentLine, "↳ ") {
			candidate = currentLine + seg
		} else {
			candidate = currentLine + " ➔ " + seg
		}

		w, _ := textui.MeasureText(candidate, fontSize)
		if w <= maxWidth {
			currentLine = candidate
			continue
		}

		// candidate exceeds maxWidth: need to wrap or summarize
		if len(lines)+1 >= maxLines {
			// On the last line: summarize remaining stops to avoid overflowing
			remaining := len(items) - i
			lastItem := items[len(items)-1]
			summary := locale.Tf(" ➔ ... (+%d站) ➔ %s", " ➔ ... (+%d) ➔ %s", remaining, lastItem.display)

			// Ensure currentLine + summary fits within maxWidth
			for {
				sw, _ := textui.MeasureText(currentLine+summary, fontSize)
				if sw <= maxWidth || !strings.Contains(currentLine, " ➔ ") {
					break
				}
				lastIdx := strings.LastIndex(currentLine, " ➔ ")
				currentLine = currentLine[:lastIdx]
			}
			if width, _ := textui.MeasureText(currentLine+summary, fontSize); width > maxWidth {
				// The final remaining stop can itself be too wide. Keep the
				// destination summary instead of appending it to an oversized line.
				currentLine = "   ↳ "
			}
			currentLine += summary
			break
		}

		// Wrap to new line
		lines = append(lines, currentLine)
		currentLine = "   ↳ " + seg
	}

	if currentLine != "" {
		lines = append(lines, currentLine)
	}

	for i := range lines {
		lines[i] = fitRouteLine(lines[i], maxWidth, fontSize)
	}
	return lines
}

// Single long names and compact summaries must fit even with fallback fonts.
func fitRouteLine(line string, maxWidth, fontSize float64) string {
	if width, _ := textui.MeasureText(line, fontSize); width <= maxWidth {
		return line
	}
	suffix := "..."
	if width, _ := textui.MeasureText(suffix, fontSize); width > maxWidth {
		return ""
	}
	runes := []rune(line)
	low, high := 0, len(runes)
	for low < high {
		mid := (low + high + 1) / 2
		if width, _ := textui.MeasureText(string(runes[:mid])+suffix, fontSize); width <= maxWidth {
			low = mid
		} else {
			high = mid - 1
		}
	}
	return string(runes[:low]) + suffix
}
