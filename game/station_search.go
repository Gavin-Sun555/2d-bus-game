package game

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/platforminput"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

const stationSearchRows = 6

type stationSearch struct {
	stationTextInput
	query              string
	results            []*sim.BusStop
	selected, firstRow int
}

type stationSearchLayout struct {
	panel, input, close sim.Rect
}

func (g *Game) stationSearchLayout() stationSearchLayout {
	w := math.Min(650, float64(g.Width)-32)
	const h = 398.
	x, y := (float64(g.Width)-w)/2, (float64(g.Height)-h)/2
	return stationSearchLayout{
		panel: sim.Rect{X: x, Y: y, W: w, H: h},
		input: sim.Rect{X: x + 22, Y: y + 62, W: w - 44, H: 42},
		close: sim.Rect{X: x + w - 112, Y: y + 18, W: 90, H: 30},
	}
}

func (b stationSearchLayout) row(i int) sim.Rect {
	return sim.Rect{X: b.input.X, Y: b.panel.Y + 141 + float64(i)*36, W: b.input.W, H: 33}
}

func (g *Game) openStationSearch() {
	if !g.IsPlanMode || g.World == nil {
		return
	}
	s := &stationSearch{}
	s.initIME(func(int) sim.Rect { return g.stationSearchLayout().input })
	g.stationSearch = s
	g.isDragging = false
}

func (g *Game) closeStationSearch() {
	if g.stationSearch != nil {
		g.stationSearch.ime.Cancel()
		g.stationSearch = nil
		platforminput.ResetScrollAndPinch()
	}
}

func (s *stationSearch) refresh(stops []*sim.BusStop) {
	query := strings.Join(strings.Fields(strings.ToLower(s.fields[0])), " ")
	if query == s.query {
		return
	}
	s.query = query
	s.results = s.results[:0]
	s.firstRow, s.selected = 0, 0
	if query == "" {
		return
	}
	terms := strings.Fields(query)
	for _, stop := range stops {
		if stop == nil {
			continue
		}
		names := strings.ToLower(strings.Join([]string{stop.Name, stop.CustomNameZH, stop.CustomNameEN}, " "))
		matched := true
		for _, term := range terms {
			if !strings.Contains(names, term) {
				matched = false
				break
			}
		}
		if matched {
			s.results = append(s.results, stop)
		}
	}
}

func (s *stationSearch) moveSelection(step int) {
	if len(s.results) == 0 {
		return
	}
	s.selected = max(0, min(s.selected+step, len(s.results)-1))
	s.firstRow = min(s.firstRow, s.selected)
	s.firstRow = max(s.firstRow, s.selected-stationSearchRows+1)
}

func (g *Game) focusSearchedStation(stop *sim.BusStop) {
	g.Camera.Pos, g.Camera.TargetPos = stop.ShelterPos, stop.ShelterPos
	g.Camera.Zoom = max(g.Camera.Zoom, .42)
	g.Camera.TargetZoom = g.Camera.Zoom
	g.pinchTouchActive = false
	g.pinchLastTouchDist = 0
	g.closeStationSearch()
	g.HUD.AddToast(locale.Tf("已定位 #%d %s", "Located #%d %s", stop.ID+1, stop.CleanName()), true)
}

func (g *Game) updateStationSearch() {
	s := g.stationSearch
	if !g.IsPlanMode || g.World == nil {
		g.closeStationSearch()
		return
	}
	if !ebiten.IsFocused() {
		s.ime.Confirm()
		return
	}
	handled := s.readInput()
	if !handled {
		s.editKeys()
	}
	s.refresh(g.World.Stops)
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		p := sim.Vec2{X: float64(x), Y: float64(y)}
		b := g.stationSearchLayout()
		if b.close.Contains(p) {
			g.closeStationSearch()
			return
		}
		for row := 0; row < stationSearchRows && s.firstRow+row < len(s.results); row++ {
			if b.row(row).Contains(p) {
				g.focusSearchedStation(s.results[s.firstRow+row])
				return
			}
		}
	}
	if handled {
		return
	}
	switch {
	case inpututil.IsKeyJustPressed(ebiten.KeyEscape):
		g.closeStationSearch()
	case repeatedKey(ebiten.KeyUp):
		s.moveSelection(-1)
	case repeatedKey(ebiten.KeyDown):
		s.moveSelection(1)
	case inpututil.IsKeyJustPressed(ebiten.KeyEnter) || inpututil.IsKeyJustPressed(ebiten.KeyNumpadEnter):
		if len(s.results) > 0 {
			g.focusSearchedStation(s.results[s.selected])
		}
	default:
		_, wheel := ebiten.Wheel()
		if wheel != 0 {
			step := int(math.Copysign(math.Max(1, math.Abs(wheel)), -wheel))
			s.moveSelection(step)
		}
	}
}

func (g *Game) drawStationSearch(dst *ebiten.Image) {
	s := g.stationSearch
	if s == nil {
		return
	}
	b := g.stationSearchLayout()
	p := b.panel
	muted := sim.RGBA(148, 163, 184, 255)
	vector.DrawFilledRect(dst, 0, 0, float32(g.Width), float32(g.Height), sim.RGBA(0, 0, 0, 165), false)
	vector.DrawFilledRect(dst, float32(p.X), float32(p.Y), float32(p.W), float32(p.H), sim.RGBA(15, 23, 42, 255), true)
	vector.StrokeRect(dst, float32(p.X), float32(p.Y), float32(p.W), float32(p.H), 2, sim.RGBA(56, 189, 248, 255), true)
	textui.DrawText(dst, locale.T("搜索站名", "Search Stations"), b.input.X, p.Y+20, 22, color.White)
	drawStopEditorButton(dst, b.close, locale.T("关闭 [Esc]", "Close [Esc]"), sim.RGBA(30, 41, 59, 255))
	s.drawField(dst, b.input, 0)
	status := locale.T("输入中文或英文站名", "Type a station name in Chinese or English")
	if s.query != "" {
		if len(s.results) == 0 {
			status = locale.T("当前地图没有匹配的站点", "No matching stations on this map")
		} else {
			status = locale.Tf("%d 个站点 · ↑↓ 选择 · Enter 定位", "%d stations · ↑↓ Select · Enter to locate", len(s.results))
		}
	}
	if s.err != "" {
		status = s.err
	}
	textui.DrawText(dst, status, b.input.X, p.Y+116, 13, muted)
	for row := 0; row < stationSearchRows && s.firstRow+row < len(s.results); row++ {
		i := s.firstRow + row
		stop := s.results[i]
		box := b.row(row)
		fill := sim.RGBA(30, 41, 59, 255)
		if i == s.selected {
			fill = sim.RGBA(14, 75, 104, 255)
		}
		vector.DrawFilledRect(dst, float32(box.X), float32(box.Y), float32(box.W), float32(box.H), fill, false)
		label := fmt.Sprintf("#%d  %s", stop.ID+1, stop.LocalizedName())
		clip := dst.SubImage(image.Rect(int(box.X+10), int(box.Y), int(box.X+box.W-9), int(box.Y+box.H))).(*ebiten.Image)
		textui.DrawText(clip, label, box.X+10, box.Y+8, 14, color.White)
	}
}
