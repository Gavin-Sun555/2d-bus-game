package game

import (
	"fmt"
	"image"
	"image/color"
	"math"
	"unicode/utf8"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type stopNameLayout struct {
	panel               sim.Rect
	fields              [2]sim.Rect
	save, cancel, reset sim.Rect
}

func (g *Game) stopNameEditorLayout() stopNameLayout {
	w := math.Min(660, float64(g.Width)-32)
	h := 380.
	x := (float64(g.Width) - w) / 2
	y := (float64(g.Height) - h) / 2
	return stopNameLayout{panel: sim.Rect{X: x, Y: y, W: w, H: h}, fields: [2]sim.Rect{{X: x + 24, Y: y + 100, W: w - 48, H: 44}, {X: x + 24, Y: y + 192, W: w - 48, H: 44}}, reset: sim.Rect{X: x + 24, Y: y + 320, W: 132, H: 36}, cancel: sim.Rect{X: x + w - 244, Y: y + 320, W: 100, H: 36}, save: sim.Rect{X: x + w - 132, Y: y + 320, W: 108, H: 36}}
}
func (g *Game) openStopNameEditor(id int) {
	if !g.IsPlanMode || id < 0 || id >= len(g.World.Stops) {
		return
	}
	g.closeStopNameEditor()
	s := g.World.Stops[id]
	e := &stopNameEditor{stopID: id, fields: [2]string{s.CustomNameZH, s.CustomNameEN}}
	e.caret = len(e.fields[0])
	e.initIME(g)
	g.stopNameEditor = e
	g.isDragging = false
}
func (g *Game) closeStopNameEditor() {
	if g.stopNameEditor != nil {
		g.stopNameEditor.ime.Cancel()
		g.stopNameEditor = nil
	}
}
func (g *Game) saveEditedStopName(reset bool) bool {
	e := g.stopNameEditor
	if e == nil {
		return false
	}
	e.ime.Confirm()
	n := stopLabels{Chinese: e.fields[0], English: e.fields[1]}
	if reset {
		n = stopLabels{}
	}
	if err := g.renameStop(e.stopID, n); err != nil {
		e.err = err.Error()
		return false
	}
	g.closeStopNameEditor()
	g.HUD.AddToast(locale.T("站名已保存到本地", "Station name saved locally"), true)
	return true
}
func (g *Game) updateStopNameEditor() {
	e := g.stopNameEditor
	if e == nil {
		return
	}
	// macOS attaches the input client to the key window. Do not start a
	// session before the window gains focus, and recreate it after refocusing.
	if !ebiten.IsFocused() {
		e.ime.Confirm()
		return
	}
	e.committed = false
	handled, err := e.ime.Update()
	// Unicode accessibility input can bypass the native IME client. Accept
	// unconsumed character events, without duplicating native commits/preedit.
	if !handled && !e.committed && e.composition == "" {
		if chars := ebiten.AppendInputChars(nil); len(chars) > 0 {
			e.insert(string(chars))
		}
	}
	if err != nil {
		e.err = err.Error()
	}
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		p := sim.Vec2{X: float64(x), Y: float64(y)}
		b := g.stopNameEditorLayout()
		switch {
		case b.save.Contains(p):
			g.saveEditedStopName(false)
			return
		case b.cancel.Contains(p):
			g.closeStopNameEditor()
			return
		case b.reset.Contains(p):
			g.saveEditedStopName(true)
			return
		}
		for i, box := range b.fields {
			if box.Contains(p) {
				e.focus(i)
				return
			}
		}
	}
	// Mouse actions can confirm/cancel an active composition, but an IME's
	// Enter/Escape/arrow key must not also trigger the dialog's shortcuts.
	if handled {
		return
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		g.closeStopNameEditor()
		return
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyTab) {
		e.focus(1 - e.focused)
		return
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEnter) || inpututil.IsKeyJustPressed(ebiten.KeyNumpadEnter) {
		g.saveEditedStopName(false)
		return
	}
	e.editKeys()
}
func drawStopEditorButton(dst *ebiten.Image, b sim.Rect, label string, fill color.Color) {
	vector.DrawFilledRect(dst, float32(b.X), float32(b.Y), float32(b.W), float32(b.H), fill, true)
	width, _ := textui.MeasureText(label, 14)
	textui.DrawText(dst, label, b.X+(b.W-width)/2, b.Y+10, 14, color.White)
}
func (g *Game) drawStopNameEditor(dst *ebiten.Image) {
	e := g.stopNameEditor
	if e == nil {
		return
	}
	b := g.stopNameEditorLayout()
	p := b.panel
	vector.DrawFilledRect(dst, 0, 0, float32(g.Width), float32(g.Height), sim.RGBA(0, 0, 0, 175), false)
	vector.DrawFilledRect(dst, float32(p.X), float32(p.Y), float32(p.W), float32(p.H), sim.RGBA(15, 23, 42, 255), true)
	vector.StrokeRect(dst, float32(p.X), float32(p.Y), float32(p.W), float32(p.H), 2, sim.RGBA(56, 189, 248, 255), true)
	textui.DrawText(dst, locale.Tf("编辑站名 · #%d", "Edit station name · #%d", e.stopID+1), p.X+24, p.Y+20, 22, color.White)
	defaultName := locale.T("默认：", "Default: ") + g.World.Stops[e.stopID].Name
	for width, _ := textui.MeasureText(defaultName, 12); width > p.W-48; width, _ = textui.MeasureText(defaultName, 12) {
		r := []rune(defaultName)
		defaultName = string(r[:len(r)-2]) + "…"
	}
	textui.DrawText(dst, defaultName, p.X+24, p.Y+53, 12, sim.RGBA(148, 163, 184, 255))
	labels := [2]string{locale.T("中文站名 · 最多12字", "Chinese name · max 12 characters"), locale.T("英文站名 · 最多30字符（含空格）", "English name · max 30 characters incl. spaces")}
	limits := [2]int{12, 30}
	for i, box := range b.fields {
		textui.DrawText(dst, labels[i], box.X, box.Y-23, 13, sim.RGBA(203, 213, 225, 255))
		edge := sim.RGBA(71, 85, 105, 255)
		if i == e.focused {
			edge = sim.RGBA(56, 189, 248, 255)
		}
		vector.DrawFilledRect(dst, float32(box.X), float32(box.Y), float32(box.W), float32(box.H), sim.RGBA(30, 41, 59, 255), true)
		vector.StrokeRect(dst, float32(box.X), float32(box.Y), float32(box.W), float32(box.H), 1.5, edge, true)
		value := e.fields[i]
		caret := len(value)
		preedit := ""
		if i == e.focused {
			caret = e.caret
			preedit = e.composition
		}
		visible := value[:caret] + preedit + value[caret:]
		advance, _ := textui.MeasureText(value[:caret]+preedit, 16)
		scroll := math.Max(0, advance-(box.W-30))
		clip := dst.SubImage(image.Rect(int(box.X+8), int(box.Y+5), int(box.X+box.W-8), int(box.Y+box.H-5))).(*ebiten.Image)
		if i == e.focused && e.selected {
			width, _ := textui.MeasureText(value, 16)
			vector.DrawFilledRect(clip, float32(box.X+12-scroll), float32(box.Y+11), float32(width), 21, sim.RGBA(37, 99, 235, 180), false)
		}
		textui.DrawText(clip, visible, box.X+12-scroll, box.Y+12, 16, color.White)
		if i == e.focused {
			cx := box.X + 12 + advance - scroll
			vector.StrokeLine(clip, float32(cx), float32(box.Y+10), float32(cx), float32(box.Y+32), 1, sim.RGBA(125, 211, 252, 255), false)
			if preedit != "" {
				preWidth, _ := textui.MeasureText(preedit, 16)
				vector.StrokeLine(clip, float32(cx-preWidth), float32(box.Y+33), float32(cx), float32(box.Y+33), 1, sim.RGBA(125, 211, 252, 255), false)
			}
		}
		count := utf8.RuneCountInString(value)
		countColor := sim.RGBA(148, 163, 184, 255)
		if count > limits[i] {
			countColor = sim.RGBA(248, 113, 113, 255)
		}
		counter := fmt.Sprintf("%d / %d", count, limits[i])
		cw, _ := textui.MeasureText(counter, 12)
		textui.DrawText(dst, counter, box.X+box.W-cw, box.Y+50, 12, countColor)
	}
	hint := locale.T("留空使用默认名 · Tab切换 · Enter保存 · Esc取消", "Blank: default name · Tab: next field · Enter: save · Esc: cancel")
	col := sim.RGBA(148, 163, 184, 255)
	if e.err != "" {
		hint = e.err
		col = sim.RGBA(248, 113, 113, 255)
	} else if err := validateStopLabels(stopLabels{Chinese: e.fields[0], English: e.fields[1]}); err != nil {
		hint = err.Error()
		col = sim.RGBA(248, 113, 113, 255)
	}
	for width, _ := textui.MeasureText(hint, 12); width > p.W-48; width, _ = textui.MeasureText(hint, 12) {
		r := []rune(hint)
		hint = string(r[:len(r)-2]) + "…"
	}
	textui.DrawText(dst, hint, p.X+24, p.Y+284, 12, col)
	drawStopEditorButton(dst, b.reset, locale.T("恢复默认", "Reset names"), sim.RGBA(71, 85, 105, 255))
	drawStopEditorButton(dst, b.cancel, locale.T("取消", "Cancel"), sim.RGBA(51, 65, 85, 255))
	drawStopEditorButton(dst, b.save, locale.T("保存", "Save"), sim.RGBA(2, 132, 199, 255))
}
