package game

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type saveSlotPicker struct {
	open, load, confirm bool
	selected            int
	slots               [MaxSaveSlots]SaveSlotInfo
	message             string
}

type savePickerLayout struct {
	panel          sim.Rect
	rows           [MaxSaveSlots]sim.Rect
	action, cancel sim.Rect
}

func savePickerBounds(width, height int) savePickerLayout {
	w := min(760., float64(width)-32)
	h := 430.
	x, y := (float64(width)-w)/2, (float64(height)-h)/2
	l := savePickerLayout{panel: sim.Rect{X: x, Y: y, W: w, H: h}}
	rowW := (w - 60) / 2
	for i := range l.rows {
		l.rows[i] = sim.Rect{X: x + 20 + float64(i/5)*(rowW+20), Y: y + 78 + float64(i%5)*51, W: rowW, H: 43}
	}
	l.action = sim.Rect{X: x + w - 260, Y: y + h - 54, W: 240, H: 36}
	l.cancel = sim.Rect{X: x + 20, Y: y + h - 54, W: 120, H: 36}
	return l
}

func (g *Game) openSavePicker(load bool) {
	if !load && g.State != StatePlaying {
		return
	}
	slots, err := g.SaveSlots()
	g.savePicker = saveSlotPicker{open: true, load: load, selected: g.activeSaveSlot, slots: slots}
	if err != nil {
		g.savePicker.message = err.Error()
	}
}

func (g *Game) selectSaveSlot(slot int) {
	g.savePicker.selected = (slot + MaxSaveSlots) % MaxSaveSlots
	g.savePicker.confirm = false
	g.savePicker.message = ""
}

func (g *Game) performSlotAction() {
	p := &g.savePicker
	slot := p.selected
	if p.load && !p.slots[slot].Exists {
		p.message = locale.T("这个槽位还没有存档。", "This slot is empty.")
		return
	}
	// Recheck the file at confirmation time, not just when the dialog opened.
	slots, err := g.SaveSlots()
	if err != nil {
		p.message = err.Error()
		return
	}
	if !p.load && slots[slot].Exists && !p.confirm {
		p.slots = slots
		p.confirm = true
		p.message = locale.Tf("再次确认将覆盖存档 %02d。", "Confirm again to overwrite slot %02d.", slot+1)
		return
	}
	load := p.load
	if load {
		err = g.LoadGameSlot(slot)
	} else {
		err = g.SaveGameSlot(slot)
	}
	if err != nil {
		g.savePicker.message = locale.T("操作失败：", "Operation failed: ") + err.Error()
		g.savePicker.confirm = false
		return
	}
	g.savePicker.open = false
	if load {
		g.saveMessage = locale.Tf("已读取存档 %02d。", "Loaded slot %02d.", slot+1)
	} else {
		g.saveMessage = locale.Tf("已保存至存档 %02d。", "Saved to slot %02d.", slot+1)
	}
	g.HUD.AddToast(g.saveMessage, true)
}

func (g *Game) updateSavePicker() {
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		g.savePicker.open = false
		return
	}
	keys := []ebiten.Key{ebiten.Key1, ebiten.Key2, ebiten.Key3, ebiten.Key4, ebiten.Key5, ebiten.Key6, ebiten.Key7, ebiten.Key8, ebiten.Key9, ebiten.Key0}
	for i, k := range keys {
		if inpututil.IsKeyJustPressed(k) {
			g.selectSaveSlot(i)
		}
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyUp) {
		g.selectSaveSlot(g.savePicker.selected - 1)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyDown) {
		g.selectSaveSlot(g.savePicker.selected + 1)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyLeft) || inpututil.IsKeyJustPressed(ebiten.KeyRight) {
		g.selectSaveSlot(g.savePicker.selected + 5)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEnter) {
		g.performSlotAction()
		return
	}
	if !inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		return
	}
	mx, my := ebiten.CursorPosition()
	point := sim.Vec2{X: float64(mx), Y: float64(my)}
	l := savePickerBounds(g.Width, g.Height)
	for i, r := range l.rows {
		if r.Contains(point) {
			g.selectSaveSlot(i)
			return
		}
	}
	if l.cancel.Contains(point) {
		g.savePicker.open = false
		return
	}
	if l.action.Contains(point) {
		g.performSlotAction()
	}
}

func (g *Game) drawSavePicker(screen *ebiten.Image) {
	p := &g.savePicker
	if !p.open {
		return
	}
	l := savePickerBounds(g.Width, g.Height)
	vector.DrawFilledRect(screen, 0, 0, float32(g.Width), float32(g.Height), sim.RGBA(0, 0, 0, 185), false)
	box := func(r sim.Rect, selected bool) {
		col := sim.RGBA(30, 41, 59, 255)
		if selected {
			col = sim.RGBA(12, 74, 110, 255)
		}
		vector.DrawFilledRect(screen, float32(r.X), float32(r.Y), float32(r.W), float32(r.H), col, false)
		if selected {
			vector.StrokeRect(screen, float32(r.X), float32(r.Y), float32(r.W), float32(r.H), 2, sim.RGBA(56, 189, 248, 255), false)
		}
	}
	box(l.panel, false)
	title := locale.T("保存游戏", "Save game")
	if p.load {
		title = locale.T("读取游戏", "Load game")
	}
	textui.DrawText(screen, title, l.panel.X+20, l.panel.Y+18, 22, sim.RGBA(241, 245, 249, 255))
	textui.DrawText(screen, locale.T("选择槽位 · 数字 1–9 / 0（10）· 回车确认 · ESC 返回", "Select slot · 1–9 / 0 (10) · Enter to confirm · Esc to close"), l.panel.X+20, l.panel.Y+51, 13, sim.RGBA(148, 163, 184, 255))
	for i, r := range l.rows {
		box(r, p.selected == i)
		detail := locale.T("空存档", "Empty")
		if p.slots[i].Exists {
			detail = p.slots[i].SavedAt.Format("2006-01-02 15:04:05")
		}
		textui.DrawText(screen, fmt.Sprintf("%02d   %s", i+1, detail), r.X+12, r.Y+13, 14, sim.RGBA(226, 232, 240, 255))
	}
	// Keep error messages within the modal on narrow displays.
	msg := []rune(p.message)
	for len(msg) > 0 {
		w, _ := textui.MeasureText(string(msg), 13)
		if w <= l.panel.W-52 {
			break
		}
		msg = msg[:len(msg)-1]
	}
	if len(msg) > 0 {
		textui.DrawText(screen, string(msg), l.panel.X+20, l.panel.Y+340, 13, sim.RGBA(251, 191, 36, 255))
	}
	box(l.cancel, false)
	textui.DrawText(screen, locale.T("返回", "Back"), l.cancel.X+16, l.cancel.Y+10, 15, sim.RGBA(226, 232, 240, 255))
	action := locale.T("保存到此槽位", "Save to slot")
	if p.load {
		action = locale.T("读取此存档", "Load slot")
	} else if p.confirm {
		action = locale.T("确认覆盖存档", "Confirm overwrite")
	}
	box(l.action, true)
	textui.DrawText(screen, action, l.action.X+14, l.action.Y+10, 15, sim.RGBA(226, 232, 240, 255))
}
