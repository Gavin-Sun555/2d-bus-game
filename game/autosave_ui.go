package game

import (
	"fmt"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

func drawSaveSlotEntry(screen *ebiten.Image, r sim.Rect, index int, name string, info SaveSlotInfo) {
	if name == "" {
		name = locale.T("未命名", "Unnamed")
	}
	metadata := fitSavePickerText(info.Metadata.label(), 11, r.W*.55)
	metaWidth, _ := textui.MeasureText(metadata, 11)
	top := fitSavePickerText(fmt.Sprintf("%02d  %s", index+1, name), 13, r.W-metaWidth-36)
	textui.DrawText(screen, top, r.X+12, r.Y+6, 13, careerFG)
	textui.DrawText(screen, metadata, r.X+r.W-metaWidth-12, r.Y+8, 11, careerAccent)
	stamp := info.SavedAt.Local().Format("2006-01-02 15:04:05")
	if info.Metadata != nil {
		if info.Metadata.Mode == ModeCareer {
			if info.Metadata.DriverName != "" {
				stamp += " · " + info.Metadata.DriverName
			} else {
				stamp += " · " + locale.T("生涯全局进度", "Career progress")
			}
		} else {
			if info.Metadata.CenterBRT {
				stamp += " · BRT"
			}
			if info.Metadata.LargeGrid {
				stamp += " · 4x"
			}
		}
	}
	textui.DrawText(screen, fitSavePickerText(stamp, 11, r.W-24), r.X+12, r.Y+24, 11, careerMuted)
}
func (g *Game) setSavePickerSource(automatic bool) {
	if !g.savePicker.load {
		return
	}
	g.savePicker.cancelSaveSlotRename()
	g.savePicker.automatic = automatic
	g.savePicker.selected = 0
	g.savePicker.confirm = false
	g.savePicker.confirmDelete = false
	g.savePicker.message = ""
	if automatic {
		saves, err := g.AutoSaves()
		g.savePicker.slots = [MaxSaveSlots]SaveSlotInfo{}
		if err != nil {
			g.savePicker.message = err.Error()
			return
		}
		for i, s := range saves {
			g.savePicker.slots[i] = s
		}
	} else {
		slots, err := g.SaveSlots()
		g.savePicker.slots = slots
		if err != nil {
			g.savePicker.message = err.Error()
		}
	}
}
func autoSaveRows(l savePickerLayout) [MaxAutoSaves]sim.Rect {
	var rows [MaxAutoSaves]sim.Rect
	for i := range rows {
		rows[i] = sim.Rect{X: l.panel.X + 20, Y: l.panel.Y + 78 + float64(i)*50, W: l.panel.W - 40, H: 43}
	}
	return rows
}
func (g *Game) loadSelectedAutoSave() {
	p := &g.savePicker
	info := p.slots[p.selected]
	if !info.Exists {
		p.message = locale.T("这个位置还没有自动存档", "No autosave here yet")
		return
	}
	// Use the selected file's physical slot, not its changing chronological rank.
	if err := g.LoadAutoSave(info.Slot); err != nil {
		g.savePicker.message = err.Error()
		return
	}
	if g.isCareer() {
		g.State = StateMainMenu
	}
	g.savePicker.open = false
	g.saveMessage = locale.T("已读取自动存档", "Autosave loaded")
	g.HUD.AddToast(g.saveMessage, true)
}
func (g *Game) updateAutoSavePicker() {
	p := &g.savePicker
	l := savePickerBounds(g.Width, g.Height)
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		p.open = false
		return
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyUp) {
		p.selected = (p.selected + MaxAutoSaves - 1) % MaxAutoSaves
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyDown) {
		p.selected = (p.selected + 1) % MaxAutoSaves
	}
	for i, k := range []ebiten.Key{ebiten.Key1, ebiten.Key2, ebiten.Key3, ebiten.Key4, ebiten.Key5} {
		if inpututil.IsKeyJustPressed(k) {
			p.selected = i
		}
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEnter) || inpututil.IsKeyJustPressed(ebiten.KeyNumpadEnter) {
		g.loadSelectedAutoSave()
		return
	}
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		point := sim.Vec2{X: float64(x), Y: float64(y)}
		for i, r := range autoSaveRows(l) {
			if r.Contains(point) {
				p.selected = i
				return
			}
		}
		if l.cancel.Contains(point) {
			p.open = false
			return
		}
		if l.action.Contains(point) {
			g.loadSelectedAutoSave()
		}
	}
}
func (g *Game) drawAutoSavePicker(screen *ebiten.Image, l savePickerLayout) {
	p := &g.savePicker
	for i, r := range autoSaveRows(l) {
		bg := sim.RGBA(30, 41, 59, 255)
		if i == p.selected {
			bg = sim.RGBA(12, 74, 110, 255)
		}
		panelBox(screen, r, bg)
		info := p.slots[i]
		if !info.Exists {
			careerText(screen, locale.Tf("%02d  暂无自动存档", "%02d  No autosave yet", i+1), r.X+12, r.Y+13, 14, careerMuted)
			continue
		}
		drawSaveSlotEntry(screen, r, i, info.Name, info)
	}
	careerText(screen, locale.T("每 5 分钟及退出时保存；各模式分别保留最新 5 份。", "Saved every 5 minutes and on exit; latest 5 per mode."), l.panel.X+20, l.panel.Y+342, 13, careerAccent)
	careerText(screen, locale.T("按时间从新到旧排列 · 与手动槽位独立 · [Tab] 切换", "Newest first · Separate from manual slots · [Tab] Switch"), l.panel.X+20, l.panel.Y+366, 12, careerMuted)
	if p.message != "" {
		careerText(screen, fitSavePickerText(p.message, 13, l.panel.W-40), l.panel.X+20, l.panel.Y+391, 13, sim.RGBA(251, 191, 36, 255))
	}
	panelBox(screen, l.cancel, sim.RGBA(30, 41, 59, 255))
	careerText(screen, locale.T("返回", "Back"), l.cancel.X+16, l.cancel.Y+10, 15, careerFG)
	panelBox(screen, l.action, sim.RGBA(12, 74, 110, 255))
	careerText(screen, locale.T("读取自动存档", "Load autosave"), l.action.X+14, l.action.Y+10, 15, careerFG)
}
