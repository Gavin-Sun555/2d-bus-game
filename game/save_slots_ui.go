package game

import (
	"fmt"
	"image"
	"math"
	"strings"
	"unicode/utf8"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/exp/textinput"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type saveSlotPicker struct {
	open, load, confirm, confirmDelete bool
	selected                           int
	slots                              [MaxSaveSlots]SaveSlotInfo
	draftNames                         [MaxSaveSlots]string
	hasDraftName                       [MaxSaveSlots]bool
	editingName                        bool
	nameInput                          string
	nameCaret                          int
	nameSelected                       bool
	nameComposition                    string
	nameCommitted                      bool
	nameIME                            textinput.Composer
	message                            string
}

type savePickerLayout struct {
	panel                  sim.Rect
	rows                   [MaxSaveSlots]sim.Rect
	nameField, rename      sim.Rect
	action, delete, cancel sim.Rect
}

func savePickerBounds(width, height int) savePickerLayout {
	w := min(760., float64(width)-32)
	h := 486.
	x, y := (float64(width)-w)/2, (float64(height)-h)/2
	l := savePickerLayout{panel: sim.Rect{X: x, Y: y, W: w, H: h}}
	rowW := (w - 60) / 2
	for i := range l.rows {
		l.rows[i] = sim.Rect{X: x + 20 + float64(i/5)*(rowW+20), Y: y + 78 + float64(i%5)*50, W: rowW, H: 43}
	}
	l.nameField = sim.Rect{X: x + 20, Y: y + 334, W: w - 182, H: 36}
	l.rename = sim.Rect{X: x + w - 150, Y: y + 334, W: 130, H: 36}
	l.cancel = sim.Rect{X: x + 20, Y: y + h - 52, W: 120, H: 36}
	l.delete = sim.Rect{X: x + 152, Y: y + h - 52, W: 156, H: 36}
	l.action = sim.Rect{X: x + w - 260, Y: y + h - 52, W: 240, H: 36}
	return l
}

func (p *saveSlotPicker) slotDisplayName(slot int) string {
	if slot < 0 || slot >= MaxSaveSlots {
		return ""
	}
	if p.hasDraftName[slot] {
		return p.draftNames[slot]
	}
	return p.slots[slot].Name
}

func (p *saveSlotPicker) cancelSaveSlotRename() {
	if p.editingName {
		p.nameIME.Cancel()
		p.editingName = false
		p.nameComposition = ""
		p.nameSelected = false
	}
}

func (g *Game) initSavePickerIME() {
	p := &g.savePicker
	p.nameIME.OnNewSession = func() *textinput.SessionOptions {
		box := savePickerBounds(g.Width, g.Height).nameField
		prefix, suffix := p.nameInput[:p.nameCaret], p.nameInput[p.nameCaret:]
		if p.nameSelected {
			prefix, suffix = "", ""
		}
		width, _ := textui.MeasureText(prefix, 14)
		x := int(box.X + 12 + min(width, box.W-28))
		y := int(box.Y + 8)
		return &textinput.SessionOptions{CaretBounds: image.Rect(x, y, x+1, y+20), TextBeforeCaret: prefix, TextAfterCaret: suffix}
	}
	p.nameIME.OnComposition = func(c *textinput.Composition) {
		p.nameComposition = c.Text()
	}
	p.nameIME.OnCommit = func(c *textinput.Commit) {
		p.nameCommitted = true
		text := c.Text()
		if strings.ContainsAny(text, "\n\r\t") {
			p.message = locale.T("存档名称不能换行", "Save names must be single-line")
			return
		}
		if a, b := c.IsSurroundingTextReplaced(); a || b {
			prefix, suffix := c.SurroundingText()
			if utf8.RuneCountInString(prefix+text+suffix) > 128 {
				p.message = locale.T("输入内容过长", "Input too long")
				return
			}
			p.nameInput = prefix + text + suffix
			p.nameCaret = len(prefix) + len(text)
			p.nameSelected = false
			p.message = ""
		} else {
			p.insertNameText(text)
		}
	}
}

func (p *saveSlotPicker) insertNameText(text string) {
	value := p.nameInput
	if p.nameSelected {
		value = ""
		p.nameCaret = 0
		p.nameSelected = false
	}
	candidate := value[:p.nameCaret] + text + value[p.nameCaret:]
	if utf8.RuneCountInString(candidate) > 128 {
		p.message = locale.T("输入内容过长", "Input too long")
		return
	}
	p.nameInput = candidate
	p.nameCaret += len(text)
	p.message = ""
}

func (p *saveSlotPicker) editNameKeys() {
	ctrl := ebiten.IsKeyPressed(ebiten.KeyControl) || ebiten.IsKeyPressed(ebiten.KeyMeta)
	if ctrl && inpututil.IsKeyJustPressed(ebiten.KeyV) && !p.nameCommitted {
		p.nameIME.Confirm()
		if text := stopNameClipboardText(); text != "" && !strings.ContainsAny(text, "\n\r\t") {
			p.insertNameText(text)
		}
		return
	}
	if ctrl && inpututil.IsKeyJustPressed(ebiten.KeyA) {
		p.nameIME.Confirm()
		p.nameSelected = true
		return
	}
	if !repeatedKey(ebiten.KeyBackspace) && !repeatedKey(ebiten.KeyDelete) && !repeatedKey(ebiten.KeyLeft) && !repeatedKey(ebiten.KeyRight) && !inpututil.IsKeyJustPressed(ebiten.KeyHome) && !inpututil.IsKeyJustPressed(ebiten.KeyEnd) {
		return
	}
	p.nameIME.Confirm()
	value := p.nameInput
	switch {
	case repeatedKey(ebiten.KeyBackspace):
		if p.nameSelected {
			p.nameInput = ""
			p.nameCaret = 0
		} else if p.nameCaret > 0 {
			_, n := utf8.DecodeLastRuneInString(value[:p.nameCaret])
			p.nameInput = value[:p.nameCaret-n] + value[p.nameCaret:]
			p.nameCaret -= n
		}
	case repeatedKey(ebiten.KeyDelete):
		if p.nameSelected {
			p.nameInput = ""
			p.nameCaret = 0
		} else if p.nameCaret < len(value) {
			_, n := utf8.DecodeRuneInString(value[p.nameCaret:])
			p.nameInput = value[:p.nameCaret] + value[p.nameCaret+n:]
		}
	case repeatedKey(ebiten.KeyLeft):
		if p.nameSelected {
			p.nameCaret = 0
		} else if p.nameCaret > 0 {
			_, n := utf8.DecodeLastRuneInString(value[:p.nameCaret])
			p.nameCaret -= n
		}
	case repeatedKey(ebiten.KeyRight):
		if p.nameSelected {
			p.nameCaret = len(value)
		} else if p.nameCaret < len(value) {
			_, n := utf8.DecodeRuneInString(value[p.nameCaret:])
			p.nameCaret += n
		}
	case inpututil.IsKeyJustPressed(ebiten.KeyHome):
		p.nameCaret = 0
	case inpututil.IsKeyJustPressed(ebiten.KeyEnd):
		p.nameCaret = len(value)
	}
	p.nameSelected = false
	p.message = ""
}

func (g *Game) beginSaveSlotRename() {
	p := &g.savePicker
	slot := p.selected
	if p.load && !p.slots[slot].Exists {
		p.message = locale.T("这个槽位还没有存档。", "This slot is empty.")
		return
	}
	p.confirm = false
	p.confirmDelete = false
	p.message = ""
	p.editingName = true
	p.nameInput = p.slotDisplayName(slot)
	p.nameCaret = len(p.nameInput)
	p.nameSelected = false
	p.nameComposition = ""
	g.initSavePickerIME()
}

func (g *Game) commitSaveSlotRename() bool {
	p := &g.savePicker
	if !p.editingName {
		return true
	}
	p.nameIME.Confirm()
	clean, err := validateSaveSlotName(p.nameInput)
	if err != nil {
		p.message = err.Error()
		return false
	}
	slot := p.selected
	if slots, err := g.SaveSlots(); err == nil {
		p.slots = slots
	}
	if p.slots[slot].Exists {
		if err := g.RenameSaveSlot(slot, clean); err != nil {
			p.message = locale.T("操作失败：", "Operation failed: ") + err.Error()
			return false
		}
		p.slots[slot].Name = clean
		p.draftNames[slot] = clean
		p.hasDraftName[slot] = true
		p.editingName = false
		p.nameComposition = ""
		p.nameSelected = false
		if clean == "" {
			p.message = locale.Tf("已清除存档 %02d 名称。", "Cleared slot %02d name.", slot+1)
		} else {
			p.message = locale.Tf("已将存档 %02d 命名为「%s」。", "Renamed slot %02d to \"%s\".", slot+1, clean)
		}
		if g.HUD != nil {
			g.HUD.AddToast(p.message, true)
		}
		return true
	}
	if p.load {
		p.editingName = false
		p.message = locale.T("这个槽位还没有存档。", "This slot is empty.")
		return false
	}
	p.draftNames[slot] = clean
	p.hasDraftName[slot] = true
	p.editingName = false
	p.nameComposition = ""
	p.nameSelected = false
	if clean == "" {
		p.message = locale.Tf("已清除存档 %02d 预设名称。", "Cleared preset name for slot %02d.", slot+1)
	} else {
		p.message = locale.Tf("已设置存档 %02d 名称为「%s」，点击保存即可写入。", "Set slot %02d name to \"%s\"; save to write.", slot+1, clean)
	}
	return true
}

func (g *Game) performDeleteSlot() {
	p := &g.savePicker
	p.cancelSaveSlotRename()
	slot := p.selected
	slots, err := g.SaveSlots()
	if err != nil {
		p.message = err.Error()
		return
	}
	p.slots = slots
	if !p.slots[slot].Exists {
		p.confirmDelete = false
		p.message = locale.T("这个槽位还没有存档。", "This slot is empty.")
		return
	}
	if !p.confirmDelete {
		p.confirm = false
		p.confirmDelete = true
		p.message = locale.Tf("再次确认将删除存档 %02d。", "Confirm again to delete slot %02d.", slot+1)
		return
	}
	if err := g.DeleteSaveSlot(slot); err != nil {
		p.confirmDelete = false
		p.message = locale.T("操作失败：", "Operation failed: ") + err.Error()
		return
	}
	if refreshed, err := g.SaveSlots(); err == nil {
		p.slots = refreshed
	} else {
		p.slots[slot] = SaveSlotInfo{Slot: slot}
	}
	p.draftNames[slot] = ""
	p.hasDraftName[slot] = false
	p.confirmDelete = false
	p.confirm = false
	g.saveMessage = locale.Tf("已删除存档 %02d。", "Deleted slot %02d.", slot+1)
	p.message = g.saveMessage
	if g.HUD != nil {
		g.HUD.AddToast(g.saveMessage, true)
	}
}

func (g *Game) openSavePicker(load bool) {
	if !load && g.State != StatePlaying {
		return
	}
	g.savePicker.cancelSaveSlotRename()
	slots, err := g.SaveSlots()
	g.savePicker = saveSlotPicker{open: true, load: load, selected: g.activeSaveSlot, slots: slots}
	if err != nil {
		g.savePicker.message = err.Error()
	}
}

func (g *Game) selectSaveSlot(slot int) {
	g.savePicker.cancelSaveSlotRename()
	g.savePicker.selected = (slot + MaxSaveSlots) % MaxSaveSlots
	g.savePicker.confirm = false
	g.savePicker.confirmDelete = false
	g.savePicker.message = ""
}

func (g *Game) performSlotAction() {
	p := &g.savePicker
	slot := p.selected
	if p.editingName {
		p.nameIME.Confirm()
		clean, err := validateSaveSlotName(p.nameInput)
		if err != nil {
			p.message = err.Error()
			return
		}
		p.draftNames[slot] = clean
		p.hasDraftName[slot] = true
		p.editingName = false
		p.nameComposition = ""
		p.nameSelected = false
	}
	p.confirmDelete = false
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
	} else if p.hasDraftName[slot] {
		err = g.SaveGameSlotWithName(slot, p.draftNames[slot])
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
	p := &g.savePicker
	l := savePickerBounds(g.Width, g.Height)
	if p.editingName {
		if !ebiten.IsFocused() {
			p.nameIME.Confirm()
			return
		}
		p.nameCommitted = false
		handled, err := p.nameIME.Update()
		if !handled && !p.nameCommitted && p.nameComposition == "" {
			if chars := ebiten.AppendInputChars(nil); len(chars) > 0 {
				p.insertNameText(string(chars))
			}
		}
		if err != nil {
			p.message = err.Error()
		}
		if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
			mx, my := ebiten.CursorPosition()
			point := sim.Vec2{X: float64(mx), Y: float64(my)}
			switch {
			case l.rename.Contains(point):
				g.commitSaveSlotRename()
				return
			case l.nameField.Contains(point):
				return
			case l.cancel.Contains(point):
				p.cancelSaveSlotRename()
				p.message = ""
				return
			case l.delete.Contains(point):
				g.performDeleteSlot()
				return
			case l.action.Contains(point):
				g.performSlotAction()
				return
			}
			for i, r := range l.rows {
				if r.Contains(point) {
					g.selectSaveSlot(i)
					return
				}
			}
			p.cancelSaveSlotRename()
			return
		}
		if handled {
			return
		}
		if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
			p.cancelSaveSlotRename()
			p.message = ""
			return
		}
		if inpututil.IsKeyJustPressed(ebiten.KeyEnter) || inpututil.IsKeyJustPressed(ebiten.KeyNumpadEnter) {
			g.commitSaveSlotRename()
			return
		}
		p.editNameKeys()
		return
	}

	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		if p.confirm || p.confirmDelete {
			p.confirm = false
			p.confirmDelete = false
			p.message = ""
			return
		}
		p.open = false
		return
	}
	keys := []ebiten.Key{ebiten.Key1, ebiten.Key2, ebiten.Key3, ebiten.Key4, ebiten.Key5, ebiten.Key6, ebiten.Key7, ebiten.Key8, ebiten.Key9, ebiten.Key0}
	for i, k := range keys {
		if inpututil.IsKeyJustPressed(k) {
			g.selectSaveSlot(i)
		}
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyUp) {
		g.selectSaveSlot(p.selected - 1)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyDown) {
		g.selectSaveSlot(p.selected + 1)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyLeft) || inpututil.IsKeyJustPressed(ebiten.KeyRight) {
		g.selectSaveSlot(p.selected + 5)
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyF2) || inpututil.IsKeyJustPressed(ebiten.KeyR) {
		g.beginSaveSlotRename()
		return
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyDelete) || inpututil.IsKeyJustPressed(ebiten.KeyBackspace) {
		g.performDeleteSlot()
		return
	}
	if inpututil.IsKeyJustPressed(ebiten.KeyEnter) || inpututil.IsKeyJustPressed(ebiten.KeyNumpadEnter) {
		g.performSlotAction()
		return
	}
	if !inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		return
	}
	mx, my := ebiten.CursorPosition()
	point := sim.Vec2{X: float64(mx), Y: float64(my)}
	for i, r := range l.rows {
		if r.Contains(point) {
			g.selectSaveSlot(i)
			return
		}
	}
	if l.nameField.Contains(point) || l.rename.Contains(point) {
		g.beginSaveSlotRename()
		return
	}
	if l.delete.Contains(point) {
		g.performDeleteSlot()
		return
	}
	if l.cancel.Contains(point) {
		p.open = false
		return
	}
	if l.action.Contains(point) {
		g.performSlotAction()
	}
}

func fitSavePickerText(text string, size, maxW float64) string {
	if w, _ := textui.MeasureText(text, size); w <= maxW {
		return text
	}
	runes := []rune(text)
	for len(runes) > 1 {
		runes = runes[:len(runes)-1]
		candidate := string(runes) + "…"
		if w, _ := textui.MeasureText(candidate, size); w <= maxW {
			return candidate
		}
	}
	return ""
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
	textui.DrawText(screen, locale.T("选择槽位 · 数字 1–9/0 · R更名 · Del删除 · 回车确认 · ESC返回", "Select slot · 1–9/0 · R rename · Del delete · Enter confirm · Esc back"), l.panel.X+20, l.panel.Y+51, 13, sim.RGBA(148, 163, 184, 255))
	for i, r := range l.rows {
		box(r, p.selected == i)
		name := p.slotDisplayName(i)
		if !p.slots[i].Exists {
			detail := locale.T("空存档", "Empty")
			if name != "" {
				detail = fmt.Sprintf("%s (%s)", name, detail)
			}
			line := fitSavePickerText(fmt.Sprintf("%02d   %s", i+1, detail), 14, r.W-24)
			textui.DrawText(screen, line, r.X+12, r.Y+13, 14, sim.RGBA(226, 232, 240, 255))
			continue
		}
		stamp := p.slots[i].SavedAt.Format("2006-01-02 15:04:05")
		if name == "" {
			textui.DrawText(screen, fmt.Sprintf("%02d   %s", i+1, stamp), r.X+12, r.Y+13, 14, sim.RGBA(226, 232, 240, 255))
			continue
		}
		top := fitSavePickerText(fmt.Sprintf("%02d   %s", i+1, name), 13, r.W-24)
		textui.DrawText(screen, top, r.X+12, r.Y+6, 13, sim.RGBA(241, 245, 249, 255))
		textui.DrawText(screen, "     "+stamp, r.X+12, r.Y+24, 11, sim.RGBA(148, 163, 184, 255))
	}

	// Name editor / rename bar for the currently selected slot
	fieldEdge := sim.RGBA(71, 85, 105, 255)
	if p.editingName {
		fieldEdge = sim.RGBA(56, 189, 248, 255)
	}
	vector.DrawFilledRect(screen, float32(l.nameField.X), float32(l.nameField.Y), float32(l.nameField.W), float32(l.nameField.H), sim.RGBA(15, 23, 42, 255), false)
	vector.StrokeRect(screen, float32(l.nameField.X), float32(l.nameField.Y), float32(l.nameField.W), float32(l.nameField.H), 1.5, fieldEdge, false)
	if p.editingName {
		value := p.nameInput
		caret := min(p.nameCaret, len(value))
		preedit := p.nameComposition
		visible := value[:caret] + preedit + value[caret:]
		advance, _ := textui.MeasureText(value[:caret]+preedit, 14)
		scroll := math.Max(0, advance-(l.nameField.W-84))
		clip := screen.SubImage(image.Rect(int(l.nameField.X+8), int(l.nameField.Y+4), int(l.nameField.X+l.nameField.W-64), int(l.nameField.Y+l.nameField.H-4))).(*ebiten.Image)
		if p.nameSelected {
			width, _ := textui.MeasureText(value, 14)
			vector.DrawFilledRect(clip, float32(l.nameField.X+12-scroll), float32(l.nameField.Y+8), float32(width), 20, sim.RGBA(37, 99, 235, 180), false)
		}
		textui.DrawText(clip, visible, l.nameField.X+12-scroll, l.nameField.Y+10, 14, sim.RGBA(241, 245, 249, 255))
		cx := l.nameField.X + 12 + advance - scroll
		vector.StrokeLine(clip, float32(cx), float32(l.nameField.Y+8), float32(cx), float32(l.nameField.Y+28), 1, sim.RGBA(125, 211, 252, 255), false)
		if preedit != "" {
			preWidth, _ := textui.MeasureText(preedit, 14)
			vector.StrokeLine(clip, float32(cx-preWidth), float32(l.nameField.Y+29), float32(cx), float32(l.nameField.Y+29), 1, sim.RGBA(125, 211, 252, 255), false)
		}
		count := utf8.RuneCountInString(value)
		countCol := sim.RGBA(148, 163, 184, 255)
		if count > MaxSaveNameRunes {
			countCol = sim.RGBA(248, 113, 113, 255)
		}
		counter := fmt.Sprintf("%d/%d", count, MaxSaveNameRunes)
		cw, _ := textui.MeasureText(counter, 11)
		textui.DrawText(screen, counter, l.nameField.X+l.nameField.W-cw-10, l.nameField.Y+11, 11, countCol)
	} else {
		curName := p.slotDisplayName(p.selected)
		if curName == "" {
			placeholder := locale.Tf("存档 %02d 名称：未命名（点击此处或按 R 命名/更名，最多 %d 字）", "Slot %02d name: Unnamed (click or press R to name/rename, max %d chars)", p.selected+1, MaxSaveNameRunes)
			textui.DrawText(screen, fitSavePickerText(placeholder, 13, l.nameField.W-24), l.nameField.X+12, l.nameField.Y+10, 13, sim.RGBA(148, 163, 184, 255))
		} else {
			label := locale.Tf("存档 %02d 名称：%s", "Slot %02d name: %s", p.selected+1, curName)
			textui.DrawText(screen, fitSavePickerText(label, 14, l.nameField.W-24), l.nameField.X+12, l.nameField.Y+10, 14, sim.RGBA(226, 232, 240, 255))
		}
	}

	box(l.rename, p.editingName)
	renameLabel := locale.T("命名 / 更名", "Name / Rename")
	if p.editingName {
		renameLabel = locale.T("保存名称", "Save Name")
	}
	textui.DrawText(screen, renameLabel, l.rename.X+16, l.rename.Y+10, 14, sim.RGBA(226, 232, 240, 255))

	// Keep error messages within the modal on narrow displays.
	if p.message != "" {
		msg := fitSavePickerText(p.message, 13, l.panel.W-40)
		textui.DrawText(screen, msg, l.panel.X+20, l.panel.Y+384, 13, sim.RGBA(251, 191, 36, 255))
	}

	box(l.cancel, false)
	textui.DrawText(screen, locale.T("返回", "Back"), l.cancel.X+16, l.cancel.Y+10, 15, sim.RGBA(226, 232, 240, 255))

	delCol := sim.RGBA(69, 26, 38, 255)
	if !p.slots[p.selected].Exists {
		delCol = sim.RGBA(30, 41, 59, 180)
	} else if p.confirmDelete {
		delCol = sim.RGBA(153, 27, 27, 255)
	}
	vector.DrawFilledRect(screen, float32(l.delete.X), float32(l.delete.Y), float32(l.delete.W), float32(l.delete.H), delCol, false)
	if p.confirmDelete {
		vector.StrokeRect(screen, float32(l.delete.X), float32(l.delete.Y), float32(l.delete.W), float32(l.delete.H), 2, sim.RGBA(248, 113, 113, 255), false)
	}
	delLabel := locale.T("删除存档", "Delete slot")
	if p.confirmDelete {
		delLabel = locale.T("确认删除", "Confirm delete")
	}
	delTextCol := sim.RGBA(254, 202, 202, 255)
	if !p.slots[p.selected].Exists {
		delTextCol = sim.RGBA(100, 116, 139, 255)
	}
	textui.DrawText(screen, delLabel, l.delete.X+16, l.delete.Y+10, 15, delTextCol)

	action := locale.T("保存到此槽位", "Save to slot")
	if p.load {
		action = locale.T("读取此存档", "Load slot")
	} else if p.confirm {
		action = locale.T("确认覆盖存档", "Confirm overwrite")
	}
	box(l.action, true)
	textui.DrawText(screen, action, l.action.X+14, l.action.Y+10, 15, sim.RGBA(226, 232, 240, 255))
}
