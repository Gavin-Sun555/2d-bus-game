package game

import (
	"image"
	"strings"
	"unicode/utf8"

	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/exp/textinput"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

// Editor buffer belongs to the UI, never the station until Save succeeds.
type stopNameEditor struct {
	stopID, focused int
	fields          [2]string
	caret           int // UTF-8 byte boundary in the focused field.
	selected        bool
	composition     string
	committed       bool
	err             string
	ime             textinput.Composer
}

func (e *stopNameEditor) initIME(g *Game) {
	e.ime.OnNewSession = func() *textinput.SessionOptions {
		box := g.stopNameEditorLayout().fields[e.focused]
		prefix, suffix := e.fields[e.focused][:e.caret], e.fields[e.focused][e.caret:]
		if e.selected {
			prefix, suffix = "", ""
		}
		width, _ := textui.MeasureText(prefix, 16)
		x := int(box.X + 12 + min(width, box.W-28))
		y := int(box.Y + 10)
		return &textinput.SessionOptions{CaretBounds: image.Rect(x, y, x+1, y+20), TextBeforeCaret: prefix, TextAfterCaret: suffix}
	}
	e.ime.OnComposition = func(c *textinput.Composition) { e.composition = c.Text() }
	e.ime.OnCommit = func(c *textinput.Commit) {
		e.committed = true
		text := c.Text()
		if strings.ContainsAny(text, "\n\r\t") {
			e.err = "站名不能换行 / Station names must be single-line"
			return
		}
		if a, b := c.IsSurroundingTextReplaced(); a || b {
			prefix, suffix := c.SurroundingText()
			if utf8.RuneCountInString(prefix+text+suffix) > 512 {
				e.err = "输入内容过长 / Input too long"
				return
			}
			e.fields[e.focused] = prefix + text + suffix
			e.caret = len(prefix) + len(text)
			e.selected = false
			e.err = ""
		} else {
			e.insert(text)
		}
	}
}
func (e *stopNameEditor) insert(text string) {
	value := e.fields[e.focused]
	if e.selected {
		value = ""
		e.caret = 0
		e.selected = false
	}
	candidate := value[:e.caret] + text + value[e.caret:]
	if utf8.RuneCountInString(candidate) > 512 {
		e.err = "输入内容过长 / Input too long"
		return
	}
	e.fields[e.focused] = candidate
	e.caret += len(text)
	e.err = ""
}
func (e *stopNameEditor) focus(index int) {
	e.ime.Confirm()
	e.focused = index
	e.caret = len(e.fields[index])
	e.selected = false
}
func repeatedKey(key ebiten.Key) bool {
	n := inpututil.KeyPressDuration(key)
	return n == 1 || (n >= 24 && (n-24)%3 == 0)
}
func (e *stopNameEditor) editKeys() {
	ctrl := ebiten.IsKeyPressed(ebiten.KeyControl) || ebiten.IsKeyPressed(ebiten.KeyMeta)
	if ctrl && inpututil.IsKeyJustPressed(ebiten.KeyV) && !e.committed {
		e.ime.Confirm()
		if text := stopNameClipboardText(); text != "" && !strings.ContainsAny(text, "\n\r\t") {
			e.insert(text)
		}
		return
	}
	if ctrl && inpututil.IsKeyJustPressed(ebiten.KeyA) {
		e.ime.Confirm()
		e.selected = true
		return
	}
	if !repeatedKey(ebiten.KeyBackspace) && !repeatedKey(ebiten.KeyDelete) && !repeatedKey(ebiten.KeyLeft) && !repeatedKey(ebiten.KeyRight) && !inpututil.IsKeyJustPressed(ebiten.KeyHome) && !inpututil.IsKeyJustPressed(ebiten.KeyEnd) {
		return
	}
	e.ime.Confirm()
	value := e.fields[e.focused]
	switch {
	case repeatedKey(ebiten.KeyBackspace):
		if e.selected {
			e.fields[e.focused] = ""
			e.caret = 0
		} else if e.caret > 0 {
			_, n := utf8.DecodeLastRuneInString(value[:e.caret])
			e.fields[e.focused] = value[:e.caret-n] + value[e.caret:]
			e.caret -= n
		}
	case repeatedKey(ebiten.KeyDelete):
		if e.selected {
			e.fields[e.focused] = ""
			e.caret = 0
		} else if e.caret < len(value) {
			_, n := utf8.DecodeRuneInString(value[e.caret:])
			e.fields[e.focused] = value[:e.caret] + value[e.caret+n:]
		}
	case repeatedKey(ebiten.KeyLeft):
		if e.selected {
			e.caret = 0
		} else if e.caret > 0 {
			_, n := utf8.DecodeLastRuneInString(value[:e.caret])
			e.caret -= n
		}
	case repeatedKey(ebiten.KeyRight):
		if e.selected {
			e.caret = len(value)
		} else if e.caret < len(value) {
			_, n := utf8.DecodeRuneInString(value[e.caret:])
			e.caret += n
		}
	case inpututil.IsKeyJustPressed(ebiten.KeyHome):
		e.caret = 0
	case inpututil.IsKeyJustPressed(ebiten.KeyEnd):
		e.caret = len(value)
	}
	e.selected = false
	e.err = ""
}
