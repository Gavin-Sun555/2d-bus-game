//go:build darwin && !ios

package game

import (
	"github.com/ebitengine/purego/cstrings"
	"github.com/ebitengine/purego/objc"
	"github.com/hajimehoshi/ebiten/v2"
)

// Read only on an explicit paste shortcut while the editor is focused.
func stopNameClipboardText() string {
	var value string
	ebiten.RunOnMainThread(func() {
		pool := objc.ID(objc.GetClass("NSAutoreleasePool")).Send(objc.RegisterName("new"))
		defer pool.Send(objc.RegisterName("drain"))
		board := objc.ID(objc.GetClass("NSPasteboard")).Send(objc.RegisterName("generalPasteboard"))
		kind := objc.ID(objc.GetClass("NSString")).Send(objc.RegisterName("stringWithUTF8String:"), "public.utf8-plain-text")
		str := board.Send(objc.RegisterName("stringForType:"), kind)
		if str != 0 {
			value = cstrings.NSStringToString(str)
		}
	})
	return value
}
