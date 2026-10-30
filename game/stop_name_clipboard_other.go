//go:build !darwin || ios

package game

// Other platforms deliver paste through their native text input session.
func stopNameClipboardText() string { return "" }
