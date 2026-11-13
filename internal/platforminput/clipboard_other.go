//go:build !darwin || ios

package platforminput

// Other platforms deliver paste through their native text input session.
func ClipboardText() string { return "" }
