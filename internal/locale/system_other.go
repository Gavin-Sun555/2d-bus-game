//go:build !darwin && !windows

package locale

func systemLanguageTag() string { return "" }
