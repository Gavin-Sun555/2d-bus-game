package game

import "fmt"

// Language represents the UI localization language
type Language int

const (
	LanguageZH Language = iota // 简体中文 (Simplified Chinese)
	LanguageEN                 // English
)

// CurrentLanguage stores the currently active language across UI systems
var CurrentLanguage Language = LanguageZH

// SetLanguage updates the current localization language
func SetLanguage(lang Language) {
	CurrentLanguage = lang
}

// ToggleLanguage toggles between Chinese and English
func ToggleLanguage() Language {
	if CurrentLanguage == LanguageZH {
		CurrentLanguage = LanguageEN
	} else {
		CurrentLanguage = LanguageZH
	}
	return CurrentLanguage
}

// T returns the localized string based on CurrentLanguage
func T(zh, en string) string {
	if CurrentLanguage == LanguageEN {
		return en
	}
	return zh
}

// Tf returns a formatted localized string based on CurrentLanguage
func Tf(zhFormat, enFormat string, args ...any) string {
	if CurrentLanguage == LanguageEN {
		return fmt.Sprintf(enFormat, args...)
	}
	return fmt.Sprintf(zhFormat, args...)
}
