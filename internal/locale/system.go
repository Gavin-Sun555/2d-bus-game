package locale

import (
	"os"
	"strings"
	"unicode"
)

// InitSystemLanguage runs once at application startup, before creating settings.
// Manual switching and the language stored in a loaded save remain available.
func InitSystemLanguage() { SetLanguage(SystemLanguage()) }

func SystemLanguage() Language {
	tag := systemLanguageTag()
	if tag == "" {
		tag = environmentLanguage(os.Getenv)
	}
	return languageForTag(tag)
}
func languageForTag(tag string) Language {
	tag = strings.ToLower(strings.TrimSpace(tag))
	base := strings.FieldsFunc(tag, func(r rune) bool { return r == '-' || r == '_' || r == '.' || r == '@' || r == ':' })
	if len(base) > 0 && base[0] == "zh" {
		return LanguageZH
	}
	return LanguageEN
}
func environmentLanguage(getenv func(string) string) string {
	for _, key := range []string{"LC_ALL", "LC_MESSAGES", "LANGUAGE", "LANG"} {
		if value := strings.TrimSpace(getenv(key)); value != "" {
			return strings.SplitN(value, ":", 2)[0]
		}
	}
	return ""
}

// defaults prints AppleLanguages as a quoted, comma-separated array. Only the
// first preferred language counts: English with Chinese as a fallback is English.
func firstAppleLanguage(value string) string {
	tags := strings.FieldsFunc(value, func(r rune) bool { return unicode.IsSpace(r) || r == '(' || r == ')' || r == ',' || r == '"' })
	if len(tags) > 0 {
		return tags[0]
	}
	return ""
}
