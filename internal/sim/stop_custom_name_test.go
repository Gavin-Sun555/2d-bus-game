package sim

import (
	"encoding/json"
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"strings"
	"testing"
)

func TestCustomStationNamesInBothLanguages(t *testing.T) {
	old := locale.CurrentLanguage
	defer locale.SetLanguage(old)
	s := &BusStop{Name: "原站名 (Original Stop)", CustomNameZH: "人民广场", CustomNameEN: "People Square"}
	for _, lang := range []locale.Language{locale.LanguageZH, locale.LanguageEN} {
		locale.SetLanguage(lang)
		want := s.CustomNameZH
		if lang == locale.LanguageEN {
			want = s.CustomNameEN
		}
		if s.CleanName() != want || s.LocalizedName() != want {
			t.Fatal("custom names not displayed")
		}
	}
	data, _ := json.Marshal(s)
	if strings.Contains(string(data), "人民广场") || strings.Contains(string(data), "People Square") {
		t.Fatal("local preferences leaked into save")
	}
	s.CustomNameZH = ""
	s.CustomNameEN = ""
	locale.SetLanguage(locale.LanguageEN)
	if s.LocalizedName() != "Original Stop" {
		t.Fatal("original fallback lost")
	}
	locale.SetLanguage(locale.LanguageZH)
	if s.CleanName() != "原站名" {
		t.Fatal("Chinese fallback lost")
	}
}
