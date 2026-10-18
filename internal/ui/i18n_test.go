package ui

import (
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
)

func TestI18nLocalization(t *testing.T) {
	// 1. Initial language is Chinese
	locale.SetLanguage(locale.LanguageZH)
	if locale.CurrentLanguage != locale.LanguageZH {
		t.Fatalf("Expected initial language LanguageZH, got %v", locale.CurrentLanguage)
	}

	if val := locale.T("测试中文", "Test English"); val != "测试中文" {
		t.Fatalf("Expected Chinese string, got %s", val)
	}

	if val := locale.Tf("第 %d 站", "Stop #%d", 3); val != "第 3 站" {
		t.Fatalf("Expected formatted Chinese string, got %s", val)
	}

	// 2. Toggle to English
	toggled := locale.ToggleLanguage()
	if toggled != locale.LanguageEN || locale.CurrentLanguage != locale.LanguageEN {
		t.Fatalf("Expected toggled language LanguageEN, got %v", toggled)
	}

	if val := locale.T("测试中文", "Test English"); val != "Test English" {
		t.Fatalf("Expected English string, got %s", val)
	}

	if val := locale.Tf("第 %d 站", "Stop #%d", 3); val != "Stop #3" {
		t.Fatalf("Expected formatted English string, got %s", val)
	}

	// 3. BusStop LocalizedName
	stop := &sim.BusStop{Name: "市民广场 (Civic Plaza)"}
	if name := stop.LocalizedName(); name != "Civic Plaza" {
		t.Fatalf("Expected extracted English stop name 'Civic Plaza', got '%s'", name)
	}

	// Toggle back to Chinese
	locale.ToggleLanguage()
	if name := stop.LocalizedName(); name != "市民广场 (Civic Plaza)" {
		t.Fatalf("Expected full Chinese stop name in ZH mode, got '%s'", name)
	}

	// 4. Settings integration
	settings := NewSettings()
	if settings.Language != locale.LanguageZH {
		t.Fatalf("Expected Settings to initialize with LanguageZH")
	}
}
