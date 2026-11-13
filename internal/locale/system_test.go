package locale

import "testing"

func TestSystemLanguageTags(t *testing.T) {
	for _, tag := range []string{"zh", "zh-CN", "zh-TW", "zh-HK", "zh-Hans-CN", "zh-Hant", "zh_CN.UTF-8", " ZH_sg "} {
		if languageForTag(tag) != LanguageZH {
			t.Errorf("Chinese tag %q did not select Chinese", tag)
		}
	}
	for _, tag := range []string{"", "en", "en-CN", "ja-JP", "fr_FR.UTF-8", "de-DE", "ko-KR", "C", "POSIX", "zhx"} {
		if languageForTag(tag) != LanguageEN {
			t.Errorf("non-Chinese tag %q did not select English", tag)
		}
	}
}
func TestSystemLanguageUsesFirstPreference(t *testing.T) {
	for _, tc := range []struct {
		raw, want string
		language  Language
	}{
		{"(\n    \"en-CN\",\n    \"zh-Hans-CN\"\n)", "en-CN", LanguageEN},
		{"(\"zh-Hant-TW\", en-US)", "zh-Hant-TW", LanguageZH},
		{"(ja-JP,zh-Hans)", "ja-JP", LanguageEN},
		{"()", "", LanguageEN},
	} {
		tag := firstAppleLanguage(tc.raw)
		if tag != tc.want || languageForTag(tag) != tc.language {
			t.Errorf("preference %q got %q", tc.raw, tag)
		}
	}
}
func TestSystemLanguageEnvironmentPrecedence(t *testing.T) {
	env := map[string]string{"LANG": "zh_CN.UTF-8", "LANGUAGE": "en:zh", "LC_MESSAGES": "ja_JP.UTF-8", "LC_ALL": "zh_TW.UTF-8"}
	get := func(k string) string { return env[k] }
	for _, tc := range []struct{ remove, want string }{{"", "zh_TW.UTF-8"}, {"LC_ALL", "ja_JP.UTF-8"}, {"LC_MESSAGES", "en"}, {"LANGUAGE", "zh_CN.UTF-8"}, {"LANG", ""}} {
		delete(env, tc.remove)
		if got := environmentLanguage(get); got != tc.want {
			t.Fatalf("after removing %q got %q want %q", tc.remove, got, tc.want)
		}
	}
}
func TestSystemLanguageNativeStartup(t *testing.T) {
	before := CurrentLanguage
	defer SetLanguage(before)
	InitSystemLanguage()
	if CurrentLanguage != LanguageZH && CurrentLanguage != LanguageEN {
		t.Fatal("invalid startup language")
	}
	t.Logf("system UI tag=%q; selected language=%v", systemLanguageTag(), CurrentLanguage)
	ToggleLanguage()
	if CurrentLanguage == SystemLanguage() {
		t.Fatal("manual switch was overridden")
	}
}
