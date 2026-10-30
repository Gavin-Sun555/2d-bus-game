package sound

// VoiceStyle selects a paired Mandarin/English local voice. Zero retains a
// female default for saves created before voice selection was introduced.
type VoiceStyle int

const (
	VoiceFemale1 VoiceStyle = iota
	VoiceMale1
	VoiceFemale2
	VoiceMale2
)

func (v VoiceStyle) Valid() bool { return v >= VoiceFemale1 && v <= VoiceMale2 }
func voicePreferences(v VoiceStyle) ([]string, []string) {
	switch v {
	case VoiceMale1:
		return []string{"Reed", "Eddy", "Rocko", "Grandpa"}, []string{"Reed", "Eddy", "Rocko", "Grandpa"}
	case VoiceFemale2:
		return []string{"Shelley", "Flo", "Tingting"}, []string{"Shelley", "Flo", "Samantha"}
	case VoiceMale2:
		return []string{"Eddy", "Rocko", "Reed", "Grandpa"}, []string{"Eddy", "Rocko", "Reed", "Grandpa"}
	default:
		return []string{"Tingting", "Shelley", "Flo"}, []string{"Samantha", "Shelley", "Flo"}
	}
}
func resolveMacVoice(list, language string, preferred []string) string {
	for _, name := range preferred {
		if voice := matchMacVoice(list, language, name); voice != "" {
			return voice
		}
	}
	return ""
}
