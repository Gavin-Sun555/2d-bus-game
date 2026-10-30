package ui

import (
	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sound"
)

type AnnouncementMode int

const (
	AnnouncementsFollowLanguage AnnouncementMode = iota
	AnnouncementsOff
	AnnouncementsBilingual
)

func (m AnnouncementMode) Valid() bool {
	return m >= AnnouncementsFollowLanguage && m <= AnnouncementsBilingual
}
func (s *Settings) CycleAnnouncements() { s.Announcements = (s.Announcements + 1) % 3 }
func (m AnnouncementMode) Label() string {
	switch m {
	case AnnouncementsFollowLanguage:
		return locale.T("跟随游戏语言", "Game language")
	case AnnouncementsBilingual:
		return locale.T("中英双语", "Chinese + English")
	default:
		return locale.T("关闭", "Off")
	}
}

func (s *Settings) AnnouncementVoiceLabel() string {
	switch s.AnnouncementVoice {
	case sound.VoiceMale1:
		return locale.T("男声 1 · Reed", "Male 1 · Reed")
	case sound.VoiceFemale2:
		return locale.T("女声 2 · Shelley", "Female 2 · Shelley")
	case sound.VoiceMale2:
		return locale.T("男声 2 · Eddy", "Male 2 · Eddy")
	default:
		return locale.T("女声 1 · 标准", "Female 1 · Standard")
	}
}
