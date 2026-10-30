package game

import (
	"fmt"
	"math"
	"strings"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sound"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
	"github.com/hajimehoshi/ebiten/v2"
)

type announcementState struct {
	speaker        sound.Speech
	active         bool
	mode           ui.AnnouncementMode
	language       locale.Language
	voice          sound.VoiceStyle
	volume         int
	route          *sim.Route
	line           int
	nextKey        string
	arrived        bool
	arrivalPos     sim.Vec2
	doors          bool
	departed       bool
	safetySpeaking bool
	pendingNext    *sim.BusStop
}

func (g *Game) EnableAnnouncements() error {
	if g.announcements != nil {
		return nil
	}
	speaker, err := sound.NewLocalSpeech()
	if err != nil {
		return err
	}
	g.announcements = &announcementState{speaker: speaker}
	return nil
}
func (g *Game) CloseAnnouncements() {
	if g.announcements != nil {
		g.announcements.speaker.Close()
		g.announcements = nil
	}
}
func (g *Game) resetAnnouncements() {
	if a := g.announcements; a != nil {
		a.speaker.Stop()
		g.announcements = &announcementState{speaker: a.speaker}
	}
}
func (g *Game) updateAnnouncements(driving bool) {
	a := g.announcements
	if a == nil {
		return
	}
	active := driving && g.announcementsAllowed()
	g.syncAnnouncements(active)
}
func (g *Game) trackAnnouncementDeparture() {
	a := g.announcements
	if a == nil || !a.active || g.Bus == nil {
		return
	}
	if a.arrived && g.Bus.Pos.Distance(a.arrivalPos) > 250 {
		a.arrived = false
	}
	if a.doors && !g.Bus.DoorsOpen {
		a.departed = true
	} else if g.Bus.DoorsOpen {
		a.departed = false
		a.pendingNext = nil
	}
	a.doors = g.Bus.DoorsOpen
	if a.departed && math.Abs(g.Bus.Speed) > 12 {
		a.departed = false
		a.safetySpeaking = true
		g.speakAnnouncement("车辆起步，请抓好扶手，注意安全。", "Bus starts, please hold on to the handrails and mind your step.")
		if next := a.pendingNext; next != nil {
			a.pendingNext = nil
			a.safetySpeaking = false
			zh, en := stopSpokenNames(next)
			g.enqueueAnnouncement("下一站是"+zh+"，请准备下车。", "The next stop is "+en+". Please get ready to alight.")
		}
	}
}
func (g *Game) syncAnnouncements(active bool) {
	a := g.announcements
	if a == nil {
		return
	}
	changed := a.volume != g.Settings.Volumes().Announcements || a.voice != g.Settings.AnnouncementVoice || a.mode != g.Settings.Announcements || a.language != g.Settings.Language || a.route != g.Route || a.line != g.PlayerLine
	if !active || changed {
		if a.active || changed {
			a.speaker.Stop()
		}
		a.active = false
		a.safetySpeaking = false
		a.pendingNext = nil
	}
	a.volume = g.Settings.Volumes().Announcements
	a.voice = g.Settings.AnnouncementVoice
	a.mode = g.Settings.Announcements
	a.language = g.Settings.Language
	if a.route != g.Route || a.line != g.PlayerLine {
		a.nextKey = ""
		a.arrived = false
		a.departed = false
		a.safetySpeaking = false
		a.pendingNext = nil
	}
	a.route = g.Route
	a.line = g.PlayerLine
	if active && !a.active {
		a.active = true
		if g.Bus != nil {
			a.doors = g.Bus.DoorsOpen
		}
	}
}
func (g *Game) buildAnnouncementUtterances(zh, en string) []sound.Utterance {
	a := g.announcements
	parts := []sound.Utterance{}
	volume := .9 * float64(g.Settings.Volumes().Announcements) / 100
	if a.mode == ui.AnnouncementsBilingual || a.language != locale.LanguageEN {
		parts = append(parts, sound.Utterance{Text: zh, Voice: a.voice, Volume: &volume})
	}
	if a.mode == ui.AnnouncementsBilingual || a.language == locale.LanguageEN {
		parts = append(parts, sound.Utterance{Text: en, English: true, Voice: a.voice, Volume: &volume})
	}
	return parts
}
func (g *Game) speakAnnouncement(zh, en string) {
	a := g.announcements
	if a == nil || !a.active {
		return
	}
	a.speaker.Replace(g.buildAnnouncementUtterances(zh, en))
}
func (g *Game) enqueueAnnouncement(zh, en string) {
	a := g.announcements
	if a == nil || !a.active {
		return
	}
	a.speaker.Enqueue(g.buildAnnouncementUtterances(zh, en))
}
func stopSpokenNames(s *sim.BusStop) (string, string) {
	zh, en := s.CustomNameZH, s.CustomNameEN
	raw := s.Name
	if i := strings.Index(raw, "("); i >= 0 {
		if zh == "" {
			zh = strings.TrimSpace(raw[:i])
		}
		if j := strings.Index(raw[i+1:], ")"); j >= 0 && en == "" {
			en = strings.TrimSpace(raw[i+1 : i+1+j])
		}
	}
	if zh == "" {
		zh = raw
	}
	if en == "" {
		en = raw
	}
	return zh, en
}
func (g *Game) announceNextStop(s *sim.BusStop) {
	a := g.announcements
	if a == nil || !a.active || g.Bus.DoorsOpen {
		return
	}
	key := fmt.Sprintf("%d/%d/%d", g.Route.TotalTrips, g.Route.CurrentIndex, s.ID)
	if a.nextKey == key {
		return
	}
	a.nextKey = key
	// On short legs, let the departure safety reminder play in full first and
	// immediately follow it with the next-stop forecast instead of interrupting.
	if a.departed {
		a.pendingNext = s
		return
	}
	zh, en := stopSpokenNames(s)
	if a.safetySpeaking {
		a.safetySpeaking = false
		g.enqueueAnnouncement("下一站是"+zh+"，请准备下车。", "The next stop is "+en+". Please get ready to alight.")
		return
	}
	g.speakAnnouncement("下一站是"+zh+"，请准备下车。", "The next stop is "+en+". Please get ready to alight.")
}
func (g *Game) announceArrival(s *sim.BusStop) {
	a := g.announcements
	if a == nil || !a.active {
		return
	}
	if a.arrived && s.Pos.Distance(a.arrivalPos) < s.Length {
		return
	}
	a.arrived = true
	a.arrivalPos = g.Bus.Pos
	a.doors = true
	a.departed = false
	a.safetySpeaking = false
	a.pendingNext = nil
	zh, en := stopSpokenNames(s)
	g.speakAnnouncement(fmt.Sprintf("%s到了，请从后门下车。欢迎乘坐%d路公交车。", zh, g.PlayerLine+1), fmt.Sprintf("This stop is %s. Please exit through the rear door. Welcome aboard Line %d bus.", en, g.PlayerLine+1))
}

func (g *Game) announcementsAllowed() bool {
	muted := g.vehicleAudio != nil && g.vehicleAudio.muted
	return g.State == StatePlaying && !g.IsPlanMode && !g.Settings.IsOpen && !g.NetworkOpen && !g.OperationsPanel.IsOpen && !g.savePicker.open && g.stopNameEditor == nil && ebiten.IsFocused() && !muted && g.Settings.Volumes().Announcements > 0 && g.Settings.Announcements != ui.AnnouncementsOff
}

// Give a complete bilingual message more lead time. World distance is px;
// speed is px/s, so faster driving moves the trigger farther from the stop.
func announcementLeadDistance(speed float64, bilingual bool) float64 {
	seconds, minimum := 12., 1800.
	if bilingual {
		seconds, minimum = 20., 2600.
	}
	return math.Min(10000, math.Max(minimum, math.Abs(speed)*seconds))
}
func (g *Game) shouldPreannounceStop(s *sim.BusStop, distance float64, alignedApproach bool) bool {
	if g.announcements == nil || !g.announcements.active {
		return false
	}
	remaining := distance
	if g.Settings.AutopilotEnabled && g.AutopilotDriver != nil && g.AutopilotDriver.PathTargetStopID() == s.ID {
		remaining = g.AutopilotDriver.LastRemaining
		// A cached leg must still describe the bus's actual position. Do not
		// announce on a stale zero-progress path far from its destination.
		if math.IsNaN(remaining) || math.IsInf(remaining, 0) || remaining < 0 || remaining+300 < distance {
			return false
		}
	} else if !alignedApproach {
		return false
	}
	return remaining < announcementLeadDistance(g.Bus.Speed, g.Settings.Announcements == ui.AnnouncementsBilingual)
}
