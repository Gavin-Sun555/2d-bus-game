package game

import (
	"math"
	"path/filepath"
	"strings"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sound"
	"github.com/Gavin-Sun555/2d-bus-game/internal/ui"
)

type speechRecorder struct {
	events        [][]sound.Utterance
	queued        [][]sound.Utterance
	stops, closed int
}

func (s *speechRecorder) Replace(u []sound.Utterance) {
	s.events = append(s.events, append([]sound.Utterance(nil), u...))
}
func (s *speechRecorder) Enqueue(u []sound.Utterance) {
	copyU := append([]sound.Utterance(nil), u...)
	s.events = append(s.events, copyU)
	s.queued = append(s.queued, copyU)
}
func (s *speechRecorder) Stop()  { s.stops++ }
func (s *speechRecorder) Close() { s.closed++ }
func announcementTestGame(t *testing.T) (*Game, *speechRecorder) {
	t.Helper()
	g := saveTestGame(t)
	r := &speechRecorder{}
	g.announcements = &announcementState{speaker: r}
	g.Settings.Announcements = ui.AnnouncementsFollowLanguage
	g.Settings.Language = locale.LanguageZH
	g.syncAnnouncements(true)
	return g, r
}
func TestAnnouncementsLanguagesAndCustomNames(t *testing.T) {
	g, r := announcementTestGame(t)
	s := g.World.Stops[g.Route.GetCurrentTargetStopID()]
	s.CustomNameZH = "人民广场"
	s.CustomNameEN = "Civic Center"
	g.Bus.DoorsOpen = false
	g.announceNextStop(s)
	g.announceNextStop(s)
	if len(r.events) != 1 || len(r.events[0]) != 1 || r.events[0][0].English || r.events[0][0].Text != "下一站是人民广场，请准备下车。" {
		t.Fatalf("wrong or repeated next stop: %+v", r.events)
	}
	g.Settings.Announcements = ui.AnnouncementsBilingual
	g.syncAnnouncements(true)
	g.announceArrival(s)
	got := r.events[1]
	if len(got) != 2 || !got[1].English || !strings.Contains(got[0].Text, "人民广场到了") || !strings.Contains(got[1].Text, "This stop is Civic Center.") || !strings.Contains(got[1].Text, "Welcome aboard Line 1 bus") {
		t.Fatalf("wrong bilingual arrival: %+v", got)
	}
	g.announceArrival(s)
	if len(r.events) != 2 {
		t.Fatal("arrival repeated each frame")
	}
	g.Settings.Language = locale.LanguageEN
	g.Settings.Announcements = ui.AnnouncementsFollowLanguage
	g.syncAnnouncements(true)
	g.speakAnnouncement("车辆起步，请抓好扶手，注意安全。", "Bus starts, please hold on to the handrails and mind your step.")
	if got := r.events[2]; len(got) != 1 || !got[0].English || got[0].Text != "Bus starts, please hold on to the handrails and mind your step." {
		t.Fatal(got)
	}
	g.Settings.Announcements = ui.AnnouncementsOff
	g.syncAnnouncements(false)
	g.speakAnnouncement("关闭", "off")
	if len(r.events) != 3 {
		t.Fatal("disabled announcements spoke")
	}
	stops := r.stops
	g.syncAnnouncements(false)
	if r.stops != stops {
		t.Fatal("stopped every paused frame")
	}
}
func TestAnnouncementsPersistenceAndSessionReset(t *testing.T) {
	g, r := announcementTestGame(t)
	g.Settings.Announcements = ui.AnnouncementsBilingual
	g.Settings.AnnouncementVoice = sound.VoiceMale2
	g.savePathOverride = filepath.Join(t.TempDir(), "save.json")
	if err := g.SaveGame(); err != nil {
		t.Fatal(err)
	}
	if err := g.LoadGame(); err != nil {
		t.Fatal(err)
	}
	if g.Settings.AnnouncementVoice != sound.VoiceMale2 || g.Settings.Announcements != ui.AnnouncementsBilingual || g.announcements == nil || g.announcements.speaker != r || g.announcements.active {
		t.Fatal("save did not preserve preference/reset speech")
	}
	g.syncAnnouncements(true)
	g.SetMapTheme(sim.MapThemeCoastalBay)
	if g.announcements.active || g.announcements.speaker != r {
		t.Fatal("map change left stale speech")
	}
	g.CloseAnnouncements()
	if r.closed != 1 {
		t.Fatal("speech not closed")
	}
}
func TestAnnouncementDefaultNames(t *testing.T) {
	s := &sim.BusStop{Name: "街区279 · 北行站 (Block 279 Northbound)"}
	zh, en := stopSpokenNames(s)
	if zh != "街区279 · 北行站" || en != "Block 279 Northbound" {
		t.Fatal(zh, en)
	}
}

func TestAnnouncementSafetyOnlyOnDeparture(t *testing.T) {
	g, r := announcementTestGame(t)
	s := g.World.Stops[g.Route.GetCurrentTargetStopID()]
	g.Bus.DoorsOpen = true
	g.announceArrival(s)
	g.Bus.DoorsOpen = false
	g.Bus.Speed = 0
	g.trackAnnouncementDeparture()
	if len(r.events) != 1 {
		t.Fatal("safety announced while waiting at stop")
	}
	g.Bus.Speed = 30
	g.trackAnnouncementDeparture()
	g.trackAnnouncementDeparture()
	if len(r.events) != 2 || r.events[1][0].Text != "车辆起步，请抓好扶手，注意安全。" {
		t.Fatalf("wrong or repeated safety reminder: %+v", r.events)
	}
}

func TestAnnouncementVoiceSelectionInterruptsAndAppliesToBothLanguages(t *testing.T) {
	g, r := announcementTestGame(t)
	g.Settings.Announcements = ui.AnnouncementsBilingual
	for style := sound.VoiceFemale1; style <= sound.VoiceMale2; style++ {
		stops := r.stops
		g.Settings.AnnouncementVoice = style
		g.syncAnnouncements(true)
		if style != sound.VoiceFemale1 && r.stops <= stops {
			t.Fatal("changing voices did not cancel previous speech")
		}
		g.speakAnnouncement("车辆起步，请抓好扶手，注意安全。", "Bus starts, please hold on to the handrails and mind your step.")
		got := r.events[len(r.events)-1]
		if len(got) != 2 || got[0].Voice != style || got[1].Voice != style {
			t.Fatalf("selected voice not used: %+v", got)
		}
	}
}

func TestNextStopPreannouncementStartsEarlierAndOnlyOnce(t *testing.T) {
	g, r := announcementTestGame(t)
	g.Settings.AutopilotEnabled = false
	g.Settings.Announcements = ui.AnnouncementsBilingual
	g.syncAnnouncements(true)
	s := g.World.Stops[g.Route.GetCurrentTargetStopID()]
	g.Bus.Pos = s.FrontBerthPosition().Sub(sim.UnitFromAngle(s.Heading).Mul(3000))
	g.Bus.Heading = s.Heading
	g.Bus.Speed = 200
	g.Bus.DoorsOpen = false
	g.stopBellRung = true // Do not skip the target during this presentation regression.
	g.updateStopApproach()
	g.updateStopApproach()
	if len(r.events) != 1 || !strings.HasPrefix(r.events[0][0].Text, "下一站是") {
		t.Fatalf("early next stop missing or repeated: %+v", r.events)
	}
	// Driving away / along the opposite carriageway must not trigger a report.
	g.announcements.nextKey = ""
	g.Bus.Heading = sim.NormalizeAngle(s.Heading + math.Pi)
	g.updateStopApproach()
	if len(r.events) != 1 {
		t.Fatal("wrong-way approach announced")
	}
	if announcementLeadDistance(300, true) <= announcementLeadDistance(300, false) || announcementLeadDistance(600, false) <= announcementLeadDistance(200, false) {
		t.Fatal("speed/bilingual lead time not increased")
	}
}

func TestEarlyAnnouncementUsesAutopilotRouteProgressBeforeTurn(t *testing.T) {
	g, _ := announcementTestGame(t)
	s := g.World.Stops[g.Route.GetCurrentTargetStopID()]
	snapshot := g.AutopilotDriver.Snapshot()
	snapshot.PathToID = s.ID
	snapshot.DrivePath = nil
	snapshot.Public.LastRemaining = 3000
	var err error
	g.AutopilotDriver, err = snapshot.Restore(g.World)
	if err != nil {
		t.Fatal(err)
	}
	g.Settings.AutopilotEnabled = true
	g.Settings.Announcements = ui.AnnouncementsBilingual
	g.Bus.Speed = 200
	if !g.shouldPreannounceStop(s, 1000, false) {
		t.Fatal("turn-before-stop still prevents early report")
	}
	g.AutopilotDriver.LastRemaining = 9000
	if g.shouldPreannounceStop(s, 1000, true) {
		t.Fatal("nearby stop on much later path announced too early")
	}
	g.AutopilotDriver.LastRemaining = 0
	if g.shouldPreannounceStop(s, 1000, true) {
		t.Fatal("stale zero remaining path triggered report")
	}
}

func TestShortLegNextStopFollowsAfterDepartureSafetyReminder(t *testing.T) {
	// Case 1: Next stop is within lead distance right after closing doors (before speed > 12).
	// The departure safety reminder plays first when accelerating, followed immediately by next stop.
	g, r := announcementTestGame(t)
	s := g.World.Stops[g.Route.GetCurrentTargetStopID()]
	g.announcements.doors = true
	g.Bus.DoorsOpen = false
	g.Bus.Speed = 0
	g.trackAnnouncementDeparture() // Close doors, still waiting before accelerating.
	g.announceNextStop(s)
	if len(r.events) != 0 {
		t.Fatalf("next stop spoke before departure safety reminder: %+v", r.events)
	}
	g.Bus.Speed = 40
	g.trackAnnouncementDeparture()
	g.trackAnnouncementDeparture()
	if len(r.events) != 2 || !strings.HasPrefix(r.events[0][0].Text, "车辆起步") || !strings.HasPrefix(r.events[1][0].Text, "下一站是") || len(r.queued) != 1 {
		t.Fatalf("expected safety reminder followed immediately by queued next-stop forecast, got events=%+v queued=%+v", r.events, r.queued)
	}

	// Case 2: Bus accelerates past speed > 12 (starting safety reminder), and then enters next-stop range.
	// Next stop must be enqueued after the safety reminder rather than replacing/interrupting it.
	g2, r2 := announcementTestGame(t)
	s2 := g2.World.Stops[g2.Route.GetCurrentTargetStopID()]
	g2.announcements.doors = true
	g2.Bus.DoorsOpen = false
	g2.Bus.Speed = 40
	g2.trackAnnouncementDeparture()
	g2.announceNextStop(s2)
	if len(r2.events) != 2 || !strings.HasPrefix(r2.events[0][0].Text, "车辆起步") || !strings.HasPrefix(r2.events[1][0].Text, "下一站是") || len(r2.queued) != 1 {
		t.Fatalf("expected next-stop forecast to enqueue behind active safety reminder, got events=%+v queued=%+v", r2.events, r2.queued)
	}
}
