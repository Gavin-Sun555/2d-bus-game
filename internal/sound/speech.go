package sound

import (
	"context"
	"fmt"
	"log"
	"os/exec"
	"runtime"
	"strings"
	"sync"
)

// Utterance is one language segment of a single replaceable announcement.
type Utterance struct {
	Text    string
	English bool
	Voice   VoiceStyle
	Volume  *float64 // nil keeps the system speech default; otherwise 0..1.
}
type Speech interface {
	Replace([]Utterance)
	Enqueue([]Utterance)
	Stop()
	Close()
}

// LocalSpeech runs OS speech outside the simulation thread. Every replacement
// cancels the complete previous announcement, including its remaining languages.
type LocalSpeech struct {
	mu       sync.Mutex
	cancel   context.CancelFunc
	gen      uint64
	queue    []Utterance
	speaking bool
	closed   bool
	wg       sync.WaitGroup
	run      func(context.Context, Utterance) error
}

func NewLocalSpeech() (*LocalSpeech, error) {
	if runtime.GOOS != "darwin" {
		return nil, fmt.Errorf("local announcements currently require macOS")
	}
	path, err := exec.LookPath("/usr/bin/say")
	if err != nil {
		return nil, err
	}
	voices, err := exec.Command(path, "-v", "?").Output()
	if err != nil {
		return nil, fmt.Errorf("list local speech voices: %w", err)
	}
	type voicePair struct{ chinese, english string }
	pairs := map[VoiceStyle]voicePair{}
	for style := VoiceFemale1; style <= VoiceMale2; style++ {
		zh, en := voicePreferences(style)
		pair := voicePair{resolveMacVoice(string(voices), "zh_CN", zh), resolveMacVoice(string(voices), "en_US", en)}
		if pair.chinese == "" || pair.english == "" {
			return nil, fmt.Errorf("local speech voices unavailable for style %d", style)
		}
		pairs[style] = pair
	}
	return &LocalSpeech{run: func(ctx context.Context, u Utterance) error {
		style := u.Voice
		if !style.Valid() {
			style = VoiceFemale1
		}
		voice := pairs[style].chinese
		if u.English {
			voice = pairs[style].english
		}
		// stdin keeps customized names out of command options and shell parsing.
		cmd := exec.CommandContext(ctx, path, "-v", voice, "-r", "185")
		cmd.Stdin = strings.NewReader(speechText(u))
		return cmd.Run()
	}}, nil
}
func (s *LocalSpeech) Replace(parts []Utterance) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed {
		return
	}
	if s.cancel != nil {
		s.cancel()
	}
	s.startLocked(parts)
}

// Enqueue appends parts to play immediately after the current utterance finishes,
// or starts playback right away if idle.
func (s *LocalSpeech) Enqueue(parts []Utterance) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed || len(parts) == 0 {
		return
	}
	if s.speaking {
		s.queue = append(s.queue, parts...)
		return
	}
	s.startLocked(parts)
}

func (s *LocalSpeech) startLocked(parts []Utterance) {
	ctx, cancel := context.WithCancel(context.Background())
	s.gen++
	gen := s.gen
	s.cancel = cancel
	s.queue = append([]Utterance(nil), parts...)
	s.speaking = true
	s.wg.Add(1)
	go func() {
		defer s.wg.Done()
		defer cancel()
		for {
			s.mu.Lock()
			if s.gen != gen || ctx.Err() != nil || len(s.queue) == 0 {
				if s.gen == gen {
					s.speaking = false
					s.cancel = nil
				}
				s.mu.Unlock()
				return
			}
			u := s.queue[0]
			s.queue = s.queue[1:]
			s.mu.Unlock()
			if u.Text != "" {
				if err := s.run(ctx, u); err != nil {
					if ctx.Err() == nil {
						log.Printf("Announcement speech failed: %v", err)
					}
					s.mu.Lock()
					if s.gen == gen {
						s.queue = nil
						s.speaking = false
						s.cancel = nil
					}
					s.mu.Unlock()
					return
				}
			}
		}
	}()
}

func (s *LocalSpeech) Stop() {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.gen++
	s.queue = nil
	s.speaking = false
	if s.cancel != nil {
		s.cancel()
		s.cancel = nil
	}
}
func (s *LocalSpeech) Close() {
	s.mu.Lock()
	s.closed = true
	s.gen++
	s.queue = nil
	s.speaking = false
	if s.cancel != nil {
		s.cancel()
		s.cancel = nil
	}
	s.mu.Unlock()
	s.wg.Wait()
}

// Match qualified voice names as well as older macOS short names. Never fall
// back to an arbitrary first voice: that can silently change the chosen gender.
func matchMacVoice(list, language, preferred string) string {
	for _, line := range strings.Split(list, "\n") {
		i := strings.Index(line, language)
		if i < 0 {
			continue
		}
		name := strings.TrimSpace(line[:i])
		if name == preferred || strings.HasPrefix(name, preferred+" (") {
			return name
		}
	}
	return ""
}

func speechText(u Utterance) string {
	if u.Volume == nil {
		return u.Text
	}
	// Treat station names as text, not embedded speech commands.
	text := strings.ReplaceAll(strings.ReplaceAll(u.Text, "[[", "［［"), "]]", "］］")
	return fmt.Sprintf("[[volm %.3f]]%s", *u.Volume, text)
}
