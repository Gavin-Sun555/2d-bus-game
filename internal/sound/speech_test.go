package sound

import (
	"context"
	"os"
	"strings"
	"sync"
	"testing"
	"time"
)

func TestSpeechReplacementCancelsRemainingLanguages(t *testing.T) {
	started := make(chan string, 8)
	stopped := make(chan string, 8)
	s := &LocalSpeech{run: func(ctx context.Context, u Utterance) error {
		started <- u.Text
		<-ctx.Done()
		stopped <- u.Text
		return ctx.Err()
	}}
	defer s.Close()
	s.Replace([]Utterance{{Text: "old Chinese"}, {Text: "old English", English: true}})
	expect := func(ch <-chan string, want string) {
		t.Helper()
		select {
		case got := <-ch:
			if got != want {
				t.Fatalf("got %s want %s", got, want)
			}
		case <-time.After(time.Second):
			t.Fatal("speech worker stalled")
		}
	}
	expect(started, "old Chinese")
	s.Replace([]Utterance{{Text: "new arrival"}})
	expect(stopped, "old Chinese")
	expect(started, "new arrival")
	s.Stop()
	expect(stopped, "new arrival")
	s.Close()
	s.Replace([]Utterance{{Text: "closed"}})
	select {
	case text := <-started:
		t.Fatalf("obsolete speech started: %s", text)
	default:
	}
}
func TestSpeechConcurrentReplaceAndClose(t *testing.T) {
	s := &LocalSpeech{run: func(ctx context.Context, _ Utterance) error { <-ctx.Done(); return ctx.Err() }}
	var wg sync.WaitGroup
	for i := 0; i < 20; i++ {
		wg.Add(1)
		go func() { defer wg.Done(); s.Replace([]Utterance{{Text: "station"}}); s.Stop() }()
	}
	s.Close()
	wg.Wait()
}

func TestSpeechEnqueuePlaysImmediatelyAfterCurrentWithoutInterrupting(t *testing.T) {
	started := make(chan string, 8)
	finish := make(chan struct{}, 8)
	s := &LocalSpeech{run: func(ctx context.Context, u Utterance) error {
		started <- u.Text
		select {
		case <-finish:
			return nil
		case <-ctx.Done():
			t.Errorf("utterance %q was unexpectedly interrupted", u.Text)
			return ctx.Err()
		}
	}}
	defer s.Close()

	s.Replace([]Utterance{{Text: "safety-zh"}, {Text: "safety-en", English: true}})
	if got := <-started; got != "safety-zh" {
		t.Fatalf("got %q, want safety-zh", got)
	}
	// Enqueue next-stop forecast while safety reminder is still speaking.
	s.Enqueue([]Utterance{{Text: "next-zh"}, {Text: "next-en", English: true}})

	finish <- struct{}{}
	if got := <-started; got != "safety-en" {
		t.Fatalf("got %q, want safety-en", got)
	}
	finish <- struct{}{}
	if got := <-started; got != "next-zh" {
		t.Fatalf("got %q, want next-zh", got)
	}
	finish <- struct{}{}
	if got := <-started; got != "next-en" {
		t.Fatalf("got %q, want next-en", got)
	}
	finish <- struct{}{}
}

// Opt-in native smoke: exercise real /usr/bin/say stdin playback and cancellation.
func TestNativeAnnouncementSpeech(t *testing.T) {
	if os.Getenv("BUSGAME_NATIVE_SPEECH_TEST") != "1" {
		t.Skip("native speaker opt-in")
	}
	s, err := NewLocalSpeech()
	if err != nil {
		t.Fatal(err)
	}
	defer s.Close()
	for style := VoiceFemale1; style <= VoiceMale2; style++ {
		for _, u := range []Utterance{{Text: "下一站是人民广场，请准备下车。"}, {Text: "The next stop is Civic Center. Please get ready to alight.", English: true}} {
			u.Voice = style
			ctx, cancel := context.WithCancel(context.Background())
			done := make(chan error, 1)
			go func() { done <- s.run(ctx, u) }()
			timer := time.NewTimer(600 * time.Millisecond)
			select {
			case err := <-done:
				timer.Stop()
				cancel()
				t.Fatalf("speech exited before interruption: %v", err)
			case <-timer.C:
			}
			start := time.Now()
			cancel()
			select {
			case <-done:
				if time.Since(start) > 500*time.Millisecond {
					t.Fatal("speech cancellation too slow")
				}
			case <-time.After(time.Second):
				t.Fatal("native speech did not stop")
			}
		}
	}
}

func TestVoicePresetsUseQualifiedNamesAndRespectVoiceFamily(t *testing.T) {
	list := "Eddy (Chinese (China mainland)) zh_CN # hello\nReed (Chinese (China mainland)) zh_CN # hello\nShelley (Chinese (China mainland)) zh_CN # hello\nTingting (Chinese (China mainland)) zh_CN # hello\nSamantha (English (US)) en_US # hello\nReed (English (US)) en_US # hello\nShelley (English (US)) en_US # hello\nEddy (English (US)) en_US # hello\n"
	for style := VoiceFemale1; style <= VoiceMale2; style++ {
		zh, en := voicePreferences(style)
		if got := resolveMacVoice(list, "zh_CN", zh); !strings.HasPrefix(got, zh[0]+" (") {
			t.Fatalf("style %d Mandarin=%q", style, got)
		}
		if got := resolveMacVoice(list, "en_US", en); !strings.HasPrefix(got, en[0]+" (") {
			t.Fatalf("style %d English=%q", style, got)
		}
	}
	if got := resolveMacVoice("Eddy zh_CN # male\n", "zh_CN", []string{"Tingting", "Shelley"}); got != "" {
		t.Fatal("female preset fell back to an unrelated male voice")
	}
}
