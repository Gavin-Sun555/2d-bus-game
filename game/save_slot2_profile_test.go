package game

import (
	"os"
	"path/filepath"
	"testing"

	ebiten "github.com/hajimehoshi/ebiten/v2"
)

func findSlot2ProfileSave() string {
	candidates := []string{
		filepath.Join(os.Getenv("HOME"), "Library", "Application Support", "2d-bus-game", "save-02.json"),
		filepath.Join("..", "dist", "slot2-headon-2026-11-02-1227", "slot2.json"),
		filepath.Join("dist", "slot2-headon-2026-11-02-1227", "slot2.json"),
	}
	for _, c := range candidates {
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			abs, _ := filepath.Abs(c)
			return abs
		}
	}
	return ""
}

func BenchmarkSaveSlot2Update(b *testing.B) {
	savePath := findSlot2ProfileSave()
	if savePath == "" {
		b.Skip("slot2 save file not found")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		b.Fatalf("LoadGame failed: %v", err)
	}

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
	}
	b.StopTimer()
	b.ReportMetric(float64(len(g.AIBusMgr.Buses)), "buses")
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
}

func BenchmarkSaveSlot2Draw(b *testing.B) {
	savePath := findSlot2ProfileSave()
	if savePath == "" {
		b.Skip("slot2 save file not found")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		b.Fatalf("LoadGame failed: %v", err)
	}
	screen := ebiten.NewImage(1440, 900)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		g.Draw(screen)
	}
	b.StopTimer()
}

func BenchmarkSaveSlot2FullFrame(b *testing.B) {
	savePath := findSlot2ProfileSave()
	if savePath == "" {
		b.Skip("slot2 save file not found")
	}
	g := NewGame(1440, 900)
	g.savePathOverride = savePath
	if err := g.LoadGame(); err != nil {
		b.Fatalf("LoadGame failed: %v", err)
	}
	screen := ebiten.NewImage(1440, 900)

	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if err := g.Update(); err != nil {
			b.Fatal(err)
		}
		g.Draw(screen)
	}
	b.StopTimer()
	b.ReportMetric(float64(len(g.AIBusMgr.Buses)), "buses")
	b.ReportMetric(float64(len(g.TrafficMgr.GetActiveCars(4))), "cars")
}

