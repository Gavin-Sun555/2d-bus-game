package game

import (
	"os"
	"path/filepath"
	"runtime"
	"testing"
)

// firstFixture returns the absolute path of the first candidate that is an
// existing regular file, or "" so the caller can skip. Replay fixtures live in
// the git-ignored dist/ folder and are written as ../dist/<capture>/<file>.
// testutil.Run moves tests into a temporary workspace, so relative candidates
// are resolved against this package's source directory, not the working
// directory.
func firstFixture(candidates ...string) string {
	_, source, _, _ := runtime.Caller(0)
	dir := filepath.Dir(source)
	for _, c := range candidates {
		if !filepath.IsAbs(c) {
			c = filepath.Join(dir, c)
		}
		if fi, err := os.Stat(c); err == nil && !fi.IsDir() {
			return c
		}
	}
	return ""
}

// loadSaveForTest builds a game of the given window size and loads the save
// at path into it, failing the test if the save cannot be loaded.
func loadSaveForTest(t testing.TB, path string, width, height int) *Game {
	t.Helper()
	g := NewGame(width, height)
	g.savePathOverride = path
	if err := g.LoadGame(); err != nil {
		t.Fatalf("LoadGame %s: %v", path, err)
	}
	return g
}
