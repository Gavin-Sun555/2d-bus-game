package game

import (
	"os"
	"testing"

	"github.com/Gavin-Sun555/2d-bus-game/internal/testutil"
)

func TestMain(m *testing.M) {
	if os.Getenv("BUS_NATIVE_OUTPUT") != "" {
		os.Exit(runNativePerformance())
	}
	if output := os.Getenv("BUSGAME_AUTOSAVE_PREVIEW"); output != "" {
		os.Exit(runAutosavePreview(output))
	}
	if output := os.Getenv("BUSGAME_STATION_NAME_PREVIEW"); output != "" {
		os.Exit(runStopNamePreview(output))
	}
	os.Exit(testutil.Run(m))
}
