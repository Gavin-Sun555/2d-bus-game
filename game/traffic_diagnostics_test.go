package game

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"
)

func TestGreenWaitReportSerialization(t *testing.T) {
	path := filepath.Join(t.TempDir(), "cache", "green-wait.json")
	reports := map[int]greenWaitReport{6: {BusID: 6, TargetStop: 11}}
	if err := writeGreenWaitReports(path, reports); err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var loaded map[int]greenWaitReport
	if err = json.Unmarshal(raw, &loaded); err != nil || loaded[6].TargetStop != 11 {
		t.Fatalf("invalid diagnostic: %v", err)
	}
	if _, err = os.Stat(path + ".tmp"); !os.IsNotExist(err) {
		t.Fatal("temporary file left behind")
	}
}
