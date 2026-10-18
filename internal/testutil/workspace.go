// Package testutil provides isolated filesystem fixtures for simulation tests.
package testutil

import (
	"fmt"
	"os"
	"path/filepath"
	"testing"
)

// Run copies the project's custom map into a temporary working directory.
// Package moves must not change which map a test loads, and reload tests must
// never modify the user's custom_map.json or scaffold files in source folders.
func Run(m *testing.M) int {
	root, err := os.Getwd()
	if err != nil {
		panic(err)
	}
	for {
		if _, err = os.Stat(filepath.Join(root, "go.mod")); err == nil {
			break
		}
		parent := filepath.Dir(root)
		if parent == root {
			panic("cannot locate test project root")
		}
		root = parent
	}
	data, err := os.ReadFile(filepath.Join(root, "custom_map.json"))
	if err != nil && !os.IsNotExist(err) {
		panic(fmt.Errorf("read custom map fixture: %w", err))
	}
	workspace, err := os.MkdirTemp("", "bus-game-tests-")
	if err != nil {
		panic(err)
	}
	defer os.RemoveAll(workspace)
	if len(data) > 0 {
		if err = os.WriteFile(filepath.Join(workspace, "custom_map.json"), data, 0600); err != nil {
			panic(err)
		}
	}
	previous, err := os.Getwd()
	if err != nil {
		panic(err)
	}
	if err = os.Chdir(workspace); err != nil {
		panic(err)
	}
	defer os.Chdir(previous)
	return m.Run()
}
