package integration_test

import (
	"testing"
)

// checkReleaseQualificationGate skips the exhaustive qualification tests when running in -short mode.
// Routine feature development uses 'go test -short ./...' or 'make test'.
// Prior to publishing a release, 'go test -run TestReleaseQualification ./...' or 'make test-qualify' runs the full suite.
func checkReleaseQualificationGate(t *testing.T) {
	if testing.Short() {
		t.Skip("Skipping Release Qualification suite in -short mode. Run without -short or 'make test-qualify' to qualify release.")
	}
}
