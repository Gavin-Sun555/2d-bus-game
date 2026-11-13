//go:build darwin

package locale

import (
	"context"
	"os/exec"
	"time"
)

func systemLanguageTag() string {
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	raw, err := exec.CommandContext(ctx, "/usr/bin/defaults", "read", "-g", "AppleLanguages").Output()
	if err != nil {
		return ""
	}
	return firstAppleLanguage(string(raw))
}
