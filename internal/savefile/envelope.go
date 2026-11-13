// Package savefile handles the on-disk envelope shared by manual saves and autosaves.
package savefile

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"time"
)

const Version = 1
const maxBytes = 32 << 20

type Envelope[M any] struct {
	Version  int             `json:"version"`
	SavedAt  time.Time       `json:"saved_at"`
	Name     string          `json:"name,omitempty"`
	Metadata *M              `json:"metadata,omitempty"`
	Checksum string          `json:"checksum"`
	Data     json.RawMessage `json:"data"`
}

func Encode[M any](payload []byte, name string, metadata *M, now time.Time) ([]byte, error) {
	digest := sha256.Sum256(payload)
	return json.Marshal(Envelope[M]{Version: Version, SavedAt: now, Name: name, Metadata: metadata, Checksum: hex.EncodeToString(digest[:]), Data: payload})
}

// Write replaces the target atomically after syncing the temporary file.
func Write(path string, data []byte) error {
	if len(data) > maxBytes {
		return fmt.Errorf("save exceeds size limit")
	}
	if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
		return err
	}
	f, err := os.CreateTemp(filepath.Dir(path), ".save-*.tmp")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	if _, err = f.Write(data); err != nil {
		f.Close()
		return err
	}
	if err = f.Sync(); err != nil {
		f.Close()
		return err
	}
	if err = f.Close(); err != nil {
		return err
	}
	return os.Rename(f.Name(), path)
}

// Read validates the version and checksum before returning a save payload.
func Read[M any](path string) (Envelope[M], error) {
	var envelope Envelope[M]
	raw, err := read(path)
	if err != nil {
		return envelope, err
	}
	if err = json.Unmarshal(raw, &envelope); err != nil {
		return envelope, fmt.Errorf("invalid save file: %w", err)
	}
	if envelope.Version != Version {
		return envelope, fmt.Errorf("unsupported save version %d", envelope.Version)
	}
	digest := sha256.Sum256(envelope.Data)
	if hex.EncodeToString(digest[:]) != envelope.Checksum {
		return envelope, fmt.Errorf("save checksum mismatch")
	}
	return envelope, nil
}

type Header[M any] struct {
	Version  int             `json:"version"`
	Name     string          `json:"name"`
	Metadata *M              `json:"metadata"`
	SavedAt  time.Time       `json:"saved_at"`
	Data     json.RawMessage `json:"data"`
}

// ReadHeader reads the fields displayed by the save picker. Read performs
// the full checksum validation before any state can be restored.
func ReadHeader[M any](path string) (Header[M], error) {
	var header Header[M]
	raw, err := read(path)
	if err != nil {
		return header, err
	}
	if err := json.Unmarshal(raw, &header); err != nil {
		return header, err
	}
	if header.Version != Version {
		return header, fmt.Errorf("unsupported save version %d", header.Version)
	}
	return header, nil
}

func read(path string) ([]byte, error) {
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer file.Close()
	raw, err := io.ReadAll(io.LimitReader(file, maxBytes+1))
	if err != nil {
		return nil, err
	}
	if len(raw) > maxBytes {
		return nil, fmt.Errorf("save exceeds size limit")
	}
	return raw, nil
}
