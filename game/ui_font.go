package game

import (
	"bytes"
	"image/color"
	"os"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/text/v2"
)

var (
	chineseFontSource *text.GoTextFaceSource
	fontLoaded        bool
)

func init() {
	loadChineseFont()
}

func loadChineseFont() {
	fontPaths := []string{
		// macOS
		"/System/Library/Fonts/Hiragino Sans GB.ttc",
		"/System/Library/Fonts/STHeiti Light.ttc",
		"/System/Library/Fonts/STHeiti Medium.ttc",
		"/System/Library/Fonts/Supplemental/Songti.ttc",
		"/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
		"/Library/Fonts/Arial Unicode.ttf",
		// Windows
		`C:\Windows\Fonts\msyh.ttc`,
		`C:\Windows\Fonts\msyh.ttf`,
		`C:\Windows\Fonts\simsun.ttc`,
		// Linux
		"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
		"/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
	}

	for _, p := range fontPaths {
		data, err := os.ReadFile(p)
		if err != nil {
			continue
		}
		// Try collection first
		sources, err := text.NewGoTextFaceSourcesFromCollection(bytes.NewReader(data))
		if err == nil && len(sources) > 0 {
			chineseFontSource = sources[0]
			fontLoaded = true
			return
		}
		// Try single font
		source, err := text.NewGoTextFaceSource(bytes.NewReader(data))
		if err == nil {
			chineseFontSource = source
			fontLoaded = true
			return
		}
	}
}

// DrawText draws text using the loaded Chinese font, or falls back to DebugPrintAt
func DrawText(dst *ebiten.Image, str string, x, y float64, size float64, clr color.Color) {
	if fontLoaded && chineseFontSource != nil {
		face := &text.GoTextFace{
			Source: chineseFontSource,
			Size:   size,
		}
		op := &text.DrawOptions{}
		op.GeoM.Translate(x, y)
		if clr != nil {
			op.ColorScale.ScaleWithColor(clr)
		}
		text.Draw(dst, str, face, op)
		return
	}

	// Fallback to debug print
	ebitenutil.DebugPrintAt(dst, str, int(x), int(y))
}

// MeasureText measures the width and height of a string
func MeasureText(str string, size float64) (float64, float64) {
	if fontLoaded && chineseFontSource != nil {
		face := &text.GoTextFace{
			Source: chineseFontSource,
			Size:   size,
		}
		return text.Measure(str, face, 0)
	}
	return float64(len(str) * 7), 13.0
}
