package game

import (
	"bytes"
	_ "embed"
	"image"
	"image/color"
	_ "image/png"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

//go:embed assets/logo_512.png
var menuLogoBytes []byte

var menuLogoImg *ebiten.Image

func (g *Game) updateMainMenu() {
	// 1. Keyboard shortcuts: [1]..[6] / [C]
	if inpututil.IsKeyJustPressed(ebiten.Key1) {
		g.SelectedMapTheme = sim.MapThemeMetropolis
	} else if inpututil.IsKeyJustPressed(ebiten.Key2) {
		g.SelectedMapTheme = sim.MapThemeCoastalBay
	} else if inpututil.IsKeyJustPressed(ebiten.Key3) {
		g.SelectedMapTheme = sim.MapThemeRiverValley
	} else if inpututil.IsKeyJustPressed(ebiten.Key4) {
		g.SelectedMapTheme = sim.MapThemeTwinCities
	} else if inpututil.IsKeyJustPressed(ebiten.Key5) {
		g.SelectedMapTheme = sim.MapThemeCustom
	} else if inpututil.IsKeyJustPressed(ebiten.Key6) || inpututil.IsKeyJustPressed(ebiten.KeyC) {
		g.SelectedMapTheme = sim.MapThemeCustom
	}

	// Hotkey L: Toggle interface language
	if inpututil.IsKeyJustPressed(ebiten.KeyL) {
		newLang := locale.ToggleLanguage()
		g.Settings.Language = newLang
	}

	// Hotkey G: Toggle Metropolis Large Road Network (4x block distance)
	if inpututil.IsKeyJustPressed(ebiten.KeyG) {
		g.Settings.LargeRoadNetwork = !g.Settings.LargeRoadNetwork
	}

	// Hotkey R: reload custom map from custom_map.json
	if inpututil.IsKeyJustPressed(ebiten.KeyR) {
		_ = g.ReloadCustomMap()
	}

	// Arrow keys / A / D
	if inpututil.IsKeyJustPressed(ebiten.KeyLeft) || inpututil.IsKeyJustPressed(ebiten.KeyA) {
		g.SelectedMapTheme = previousTheme(g.SelectedMapTheme)
	} else if inpututil.IsKeyJustPressed(ebiten.KeyRight) || inpututil.IsKeyJustPressed(ebiten.KeyD) {
		g.SelectedMapTheme = g.SelectedMapTheme.Next()
	}

	// Enter or Space: Enter city!
	if inpututil.IsKeyJustPressed(ebiten.KeyEnter) || inpututil.IsKeyJustPressed(ebiten.KeySpace) {
		g.EnterCity(g.SelectedMapTheme)
		return
	}

	// Mouse click detection
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mPos := sim.Vec2{X: float64(mx), Y: float64(my)}

		if g.menuContinueBounds.Contains(mPos) {
			g.openSavePicker(true)
			return
		}
		if g.menuLargeGridBounds.Contains(mPos) {
			g.Settings.LargeRoadNetwork = !g.Settings.LargeRoadNetwork
			g.SelectedMapTheme = sim.MapThemeMetropolis
			return
		}
		if g.menuLangBounds.Contains(mPos) {
			newLang := locale.ToggleLanguage()
			g.Settings.Language = newLang
			return
		}

		if g.menuStartBounds.Contains(mPos) {
			g.EnterCity(g.SelectedMapTheme)
			return
		}

		for i, theme := range sim.SelectableMapThemes() {
			if g.menuCardBounds[i].Contains(mPos) {
				if g.SelectedMapTheme == theme {
					g.EnterCity(g.SelectedMapTheme)
					return
				}
				g.SelectedMapTheme = theme
				return
			}
		}
	}
}

func (g *Game) drawMainMenu(screen *ebiten.Image) {
	sw := float64(screen.Bounds().Dx())
	sh := float64(screen.Bounds().Dy())

	// 1. Deep slate background with ambient grid
	screen.Fill(sim.RGBA(15, 23, 42, 255))
	gridSpacing := 60.0
	for gx := 0.0; gx < sw; gx += gridSpacing {
		vector.StrokeLine(screen, float32(gx), 0, float32(gx), float32(sh), 1.0, sim.RGBA(30, 41, 59, 120), true)
	}
	for gy := 0.0; gy < sh; gy += gridSpacing {
		vector.StrokeLine(screen, 0, float32(gy), float32(sw), float32(gy), 1.0, sim.RGBA(30, 41, 59, 120), true)
	}

	// Language toggle button at top right
	langBtnW := 190.0
	langBtnH := 32.0
	langBtnX := sw - langBtnW - 24.0
	langBtnY := 18.0
	g.menuLangBounds = sim.Rect{X: langBtnX, Y: langBtnY, W: langBtnW, H: langBtnH}
	vector.DrawFilledRect(screen, float32(langBtnX), float32(langBtnY), float32(langBtnW), float32(langBtnH), sim.RGBA(30, 41, 59, 220), true)
	vector.StrokeRect(screen, float32(langBtnX), float32(langBtnY), float32(langBtnW), float32(langBtnH), 1.5, sim.RGBA(56, 189, 248, 255), true)
	textui.DrawText(screen, locale.T("🌐 语言: 简体中文 [L]", "🌐 Lang: English [L]"), langBtnX+18, langBtnY+8, 12, sim.RGBA(56, 189, 248, 255))

	// 2. Title and Header
	titleY := 36.0
	if menuLogoImg == nil && len(menuLogoBytes) > 0 {
		if raw, _, err := image.Decode(bytes.NewReader(menuLogoBytes)); err == nil {
			menuLogoImg = ebiten.NewImageFromImage(raw)
		}
	}

	titleText := locale.T("2D BUS SIMULATOR · 城市公交模拟器", "2D BUS SIMULATOR · Urban Bus Simulator")
	tw, _ := textui.MeasureText(titleText, 24)
	targetSize := 64.0
	gap := 16.0
	totalHeaderW := tw
	if menuLogoImg != nil {
		totalHeaderW = targetSize + gap + tw
	}
	headerStartX := (sw - totalHeaderW) / 2.0

	if menuLogoImg != nil {
		op := &ebiten.DrawImageOptions{}
		scale := targetSize / float64(menuLogoImg.Bounds().Dx())
		op.GeoM.Scale(scale, scale)
		op.GeoM.Translate(headerStartX, titleY-18)
		op.Filter = ebiten.FilterLinear // Ultra-crisp HD bilinear filtering for Retina displays
		screen.DrawImage(menuLogoImg, op)
		headerStartX += targetSize + gap
	}

	textui.DrawText(screen, titleText, headerStartX, titleY, 24, sim.RGBA(248, 250, 252, 255))

	subText := locale.T("请选择营运城市与地图线路 (4 座大型城市 · 街区双侧设站 · 1～6 条线路 · 支持玩家自定义地图及热重载)", "Select city & route (4 expanded cities · Stops on both street sides · 1–6 lines · Custom map & hot-reload supported)")
	subW, _ := textui.MeasureText(subText, 14)
	textui.DrawText(screen, subText, (sw-subW)/2.0, titleY+42, 14, sim.RGBA(148, 163, 184, 255))

	// 3. 6 Map Cards
	cardW := 192.0
	cardH := 380.0
	cardGap := 12.0
	totalW := 5*cardW + 4*cardGap
	startX := (sw - totalW) / 2.0
	startY := 115.0

	themes := []sim.MapTheme{
		sim.MapThemeMetropolis,
		sim.MapThemeCoastalBay,
		sim.MapThemeRiverValley,
		sim.MapThemeTwinCities,
		sim.MapThemeCustom,
	}

	accents := []color.RGBA{
		sim.RGBA(59, 130, 246, 255), // Metropolis: Royal Blue
		sim.RGBA(6, 182, 212, 255),  // Coastal Bay: Ocean Cyan
		sim.RGBA(16, 185, 129, 255), // River Valley: River Emerald
		sim.RGBA(168, 85, 247, 255), // Twin Cities: High-tech Violet
		sim.RGBA(245, 158, 11, 255), // Custom Map: Amber Gold
	}

	type mapCardInfo struct {
		tag      string
		name     string
		sub      string
		topo     string
		landmark string
		stops    string
	}

	metropolisTopo := locale.T("• 公交专用八车道主干路网", "• Bus-priority boulevard grid")
	if g.Settings.LargeRoadNetwork {
		metropolisTopo = locale.T("• 4x 超长大路网 (路口4倍间距)", "• 4x Large grid (4x block length)")
	}

	cardData := []mapCardInfo{
		{
			tag:      locale.T("【01 都会 CBD】", "【01 Metropolis】"),
			name:     locale.T("都会核心 (Metropolis)", "Metropolis CBD"),
			sub:      locale.T("经典都会 · 商务高层群", "Classic Skyline & CBD"),
			topo:     metropolisTopo,
			landmark: locale.T("• 运河双子桥与喷泉环岛", "• Canal bridges & fountain"),
			stops:    locale.T("• 大街区道路双侧密集设站", "• Dense two-sided bus stops"),
		},
		{
			tag:      locale.T("【02 海滨港湾】", "【02 Coastal Bay】"),
			name:     locale.T("海滨港湾 (Coastal Bay)", "Coastal Bay Port"),
			sub:      locale.T("海景风情 · 国际深水港", "Scenic coast & seaport"),
			topo:     locale.T("• 黄金海岸与高密港口路网", "• Ocean drive & freight grid"),
			landmark: locale.T("• 1200m 跨海悬索特大桥", "• 1200m Suspension bridge"),
			stops:    locale.T("• 大街区道路双侧密集设站", "• Dense two-sided bus stops"),
		},
		{
			tag:      locale.T("【03 溪谷江城】", "【03 River Valley】"),
			name:     locale.T("溪谷江城 (River Valley)", "River Valley City"),
			sub:      locale.T("天然大江 · 山城两岸立体拓扑", "River canyon & mountain city"),
			topo:     locale.T("• 跨江斜拉双桥与沿江干道", "• Dual cable-stayed bridges"),
			landmark: locale.T("• 滨江大道与观景高地", "• Riverside expressway"),
			stops:    locale.T("• 大街区道路双侧密集设站", "• Dense two-sided bus stops"),
		},
		{
			tag:      locale.T("【04 高新双城】", "【04 Twin Districts】"),
			name:     locale.T("高新双城 (Twin Districts)", "Twin Tech Districts"),
			sub:      locale.T("临空制造 · 硅谷软件双城融合", "Aerospace & software hub"),
			topo:     locale.T("• 东西双城干道与中央快速路", "• Inter-district corridors"),
			landmark: locale.T("• 跨区中央高架特大桥", "• Grade-separated overpass"),
			stops:    locale.T("• 大街区道路双侧密集设站", "• Dense two-sided bus stops"),
		},
		{
			tag:      locale.T("【05 自定义地图】", "【05 Custom Map】"),
			name:     locale.T("自定义地图 (Custom)", "Custom Map (JSON)"),
			sub:      locale.T("custom_map.json 动态载入", "Dynamic custom_map.json"),
			topo:     locale.T("• 自由扩展道路/站点/水系", "• User-defined roads & stops"),
			landmark: locale.T("• 支持随时按 [R] 热重载", "• Hot reload with [R] anytime"),
			stops:    locale.T("• 站点数量由玩家地图定义", "• Fully customizable stops"),
		},
	}

	for i, th := range themes {
		cx := startX + float64(i)*(cardW+cardGap)
		cy := startY
		isSelected := (g.SelectedMapTheme == th)
		if isSelected {
			cy -= 8.0 // Lift up
		}

		g.menuCardBounds[i] = sim.Rect{X: cx, Y: cy, W: cardW, H: cardH}
		accent := accents[i]
		info := cardData[i]

		// Card background
		bgCol := sim.RGBA(30, 41, 59, 230)
		borderW := float32(1.5)
		borderCol := sim.RGBA(71, 85, 105, 200)
		if isSelected {
			bgCol = sim.RGBA(30, 41, 59, 255)
			borderW = 3.0
			borderCol = accent
		}

		// Draw card shadow & card body
		vector.DrawFilledRect(screen, float32(cx+4), float32(cy+6), float32(cardW), float32(cardH), sim.RGBA(2, 6, 23, 160), true)
		vector.DrawFilledRect(screen, float32(cx), float32(cy), float32(cardW), float32(cardH), bgCol, true)
		vector.StrokeRect(screen, float32(cx), float32(cy), float32(cardW), float32(cardH), borderW, borderCol, true)

		// Top accent line
		vector.DrawFilledRect(screen, float32(cx), float32(cy), float32(cardW), 6, accent, true)

		// Tag & Titles
		textui.DrawText(screen, info.tag, cx+12, cy+22, 12, accent)
		textui.DrawText(screen, info.name, cx+12, cy+52, 12, sim.RGBA(248, 250, 252, 255))
		textui.DrawText(screen, info.sub, cx+12, cy+78, 10, sim.RGBA(148, 163, 184, 255))

		// Separator line
		vector.StrokeLine(screen, float32(cx+12), float32(cy+105), float32(cx+cardW-12), float32(cy+105), 1.0, sim.RGBA(71, 85, 105, 180), true)

		// Features
		textui.DrawText(screen, locale.T("【地理与路网特色】", "【Map Features】"), cx+12, cy+125, 11, accent)
		textui.DrawText(screen, info.topo, cx+12, cy+155, 10, sim.RGBA(226, 232, 240, 255))
		textui.DrawText(screen, info.landmark, cx+12, cy+185, 10, sim.RGBA(226, 232, 240, 255))
		textui.DrawText(screen, info.stops, cx+12, cy+215, 10, sim.RGBA(226, 232, 240, 255))

		if i == 0 {
			largeGridY := cy + cardH - 84
			largeGridW := cardW - 24
			largeGridH := 28.0
			g.menuLargeGridBounds = sim.Rect{X: cx + 12, Y: largeGridY, W: largeGridW, H: largeGridH}
			btnBg := sim.RGBA(15, 23, 42, 220)
			borderCol := sim.RGBA(100, 116, 139, 200)
			textColor := sim.RGBA(148, 163, 184, 255)
			btnText := locale.T("🛣️ 标准路网 1x [G]", "🛣️ Standard 1x [G]")
			if g.Settings.LargeRoadNetwork {
				btnBg = sim.RGBA(30, 58, 138, 240)
				borderCol = sim.RGBA(56, 189, 248, 255)
				textColor = sim.RGBA(56, 189, 248, 255)
				btnText = locale.T("🛣️ 大路网 4x [G]", "🛣️ Large Grid 4x [G]")
			}
			vector.DrawFilledRect(screen, float32(cx+12), float32(largeGridY), float32(largeGridW), float32(largeGridH), btnBg, true)
			vector.StrokeRect(screen, float32(cx+12), float32(largeGridY), float32(largeGridW), float32(largeGridH), 1.5, borderCol, true)
			textui.DrawText(screen, btnText, cx+22, largeGridY+6, 11, textColor)
		}

		// Button inside card
		btnBoxY := cy + cardH - 48
		btnBoxW := cardW - 24
		btnBoxH := 32.0
		if isSelected {
			vector.DrawFilledRect(screen, float32(cx+12), float32(btnBoxY), float32(btnBoxW), float32(btnBoxH), accent, true)
			textui.DrawText(screen, locale.T("★ 当前选定 ★", "★ Selected ★"), cx+32, btnBoxY+8, 12, color.White)
		} else {
			vector.DrawFilledRect(screen, float32(cx+12), float32(btnBoxY), float32(btnBoxW), float32(btnBoxH), sim.RGBA(15, 23, 42, 220), true)
			vector.StrokeRect(screen, float32(cx+12), float32(btnBoxY), float32(btnBoxW), float32(btnBoxH), 1.0, sim.RGBA(100, 116, 139, 200), true)
			textui.DrawText(screen, locale.Tf("点击选择 [%d]", "Select [%d]", i+1), cx+36, btnBoxY+8, 12, sim.RGBA(148, 163, 184, 255))
		}
	}

	// 4. Large Action Button: Start Playing
	btnW := 380.0
	btnH := 48.0
	btnX := (sw - btnW) / 2.0
	btnY := startY + cardH + 24.0
	g.menuStartBounds = sim.Rect{X: btnX, Y: btnY, W: btnW, H: btnH}

	vector.DrawFilledRect(screen, float32(btnX+3), float32(btnY+5), float32(btnW), float32(btnH), sim.RGBA(2, 6, 23, 180), true)
	vector.DrawFilledRect(screen, float32(btnX), float32(btnY), float32(btnW), float32(btnH), sim.RGBA(16, 185, 129, 255), true)
	vector.StrokeRect(screen, float32(btnX), float32(btnY), float32(btnW), float32(btnH), 2.0, sim.RGBA(167, 243, 208, 255), true)
	textui.DrawText(screen, locale.T("▶ 开始营运 / 进入所选城市 (回车 / 空格)", "▶ Start Operation / Enter City (Enter / Space)"), btnX+24, btnY+14, 15, color.White)

	// 5. Bottom Operation Tips
	continueY := btnY + btnH + 12
	g.menuContinueBounds = sim.Rect{X: btnX, Y: continueY, W: btnW, H: 40}
	vector.DrawFilledRect(screen, float32(btnX), float32(continueY), float32(btnW), 40, sim.RGBA(51, 65, 85, 255), true)
	label := locale.T("读取存档 [F9] · 10 个存档槽位", "Load game [F9] · 10 save slots")
	textui.DrawText(screen, label, btnX+20, continueY+12, 14, sim.RGBA(186, 230, 253, 255))
	if g.saveMessage != "" {
		textui.DrawText(screen, g.saveMessage, 24, sh-24, 12, sim.RGBA(251, 191, 36, 255))
	}
	bottomY := continueY + 54.0
	tips := locale.T("【操作提示】按数字键 [1-5/C] 或 [← / →] 切换城市 | 按 [G] 切换都会大路网 (4x) | 按 [L] 切换语言 | 按 [R] 重载自定义地图 | 驾驶途中点击右上角【主菜单】或按 ESC 退回更换", "【Controls】Keys [1-5/C] or [←/→] Select City | [G] Metropolis 4x Grid | [L] Toggle Language | [R] Hot Reload Map | Click [Menu] or ESC to Change")
	tipW, _ := textui.MeasureText(tips, 12)
	textui.DrawText(screen, tips, (sw-tipW)/2.0, bottomY, 12, sim.RGBA(148, 163, 184, 255))
}

func previousTheme(theme sim.MapTheme) sim.MapTheme {
	themes := sim.SelectableMapThemes()
	for i, t := range themes {
		if t == theme {
			return themes[(i+len(themes)-1)%len(themes)]
		}
	}
	return themes[0]
}
