package game

import (
	"fmt"
	"math"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"

	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

func (g *Game) updateAutopilotHotkeys() {
	// 0. AI Autopilot Toggle (P key) & Berserk Mode Toggle (B key or Shift+P)
	if inpututil.IsKeyJustPressed(ebiten.KeyP) {
		if ebiten.IsKeyPressed(ebiten.KeyShift) {
			// Shift+P: Toggle Berserk Mode
			g.Settings.AutopilotBerserk = !g.Settings.AutopilotBerserk
			if !g.Settings.AutopilotEnabled {
				g.Settings.AutopilotEnabled = true
				g.AutopilotDriver.Reset()
			}
			if g.Settings.AutopilotBerserk {
				g.HUD.AddToast(locale.T("【⚡ AI自动驾驶·狂暴模式已开启】直行极速狂飙，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed boosted, multi-lane overtaking enabled, obeying all rules!"), true)
			} else {
				g.HUD.AddToast(locale.T("【🤖 AI自动驾驶·标准模式已恢复】平稳巡航行驶 (按 B 键切换狂暴模式)", "【🤖 AI Standard Mode ON】Cruising safely (Press B for Berserk)"), true)
			}
		} else {
			// Normal P key: Toggle Autopilot ON/OFF
			g.Settings.AutopilotEnabled = !g.Settings.AutopilotEnabled
			if g.Settings.AutopilotEnabled {
				g.AutopilotDriver.Reset()
				if g.Settings.AutopilotBerserk {
					g.HUD.AddToast(locale.T("【⚡ AI自动驾驶·狂暴模式已开启】直行极速狂飙，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed boosted, multi-lane overtaking enabled, obeying all rules!"), true)
				} else {
					g.HUD.AddToast(locale.T("【🤖 AI自动驾驶已开启】AI已接管车辆，遵循交规循迹靠站 (按 B 开启狂暴，按 P 接管)", "【🤖 AI Autopilot ON】AI controls vehicle, serving stops (Press B for Berserk, P to exit)"), true)
				}
			} else {
				g.HUD.AddToast(locale.T("【👤 已恢复手动驾驶】您已接管公交车控制权 (按 P 键可重新开启 AI)", "【👤 Manual Mode Restored】You have taken control (Press P to enable AI)"), false)
			}
		}
	} else if inpututil.IsKeyJustPressed(ebiten.KeyB) {
		if ebiten.IsKeyPressed(ebiten.KeyShift) || ebiten.IsKeyPressed(ebiten.KeyShiftLeft) || ebiten.IsKeyPressed(ebiten.KeyShiftRight) {
			// Shift+B: Cycle Max Speed Limit (60 -> 80 -> 100 -> 120 -> 60)
			newSpd := g.Settings.CycleBerserkMaxSpeed()
			g.AutopilotDriver.BerserkMaxSpeed = g.Settings.GetBerserkCruiseSpeed()
			if g.Settings.AutopilotBerserk {
				g.Bus.MaxSpeedFwd = math.Max(g.Settings.GetBusMaxSpeed(), g.AutopilotDriver.GetBerserkCruiseSpeed()+20.0)
			} else {
				g.Bus.MaxSpeedFwd = g.Settings.GetBusMaxSpeed()
			}
			g.HUD.AddToast(fmt.Sprintf(locale.T("【⚡ 公交最高限速已切换】%d km/h (手动/狂暴模式均生效)", "【⚡ Top Speed Limit Set】%d km/h (Active in Manual & Berserk)"), newSpd), true)
		} else {
			// Dedicated B key: Toggle Berserk Mode
			g.Settings.AutopilotBerserk = !g.Settings.AutopilotBerserk
			if !g.Settings.AutopilotEnabled {
				g.Settings.AutopilotEnabled = true
				g.AutopilotDriver.Reset()
			}
			if g.Settings.AutopilotBerserk {
				g.HUD.AddToast(fmt.Sprintf(locale.T("【⚡ AI自动驾驶·狂暴模式已开启】直行最高时速 %d km/h (Shift+B可切速)，双车道变道超车，严守交规安全避障！", "【⚡ AI Berserk Mode ON】Top speed %d km/h (Shift+B to cycle speed), multi-lane overtaking enabled!"), g.Settings.AutopilotBerserkMaxSpeed), true)
			} else {
				g.HUD.AddToast(locale.T("【🤖 AI自动驾驶·标准模式已恢复】平稳巡航行驶 (按 B 键切换狂暴模式)", "【🤖 AI Standard Mode ON】Cruising safely (Press B for Berserk)"), true)
			}
		}
	}

}

func (g *Game) updateDriverHUD() bool {
	// 4. Perspective toggle hotkey (V key)
	if inpututil.IsKeyJustPressed(ebiten.KeyV) {
		newMode := g.Camera.ToggleMode()
		g.Settings.CameraViewMode = newMode
		switch newMode {
		case sim.CameraModeHeadingUp:
			g.HUD.AddToast("已切换视角: 【车头朝上 (跟随视角)】 - 车头始终朝上，视野随转向旋转", true)
		case sim.CameraModeSouthUp:
			g.HUD.AddToast("已切换视角: 【固定正南 (倒置视角)】 - 地图固定正南朝上", true)
		default:
			g.HUD.AddToast("已切换视角: 【固定正北 (标准视角)】 - 地图固定正北朝上", true)
		}
	}

	// 5. Mouse click on HUD buttons (Mode badge, Map theme badge, Perspective badge, Gear selectors & Autopilot)
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		mx, my := ebiten.CursorPosition()
		mouseScreen := sim.Vec2{X: float64(mx), Y: float64(my)}
		if g.HUD.HandleModeBtnClick(mouseScreen) {
			g.TogglePlanMode()
			return true
		} else if toggled := g.HUD.HandleMenuBtnClick(mouseScreen); toggled {
			g.State = StateMainMenu
			return true
		} else if toggled, notice := g.HUD.HandleCameraBtnClick(mouseScreen, g.Camera); toggled {
			g.Settings.CameraViewMode = g.Camera.Mode
			g.HUD.AddToast(notice, true)
		} else if shifted, notice := g.HUD.HandleGearClick(mouseScreen, g.Bus); shifted {
			g.HUD.AddToast(notice, false)
		} else if toggled, notice := g.HUD.HandleAutopilotClick(mouseScreen, g.Settings); toggled {
			if g.Settings.AutopilotEnabled {
				g.AutopilotDriver.Reset()
			}
			g.HUD.AddToast(notice, g.Settings.AutopilotEnabled)
		} else if notice != "" {
			g.HUD.AddWarningToast(notice)
		}
	}

	return false
}
