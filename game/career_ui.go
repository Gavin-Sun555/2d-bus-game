package game

import (
	"fmt"
	"image/color"
	"math"
	"strings"
	"unicode/utf8"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	ebiten "github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type careerButton struct {
	rect   sim.Rect
	action string
	id     int
	value  string
}
type careerOffice struct {
	open                  bool
	tab, selected, scroll int
	buttons               []careerButton
	message               string
	confirmSell           int
	naming                bool
	nameInput             string
}

func panelBox(screen *ebiten.Image, r sim.Rect, bg color.RGBA) {
	vector.DrawFilledRect(screen, float32(r.X), float32(r.Y), float32(r.W), float32(r.H), bg, true)
}
func careerText(screen *ebiten.Image, s string, x, y, size float64, c color.RGBA) {
	textui.DrawText(screen, s, x, y, size, c)
}

var careerFG = sim.RGBA(226, 232, 240, 255)
var careerMuted = sim.RGBA(148, 163, 184, 255)
var careerAccent = sim.RGBA(94, 234, 212, 255)

func (g *Game) officeButton(screen *ebiten.Image, r sim.Rect, label, action string, id int, value string) {
	panelBox(screen, r, sim.RGBA(30, 58, 70, 255))
	vector.StrokeRect(screen, float32(r.X), float32(r.Y), float32(r.W), float32(r.H), 1, careerAccent, true)
	careerText(screen, label, r.X+9, r.Y+8, 12, careerFG)
	g.careerOffice.buttons = append(g.careerOffice.buttons, careerButton{r, action, id, value})
}
func (g *Game) updateModeSelection() {
	if inpututil.IsKeyJustPressed(ebiten.KeyL) {
		g.Settings.Language = locale.ToggleLanguage()
	}
	if inpututil.IsKeyJustPressed(ebiten.Key1) {
		g.SelectMode(ModeFast)
		return
	}
	if inpututil.IsKeyJustPressed(ebiten.Key2) {
		g.SelectMode(ModeCareer)
		return
	}
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		for i, r := range g.modeButtons {
			if r.Contains(sim.Vec2{X: float64(x), Y: float64(y)}) {
				g.SelectMode(PlayMode(i))
				return
			}
		}
	}
}

// Measure each localized string so all mode text shares its screen/card axis.
func centerModeText(screen *ebiten.Image, s string, centerX, y, size float64, c color.RGBA) {
	width, _ := textui.MeasureText(s, size)
	careerText(screen, s, centerX-width/2, y, size, c)
}

func (g *Game) drawModeSelection(screen *ebiten.Image) {
	screen.Fill(sim.RGBA(10, 18, 32, 255))
	w, h := float64(screen.Bounds().Dx()), float64(screen.Bounds().Dy())
	centerModeText(screen, "2D BUS SIMULATOR", w/2, h*.18, 32, careerFG)
	centerModeText(screen, locale.T("选择你的游戏方式", "Choose your play mode"), w/2, h*.18+55, 20, careerAccent)
	cw := math.Min(450, (w-100)/2)
	x := (w - 2*cw - 24) / 2
	y := h * .34
	for i := 0; i < 2; i++ {
		r := sim.Rect{X: x + float64(i)*(cw+24), Y: y, W: cw, H: 260}
		g.modeButtons[i] = r
		panelBox(screen, r, sim.RGBA(22, 36, 55, 255))
		vector.StrokeRect(screen, float32(r.X), float32(r.Y), float32(r.W), float32(r.H), 2, careerAccent, true)
		title := []string{"FAST · 自由驾驶 [1]", "CAREER · 生涯经营 [2]"}[i]
		if locale.CurrentLanguage == locale.LanguageEN {
			title = []string{"FAST · Free driving [1]", "CAREER · Build your company [2]"}[i]
		}
		centerModeText(screen, title, r.X+r.W/2, r.Y+27, 23, careerFG)
		lines := []string{locale.T("保留现有玩法，所有城市直接开放", "Existing gameplay with all cities unlocked"), locale.T("自由设置车型与 AI 车队", "Configure bus models and AI fleets freely"), locale.T("独立的 10 个存档槽；兼容旧存档", "10 separate slots; existing saves stay here")}
		if i == 1 {
			lines = []string{locale.T("从一辆普通公交和 20 万资金开始", "Start with one standard bus and 200,000"), locale.T("购车、运营、维修，建立自己的车队", "Buy, operate and maintain your own fleet"), locale.T("解锁城市与成就；独立 10 个存档槽", "Unlock cities and achievements; 10 own slots")}
		}
		for j, line := range lines {
			centerModeText(screen, line, r.X+r.W/2, r.Y+92+float64(j)*34, 14, careerMuted)
		}
		centerModeText(screen, locale.T("点击进入 →", "Click to enter →"), r.X+r.W/2, r.Y+215, 16, careerAccent)
	}
	centerModeText(screen, locale.T("[L] 简体中文 / English · 两种模式的资金、车辆和存档相互独立", "[L] Language · Each mode has its own money, vehicles and saves"), w/2, y+300, 14, careerMuted)
	if g.saveMessage != "" {
		centerModeText(screen, fitSavePickerText(g.saveMessage, 13, w-40), w/2, h-30, 13, sim.RGBA(251, 191, 36, 255))
	}
}
func (g *Game) updateCareerOffice() bool {
	if g.savePicker.open || g.stopNameEditor != nil {
		return false
	}
	clickedOffice := false
	if g.isCareer() && !g.careerOffice.open && inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		clickedOffice = g.careerOpenBounds.Contains(sim.Vec2{X: float64(x), Y: float64(y)})
	}
	if (inpututil.IsKeyJustPressed(ebiten.KeyF2) || clickedOffice) && g.isCareer() {
		g.careerOffice.open = !g.careerOffice.open
		g.syncCareerCondition()
		return true
	}
	if !g.careerOffice.open {
		return false
	}
	o := &g.careerOffice
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		if o.naming {
			o.naming = false
		} else {
			o.open = false
		}
		return true
	}
	if o.naming {
		o.nameInput += string(ebiten.AppendInputChars(nil))
		r := []rune(o.nameInput)
		if len(r) > 24 {
			o.nameInput = string(r[:24])
		}
		if inpututil.IsKeyJustPressed(ebiten.KeyBackspace) && len(r) > 0 {
			o.nameInput = string(r[:len(r)-1])
		}
		if inpututil.IsKeyJustPressed(ebiten.KeyEnter) {
			name := strings.TrimSpace(o.nameInput)
			if name != "" && utf8.RuneCountInString(name) <= 24 {
				g.Career.DriverName = name
				g.careerChanged = true
			}
			o.naming = false
		}
		return true
	}
	_, wheel := ebiten.Wheel()
	if o.tab == 1 {
		o.scroll = max(0, min(max(0, len(g.Career.Vehicles)-7), o.scroll-int(wheel)))
	}
	if g.State == StateMainMenu && inpututil.IsKeyJustPressed(ebiten.KeyF5) {
		o.open = false
		g.openSavePicker(false)
		return true
	}
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		x, y := ebiten.CursorPosition()
		p := sim.Vec2{X: float64(x), Y: float64(y)}
		for _, b := range o.buttons {
			if b.rect.Contains(p) {
				g.performCareerAction(b)
				break
			}
		}
	}
	return true
}
func (g *Game) performCareerAction(b careerButton) {
	o := &g.careerOffice
	c := g.Career
	var err error
	switch b.action {
	case "close":
		o.open = false
	case "tab":
		o.tab = b.id
		o.confirmSell = 0
	case "name":
		o.naming = true
		o.nameInput = c.DriverName
	case "buy":
		var id int
		id, err = c.Buy(b.value)
		if err == nil {
			o.selected = id
			o.message = locale.T("购车成功：车辆已入库，请在车队页排班", "Purchased: assign the bus from the fleet tab")
		}
	case "select":
		o.selected = b.id
		o.confirmSell = 0
	case "sell":
		if err = g.careerFleetEditable(); err == nil {
			o.confirmSell = o.selected
			o.message = locale.T("确认以显示的回收价卖出？点击确认卖出", "Sell at the displayed resale value? Click Confirm sale")
		}
	case "confirm":
		if err = g.careerFleetEditable(); err == nil && o.confirmSell != 0 {
			_, err = c.Sell(o.confirmSell)
			if err == nil {
				o.selected = c.PlayerVehicleID
				o.message = locale.T("车辆已卖出", "Vehicle sold")
			}
			o.confirmSell = 0
		}
	case "cancel":
		o.confirmSell = 0
	case "repair":
		err = g.repairCareerVehicle(o.selected)
		if err == nil {
			o.message = locale.T("维修完成，费用已扣除", "Repaired; fee deducted")
		}
	case "drive":
		err = g.driveCareerVehicle(o.selected)
		if err == nil {
			o.message = locale.T("已选择玩家车辆，下次进城生效", "Player bus selected for next city entry")
		}
	case "line":
		v := c.Vehicle(o.selected)
		if v != nil {
			next := v.Line + 1
			if next > 5 {
				next = -1
			}
			err = g.assignCareerVehicle(v.ID, next)
			if err == nil {
				o.message = locale.T("排班已更新，下次进城生效", "Dispatch updated for next city entry")
			}
		}
	case "claim":
		_, err = c.Claim(b.value)
		if err == nil {
			o.message = locale.T("成就奖励已到账", "Achievement reward received")
		}
	}
	if err != nil {
		o.message = err.Error()
	} else {
		switch b.action {
		case "buy", "confirm", "repair", "drive", "line", "claim":
			g.careerChanged = true
		}
	}
	g.Cash = int(math.Floor(c.Balance))
}
func (g *Game) drawCareerBadge(screen *ebiten.Image) {
	if !g.isCareer() {
		return
	}
	r := sim.Rect{X: float64(screen.Bounds().Dx()) - 310, Y: float64(screen.Bounds().Dy()) - 65, W: 295, H: 28}
	g.careerOpenBounds = r
	panelBox(screen, r, sim.RGBA(15, 23, 42, 240))
	careerText(screen, fmt.Sprintf("Career · %d %s · [F2] %s", len(g.Career.Vehicles), locale.T("辆车", "buses"), locale.T("经营车库", "Garage")), r.X+10, r.Y+6, 12, careerAccent)
}
func (g *Game) drawCareerOffice(screen *ebiten.Image) {
	if !g.isCareer() || !g.careerOffice.open {
		return
	}
	o := &g.careerOffice
	c := g.Career
	o.buttons = o.buttons[:0]
	w, h := float64(screen.Bounds().Dx()), float64(screen.Bounds().Dy())
	pw := math.Min(1150, w-30)
	ph := math.Min(660, h-24)
	x, y := (w-pw)/2, (h-ph)/2
	panelBox(screen, sim.Rect{W: w, H: h}, sim.RGBA(0, 0, 0, 210))
	panelBox(screen, sim.Rect{X: x, Y: y, W: pw, H: ph}, sim.RGBA(15, 23, 42, 255))
	careerText(screen, locale.T("CAREER · 经营车库", "CAREER · Company & garage"), x+22, y+20, 23, careerFG)
	careerText(screen, fmt.Sprintf("%s · %s %.2f · %d %s", c.DriverName, locale.T("资金 ¥", "Balance ¥"), c.Balance, len(c.Vehicles), locale.T("辆车", "buses")), x+22, y+55, 15, careerAccent)
	g.officeButton(screen, sim.Rect{X: x + pw - 240, Y: y + 18, W: 110, H: 32}, locale.T("角色改名", "Rename"), "name", 0, "")
	g.officeButton(screen, sim.Rect{X: x + pw - 115, Y: y + 18, W: 92, H: 32}, locale.T("关闭 [F2]", "Close [F2]"), "close", 0, "")
	for i, label := range []string{locale.T("购车市场", "Dealership"), locale.T("我的车队", "My fleet"), locale.T("成就与城市", "Achievements")} {
		if i == o.tab {
			label = "● " + label
		}
		g.officeButton(screen, sim.Rect{X: x + 22 + float64(i)*180, Y: y + 87, W: 166, H: 32}, label, "tab", i, "")
	}
	contentY := y + 136
	switch o.tab {
	case 0:
		for _, col := range []struct {
			x      float64
			zh, en string
		}{{24, "车型 / 外观", "Vehicle / livery"}, {340, "价格", "Price"}, {460, "极速", "km/h"}, {525, "加速度", "Acceleration"}, {650, "运营/km", "Cost/km"}, {760, "维修/1%", "Repair/1%"}, {875, "折旧/km", "Deprec/km"}} {
			careerText(screen, locale.T(col.zh, col.en), x+col.x, contentY, 13, careerMuted)
		}
		for i, s := range sim.VehicleCatalog() {
			yy := contentY + 28 + float64(i)*40
			if i%2 == 0 {
				panelBox(screen, sim.Rect{X: x + 18, Y: yy - 4, W: pw - 36, H: 38}, sim.RGBA(22, 34, 51, 255))
			}
			drawCatalogSwatch(screen, s, x+26, yy+7)
			name := s.Name
			if locale.CurrentLanguage == locale.LanguageEN {
				name = s.EnglishName
			}
			careerText(screen, name, x+108, yy, 13, careerFG)
			kind := s.Model.Name()
			if locale.CurrentLanguage == locale.LanguageEN {
				kind = s.Model.EnglishName()
			}
			careerText(screen, fmt.Sprintf("%s · %d %s", kind, s.Model.Capacity(), locale.T("人", "pax")), x+108, yy+18, 10, careerMuted)
			careerText(screen, fmt.Sprintf("¥%d", s.Price), x+340, yy+5, 13, careerFG)
			careerText(screen, fmt.Sprintf("%.0f", s.TopSpeedKPH), x+460, yy+5, 13, careerFG)
			careerText(screen, fmt.Sprintf("%.2f m/s2", s.Acceleration*sim.MetersPerPixel), x+525, yy+5, 13, careerFG)
			careerText(screen, fmt.Sprintf("¥%.1f", s.CostPerKM), x+650, yy+5, 13, careerFG)
			careerText(screen, fmt.Sprintf("¥%.0f", s.RepairPerPoint), x+760, yy+5, 13, careerFG)
			careerText(screen, fmt.Sprintf("¥%.1f", s.DepreciationPerKM), x+875, yy+5, 13, careerFG)
			buyLabel := locale.T("购买", "Buy")
			if c.Balance < float64(s.Price) {
				buyLabel = locale.T("资金不足", "No funds")
			}
			g.officeButton(screen, sim.Rect{X: x + pw - 105, Y: yy - 2, W: 78, H: 30}, buyLabel, "buy", 0, s.ID)
		}
		careerText(screen, locale.T("赠送星途 S40；新车进入车库。按车队页排班后，重新进入城市开始运营。", "A free Xingtu S40 starts your career. Assign purchases, then enter a city to operate."), x+24, contentY+404, 13, careerMuted)
	case 1:
		g.drawCareerFleet(screen, x, contentY, pw)
	case 2:
		careerText(screen, fmt.Sprintf("%s %.2f km · %s %d · %s ¥%.2f · %s ¥%.2f", locale.T("总里程", "Distance"), c.TotalKM, locale.T("运送", "Passengers"), c.Passengers, locale.T("运营支出", "Operating costs"), c.OperatingCost, locale.T("维修支出", "Repairs"), c.RepairCost), x+24, contentY, 13, careerFG)
		for i, a := range sim.CareerAchievements() {
			yy := contentY + 30 + float64(i)*37
			name := a.Name
			if locale.CurrentLanguage == locale.LanguageEN {
				name = a.EnglishName
			}
			careerText(screen, name, x+26, yy+6, 14, careerFG)
			careerText(screen, fmt.Sprintf("+¥%d", a.Reward), x+500, yy+6, 14, careerAccent)
			state := locale.T("未达成", "In progress")
			if c.Achievements[a.ID] {
				state = locale.T("已领取", "Claimed")
			} else if c.AchievementReady(a.ID) {
				g.officeButton(screen, sim.Rect{X: x + 720, Y: yy, W: 130, H: 30}, locale.T("领取奖励", "Claim reward"), "claim", 0, a.ID)
				state = ""
			}
			careerText(screen, state, x+725, yy+7, 13, careerMuted)
		}
		careerText(screen, locale.T("城市解锁：都会 1 辆 → 海滨 3 辆 → 溪谷 5 辆 → 双城 8 辆", "Cities: Metropolis 1 → Coastal 3 → River 5 → Twin 8 buses"), x+24, contentY+351, 15, careerAccent)
		careerText(screen, locale.T("按历史最大持有量解锁，卖车不会重新锁城。自定义地图留在 Fast 模式。", "Unlocks use your largest fleet; selling keeps cities unlocked. Custom maps belong to Fast."), x+24, contentY+383, 13, careerMuted)
	}
	if o.naming {
		panelBox(screen, sim.Rect{X: x + 160, Y: y + 150, W: pw - 320, H: 140}, sim.RGBA(30, 58, 70, 255))
		careerText(screen, locale.T("角色名称（24 字内），Enter 确认，Esc 取消", "Driver name (24 characters), Enter to save, Esc to cancel"), x+180, y+170, 15, careerFG)
		careerText(screen, o.nameInput+"|", x+180, y+220, 22, careerAccent)
	}
	if o.message != "" {
		careerText(screen, o.message, x+22, y+ph-55, 13, sim.RGBA(251, 191, 36, 255))
	}
	footer := locale.T("[F2] 经营车库 · 打开时暂停 · [F5] 保存 Career 存档", "[F2] Garage · Simulation paused while open · [F5] Save Career")
	if g.State != StateMainMenu {
		footer = locale.T("[F2] 经营车库 · 打开时暂停 · 请返回主界面保存或读取生涯存档", "[F2] Garage · Simulation paused while open · Return to main menu to save/load")
	}
	careerText(screen, footer, x+22, y+ph-27, 12, careerMuted)
}
func drawCatalogSwatch(screen *ebiten.Image, s sim.VehicleSpec, x, y float64) {
	c := sim.RGBA(s.Color[0], s.Color[1], s.Color[2], 255)
	length := 64.
	if s.Model == sim.BusArticulated {
		length = 74
	}
	panelBox(screen, sim.Rect{X: x, Y: y, W: length, H: 14}, c)
	panelBox(screen, sim.Rect{X: x + 5, Y: y + 3, W: length - 13, H: 8}, sim.RGBA(15, 23, 42, 255))
	if s.Style == 1 {
		panelBox(screen, sim.Rect{X: x + 15, Y: y + 6, W: 28, H: 3}, careerFG)
	}
	if s.Style == 2 {
		panelBox(screen, sim.Rect{X: x + length - 23, Y: y + 1, W: 5, H: 12}, careerFG)
	}
	if s.Model == sim.BusArticulated {
		panelBox(screen, sim.Rect{X: x + 31, Y: y, W: 8, H: 14}, sim.RGBA(80, 90, 100, 255))
	}
	if s.Model == sim.BusDoubleDecker {
		panelBox(screen, sim.Rect{X: x + 6, Y: y + 5, W: length - 16, H: 3}, c)
	}
}
func (g *Game) drawCareerFleet(screen *ebiten.Image, x, y, pw float64) {
	c := g.Career
	o := &g.careerOffice
	careerText(screen, locale.T("选择车辆；运营排班和玩家车切换在城市选择页办理，下一次进城生效。", "Select a bus. Dispatch and player swaps are managed on the city menu for the next entry."), x+24, y, 13, careerMuted)
	for _, col := range []struct {
		x      float64
		zh, en string
	}{{24, "车辆", "Vehicle"}, {340, "状态", "Duty"}, {460, "里程", "Mileage"}, {590, "车况", "Condition"}, {720, "回收价格", "Resale"}, {900, "全额维修", "Repair"}} {
		careerText(screen, locale.T(col.zh, col.en), x+col.x, y+30, 13, careerMuted)
	}
	o.scroll = min(o.scroll, max(0, len(c.Vehicles)-7))
	for i := o.scroll; i < min(len(c.Vehicles), o.scroll+7); i++ {
		v := c.Vehicles[i]
		s, _ := sim.FindVehicleSpec(v.SpecID)
		yy := y + 57 + float64(i-o.scroll)*35
		r := sim.Rect{X: x + 20, Y: yy, W: pw - 40, H: 32}
		bg := sim.RGBA(22, 34, 51, 255)
		if o.selected == v.ID {
			bg = sim.RGBA(30, 75, 80, 255)
		}
		panelBox(screen, r, bg)
		o.buttons = append(o.buttons, careerButton{rect: r, action: "select", id: v.ID})
		name := s.Name
		if locale.CurrentLanguage == locale.LanguageEN {
			name = s.EnglishName
		}
		duty := locale.T("车库", "Depot")
		if v.ID == c.PlayerVehicleID {
			duty = locale.T("玩家驾驶", "Player")
		} else if v.Line >= 0 {
			duty = fmt.Sprintf("Line %d", v.Line+1)
		}
		careerText(screen, fmt.Sprintf("#%d %s", v.ID, name), x+28, yy+7, 13, careerFG)
		careerText(screen, duty, x+340, yy+7, 13, careerAccent)
		careerText(screen, fmt.Sprintf("%.2f km", v.KM), x+460, yy+7, 13, careerFG)
		careerText(screen, fmt.Sprintf("%.1f%%", v.Condition), x+590, yy+7, 13, careerFG)
		careerText(screen, fmt.Sprintf("¥%d", s.ResaleValue(v.KM)), x+720, yy+7, 13, careerFG)
		careerText(screen, fmt.Sprintf("¥%d", s.RepairQuote(v.Condition)), x+900, yy+7, 13, careerFG)
	}
	if c.Vehicle(o.selected) == nil {
		o.selected = c.PlayerVehicleID
	}
	yy := y + 320
	for i, b := range []struct{ label, action string }{{locale.T("驾驶此车", "Drive this"), "drive"}, {locale.T("排班：下条线/车库", "Next line / depot"), "line"}, {locale.T("维修所选车辆", "Repair selected"), "repair"}, {locale.T("卖出所选车辆", "Sell selected"), "sell"}} {
		g.officeButton(screen, sim.Rect{X: x + 24 + float64(i)*240, Y: yy, W: 220, H: 34}, b.label, b.action, 0, "")
	}
	careerText(screen, locale.T("卖价 = 新车售价 × 0.8 - 总里程 × 车型折旧/km（最低 0）；玩家车须先换车。", "Resale = new price × 0.8 - mileage × depreciation/km (minimum 0). Swap player bus before selling."), x+24, y+366, 13, careerMuted)
	if o.confirmSell != 0 {
		g.officeButton(screen, sim.Rect{X: x + 24, Y: y + 394, W: 190, H: 32}, locale.T("确认卖出", "Confirm sale"), "confirm", 0, "")
		g.officeButton(screen, sim.Rect{X: x + 230, Y: y + 394, W: 110, H: 32}, locale.T("取消", "Cancel"), "cancel", 0, "")
	}
}
