package ui

import (
	"fmt"
	"sort"

	"github.com/Gavin-Sun555/2d-bus-game/internal/locale"
	"github.com/Gavin-Sun555/2d-bus-game/internal/sim"
	"github.com/Gavin-Sun555/2d-bus-game/internal/textui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
)

type OperationsPanel struct {
	IsOpen                       bool
	Tab, Scroll, SelectedVehicle int
	closeBounds, vehicleBounds   sim.Rect
	tabBounds, modeBounds        [3]sim.Rect
	fieldBounds                  [7][2]sim.Rect
}

func NewOperationsPanel() *OperationsPanel { return &OperationsPanel{SelectedVehicle: -1} }

func (p *OperationsPanel) Update(fare *sim.FarePolicy, o *sim.TransitOperations) {
	if inpututil.IsKeyJustPressed(ebiten.KeyEscape) {
		p.IsOpen = false
		return
	}
	for i, key := range []ebiten.Key{ebiten.Key1, ebiten.Key2, ebiten.Key3} {
		if inpututil.IsKeyJustPressed(key) {
			p.Tab = i
			p.Scroll = 0
		}
	}
	_, wheel := ebiten.Wheel()
	if p.Tab != 0 {
		p.Scroll = max(0, p.Scroll-int(wheel)*3)
	}
	if !inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		return
	}
	mx, my := ebiten.CursorPosition()
	pos := sim.Vec2{X: float64(mx), Y: float64(my)}
	if p.closeBounds.Contains(pos) {
		p.IsOpen = false
		return
	}
	for i, r := range p.tabBounds {
		if r.Contains(pos) {
			p.Tab = i
			p.Scroll = 0
			return
		}
	}
	if p.Tab != 0 && p.vehicleBounds.Contains(pos) {
		ids := operationsVehicleIDs(o)
		options := append([]int{-1}, ids...)
		for i, id := range options {
			if id == p.SelectedVehicle {
				p.SelectedVehicle = options[(i+1)%len(options)]
				p.Scroll = 0
				return
			}
		}
		p.SelectedVehicle = -1
		return
	}
	if p.Tab != 0 {
		return
	}
	for i, r := range p.modeBounds {
		if r.Contains(pos) {
			fare.Mode = sim.FareMode(i)
			return
		}
	}
	for field, buttons := range p.fieldBounds {
		for direction, r := range buttons {
			if r.W > 0 && r.Contains(pos) {
				delta := -1
				if direction == 1 {
					delta = 1
				}
				AdjustFare(fare, field, delta)
				return
			}
		}
	}
}

func AdjustFare(p *sim.FarePolicy, field, delta int) {
	switch field {
	case 0:
		p.Fixed += delta
	case 1:
		p.Base += delta
	case 2:
		p.IncludedStops += delta
	case 3:
		p.StopsPerYuan += delta
	case 4:
		p.IncludedKM += delta
	case 5:
		p.PerKM += delta
	case 6:
		p.Cap += delta
	}
	*p = p.Normalize()
}

func operationsVehicleIDs(o *sim.TransitOperations) []int {
	ids := []int{0}
	if o == nil {
		return ids
	}
	for id := range o.Vehicles {
		if id != 0 {
			ids = append(ids, id)
		}
	}
	sort.Ints(ids)
	return ids
}
func operationsVehicleName(id int) string {
	if id < 0 {
		return locale.T("全部车辆", "All vehicles")
	}
	if id == 0 {
		return locale.T("玩家公交", "Player bus")
	}
	return fmt.Sprintf("AI #%d", id)
}
func operationsText(screen *ebiten.Image, s string, x, y float64, size float64) {
	textui.DrawText(screen, s, x, y, size, sim.RGBA(226, 232, 240, 255))
}
func operationsButton(screen *ebiten.Image, r sim.Rect, s string, selected bool) {
	c := sim.RGBA(51, 65, 85, 255)
	if selected {
		c = sim.RGBA(2, 132, 199, 255)
	}
	vector.DrawFilledRect(screen, float32(r.X), float32(r.Y), float32(r.W), float32(r.H), c, true)
	operationsText(screen, s, r.X+12, r.Y+9, 13)
}
func (p *OperationsPanel) Draw(screen *ebiten.Image, fare sim.FarePolicy, o *sim.TransitOperations, w *sim.World) {
	if !p.IsOpen {
		return
	}
	sw, sh := float64(screen.Bounds().Dx()), float64(screen.Bounds().Dy())
	width, height := min(960., sw-32), min(640., sh-24)
	x, y := (sw-width)/2, (sh-height)/2
	vector.DrawFilledRect(screen, 0, 0, float32(sw), float32(sh), sim.RGBA(0, 0, 0, 185), false)
	vector.DrawFilledRect(screen, float32(x), float32(y), float32(width), float32(height), sim.RGBA(15, 23, 42, 255), true)
	vector.StrokeRect(screen, float32(x), float32(y), float32(width), float32(height), 2, sim.RGBA(56, 189, 248, 255), true)
	operationsText(screen, locale.T("票价与运营统计", "Fares & Operations"), x+24, y+20, 20)
	p.closeBounds = sim.Rect{X: x + width - 105, Y: y + 14, W: 85, H: 34}
	operationsButton(screen, p.closeBounds, locale.T("关闭", "Close"), false)
	titles := []string{locale.T("1 票价设置", "1 Fares"), locale.T("2 站间距离 / 客流", "2 Route / Passengers"), locale.T("3 车辆运营", "3 Vehicles")}
	for i, title := range titles {
		p.tabBounds[i] = sim.Rect{X: x + 24 + float64(i)*(width-48)/3, Y: y + 64, W: (width-48)/3 - 12, H: 34}
		operationsButton(screen, p.tabBounds[i], title, p.Tab == i)
	}
	if p.Tab != 0 {
		p.vehicleBounds = sim.Rect{X: x + width - 258, Y: y + 104, W: 234, H: 34}
		operationsButton(screen, p.vehicleBounds, operationsVehicleName(p.SelectedVehicle)+"  >", false)
	}
	operationsText(screen, locale.T("[O / ESC] 关闭 · 查看面板时暂停运营", "[O / ESC] Close · Simulation paused while open"), x+24, y+height-30, 12)
	if p.Tab == 0 {
		p.drawFares(screen, fare, x, y, width)
		return
	}
	if o == nil {
		return
	}
	if p.Tab == 1 {
		p.drawStations(screen, o, w, x, y+40, width, height-40)
	} else {
		p.drawVehicles(screen, o, x, y+40, width, height-40)
	}
}

func (p *OperationsPanel) drawFares(screen *ebiten.Image, f sim.FarePolicy, x, y, width float64) {
	f = f.Normalize()
	p.fieldBounds = [7][2]sim.Rect{}
	modes := []string{locale.T("固定票价", "Flat fare"), locale.T("按站数计价", "By stops"), locale.T("按里程计价", "By distance")}
	for i, label := range modes {
		p.modeBounds[i] = sim.Rect{X: x + 24 + float64(i)*230, Y: y + 120, W: 216, H: 36}
		operationsButton(screen, p.modeBounds[i], label, int(f.Mode) == i)
	}
	fields := []int{0}
	if f.Mode == sim.FareByStops {
		fields = []int{1, 2, 3, 6}
	}
	if f.Mode == sim.FareByDistance {
		fields = []int{1, 4, 5, 6}
	}
	labels := []string{locale.T("固定票价（元）", "Flat fare"), locale.T("起步价（元）", "Base fare"), locale.T("起步价包含站数", "Stops included"), locale.T("之后每几站加 1 元", "Additional stops per +1"), locale.T("起步价包含里程（公里）", "Kilometers included"), locale.T("之后每公里票价（元）", "Price per extra kilometer"), locale.T("封顶票价（元，必须高于起步价）", "Cap (must exceed base)")}
	values := []int{f.Fixed, f.Base, f.IncludedStops, f.StopsPerYuan, f.IncludedKM, f.PerKM, f.Cap}
	for row, field := range fields {
		ry := y + 184 + float64(row)*54
		operationsText(screen, labels[field], x+36, ry+9, 15)
		for direction, label := range []string{"-", "+"} {
			bx := x + width - 264 + float64(direction)*164
			p.fieldBounds[field][direction] = sim.Rect{X: bx, Y: ry, W: 44, H: 34}
			operationsButton(screen, p.fieldBounds[field][direction], label, false)
		}
		operationsText(screen, fmt.Sprint(values[field]), x+width-178, ry+7, 18)
	}
	notes := []string{locale.T("固定票价 0～10 元，0 元为免费乘车。", "Flat fare: 0–10; 0 means free travel."), locale.T("超出起步站数后，每进入一个新的加价站数组，加收 1 元。", "After included stops, each started group adds 1."), locale.T("仅对超出包含里程的部分计价，总票价四舍五入到整数。", "Only extra kilometers are charged; the total is rounded to an integer.")}
	operationsText(screen, notes[f.Mode], x+36, y+440, 14)
	operationsText(screen, locale.T("起步价 0～8 元；封顶不超过 10 元，且始终严格高于起步价。", "Base: 0–8. Cap: at most 10, always strictly above the base."), x+36, y+474, 13)
	operationsText(screen, locale.T("按乘客起终点的规划道路里程计价，上车时收费；调整只对之后上车的乘客生效。", "Charged at boarding using the planned journey. Edits apply to future boardings."), x+36, y+508, 13)
}

func (p *OperationsPanel) drawStations(screen *ebiten.Image, o *sim.TransitOperations, w *sim.World, x, y, width, height float64) {
	distances := o.Distances
	if v := o.Vehicles[p.SelectedVehicle]; p.SelectedVehicle >= 0 && v != nil && v.Route != nil {
		distances = o.DistancesFor(v.Route)
	}

	operationsText(screen, fmt.Sprintf(locale.T("当前线路：%d 段 · 全程 %.3f km · 收入见车辆运营页", "Current route: %d legs · %.3f km · Revenue on Vehicles tab"), len(distances.Legs), distances.TotalKM), x+24, y+118, 14)
	operationsText(screen, locale.T("起点 → 下一站", "Station → Next station"), x+24, y+160, 13)
	operationsText(screen, locale.T("道路里程 km", "Road km"), x+width-338, y+160, 13)
	operationsText(screen, locale.T("本站上车", "Boarded here"), x+width-216, y+160, 13)
	operationsText(screen, locale.T("本站下车", "Alighted here"), x+width-108, y+160, 13)
	rows := max(1, int((height-232)/30))
	p.Scroll = min(p.Scroll, max(0, len(distances.Legs)-rows))
	stationName := func(id int) string {
		if w != nil && id >= 0 && id < len(w.Stops) {
			r := []rune(w.Stops[id].CleanName())
			if len(r) > 20 {
				return string(r[:19]) + "…"
			}
			return string(r)
		}
		return fmt.Sprint(id)
	}
	activities := o.Stations
	if p.SelectedVehicle >= 0 {
		activities = nil
		if v := o.Vehicles[p.SelectedVehicle]; v != nil {
			activities = v.Stations
		}
	}
	for row := 0; row < rows && p.Scroll+row < len(distances.Legs); row++ {
		i := p.Scroll + row
		leg := distances.Legs[i]
		ry := y + 194 + float64(row)*30
		activity := activities[leg.From]
		if activity == nil {
			activity = &sim.StationActivity{}
		}
		operationsText(screen, fmt.Sprintf("%d. %s → %s", i+1, stationName(leg.From), stationName(leg.To)), x+24, ry, 12)
		operationsText(screen, fmt.Sprintf("%.3f", leg.KM), x+width-338, ry, 13)
		operationsText(screen, fmt.Sprint(activity.Boarded), x+width-216, ry, 13)
		operationsText(screen, fmt.Sprint(activity.Alighted), x+width-108, ry, 13)
	}
	if len(distances.Legs) > rows {
		operationsText(screen, locale.T("滚轮滚动查看其余站点", "Scroll for more stations"), x+24, y+height-58, 12)
	}
}

func (p *OperationsPanel) drawVehicles(screen *ebiten.Image, o *sim.TransitOperations, x, y, width, height float64) {
	var boarded, alighted int
	var completedKM, completedSeconds float64
	for _, v := range o.Vehicles {
		boarded += v.Boarded
		alighted += v.Alighted
		completedKM += v.CompletedKM
		completedSeconds += v.CompletedSeconds
	}
	summary := fmt.Sprintf(locale.T("全部车辆累计：收入 ¥%d · 上车 %d 人 · 下车 %d 人", "All vehicles: revenue ¥%d · Boarded %d · Alighted %d"), o.Revenue, boarded, alighted)
	operationsText(screen, summary, x+24, y+118, 15)
	average := locale.T("全线平均速度：完成首圈后显示", "Network average: available after a full lap")
	if completedSeconds > 0 {
		average = fmt.Sprintf(locale.T("全线平均速度：%.1f km/h（按已完成圈次加权）", "Network average: %.1f km/h (completed laps, time weighted)"), completedKM/completedSeconds*3600)
	}
	operationsText(screen, average, x+24, y+150, 13)
	var fleetKM, fleetSeconds float64
	for id, v := range o.Vehicles {
		if id > 0 {
			fleetKM += v.CompletedKM
			fleetSeconds += v.CompletedSeconds
		}
	}
	fleetAverage := locale.T("AI 车队均速：待完成首圈", "AI fleet average: awaiting full lap")
	if fleetSeconds > 0 {
		fleetAverage = fmt.Sprintf(locale.T("AI 车队均速：%.1f km/h", "AI fleet average: %.1f km/h"), fleetKM/fleetSeconds*3600)
	}
	operationsText(screen, fleetAverage, x+24, y+174, 12)
	cols := []float64{24, 174, 282, 406, 518, 684}
	headers := []string{locale.T("车辆", "Vehicle"), locale.T("收入（元）", "Revenue"), locale.T("上 / 下车", "Board / Alight"), locale.T("完成圈数", "Laps"), locale.T("平均速度 km/h", "Average km/h"), locale.T("运营里程 km", "Service km")}
	for i, h := range headers {
		operationsText(screen, h, x+cols[i], y+198, 13)
	}
	ids := operationsVehicleIDs(o)
	if p.SelectedVehicle >= 0 {
		ids = []int{p.SelectedVehicle}
	}
	rows := max(1, int((height-330)/32))
	p.Scroll = min(p.Scroll, max(0, len(ids)-rows))
	row := 0
	for _, id := range ids[p.Scroll:] {
		if row >= rows {
			break
		}
		v := o.Vehicles[id]
		if v == nil {
			v = &sim.VehicleOperations{ID: id}
		}
		speed := locale.T("待完成首圈", "Awaiting full lap")
		if value, ok := v.AverageKPH(); ok {
			speed = fmt.Sprintf("%.1f", value)
		}
		name := operationsVehicleName(id)
		if v.LineName != "" {
			name += " · " + v.LineName
		}
		fields := []string{name, fmt.Sprint(v.Revenue), fmt.Sprintf("%d / %d", v.Boarded, v.Alighted), fmt.Sprint(v.CompletedLaps), speed, fmt.Sprintf("%.3f", v.KM)}
		for i, value := range fields {
			operationsText(screen, value, x+cols[i], y+236+float64(row)*32, 13)
		}
		row++
	}
	operationsText(screen, locale.T("均速计入等灯、堵车、靠站；暂停时间不计。中途重置打断的圈次不纳入均速。", "Averages include lights, traffic and dwell; pauses and interrupted laps are excluded."), x+24, y+height-86, 12)
	operationsText(screen, locale.T("换地图或重新规划线路后重新统计；收入仅统计车费，不包含罚款。", "Statistics restart on map or route changes. Revenue is fares, excluding fines."), x+24, y+height-58, 12)
}
