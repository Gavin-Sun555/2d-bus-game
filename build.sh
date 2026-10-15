#!/usr/bin/env bash
# ==============================================================================
# 2D Bus Simulator - Build & Run Script
# ==============================================================================

set -e

APP_NAME="busgame"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

print_banner() {
    echo -e "${CYAN}"
    echo "=========================================================="
    echo "   🚍 2D Bus Simulator & Route Planner - Build Tool      "
    echo "=========================================================="
    echo -e "${NC}"
}

cmd_build() {
    print_banner
    echo -e "${BLUE}🔨 正在编译当前平台可执行文件 [${APP_NAME}]...${NC}"
    
    go build -trimpath -ldflags="-s -w" -o "${APP_NAME}" .
    chmod +x "${APP_NAME}"
    
    SIZE=$(ls -lh "${APP_NAME}" | awk '{print $5}')
    echo -e "${GREEN}✅ 编译成功！${NC}"
    echo -e "   📦 二进制文件: ${GREEN}${ROOT_DIR}/${APP_NAME}${NC} (体积: ${SIZE})"
    echo -e "   🚀 直接运行方式: ${YELLOW}./${APP_NAME}${NC}"
}

cmd_run() {
    cmd_build
    echo ""
    echo -e "${GREEN}🎮 正在启动游戏...${NC}"
    exec "./${APP_NAME}"
}

cmd_test() {
    print_banner
    echo -e "${BLUE}🧪 正在运行基础快速测试 (Fast Dev Mode)...${NC}"
    go test -short ./...
}

cmd_test_qualify() {
    print_banner
    echo -e "${BLUE}🏆 正在运行全地图多轮 Release Qualification 验收测试...${NC}"
    go test -v -run TestReleaseQualification ./...
}

cmd_test_all() {
    print_banner
    echo -e "${BLUE}🧪 正在运行全部单元测试与集成测试...${NC}"
    go test -v ./...
}

cmd_clean() {
    print_banner
    echo -e "${YELLOW}🧹 清理编译输出文件...${NC}"
    rm -f "${APP_NAME}" "${APP_NAME}-*" *.exe *.test *.out
    echo -e "${GREEN}✅ 清理完成！${NC}"
}

cmd_cross() {
    print_banner
    echo -e "${BLUE}🌐 正在执行多平台跨平台编译 (Cross-compile)...${NC}"
    mkdir -p dist

    # macOS (ARM64 Apple Silicon)
    echo -e "📦 编译 macOS (Apple Silicon arm64)..."
    GOOS=darwin GOARCH=arm64 go build -trimpath -ldflags="-s -w" -o "dist/${APP_NAME}-darwin-arm64" .
    
    # macOS (Intel x86_64)
    echo -e "📦 编译 macOS (Intel amd64)..."
    GOOS=darwin GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o "dist/${APP_NAME}-darwin-amd64" .

    # Windows (x86_64)
    echo -e "📦 编译 Windows (x86_64 exe)..."
    GOOS=windows GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o "dist/${APP_NAME}-windows-amd64.exe" .

    # Linux (x86_64)
    echo -e "📦 编译 Linux (x86_64)..."
    GOOS=linux GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o "dist/${APP_NAME}-linux-amd64" .

    echo ""
    echo -e "${GREEN}✅ 跨平台编译完成！产物已存入 dist/ 目录：${NC}"
    ls -lh dist/
}

BUNDLE_NAME="2D Bus Simulator.app"

cmd_app() {
    print_banner
    echo -e "${BLUE}🍎 正在构建原生 macOS 应用程序包 [${BUNDLE_NAME}]...${NC}"
    cmd_build
    
    mkdir -p "${BUNDLE_NAME}/Contents/MacOS"
    mkdir -p "${BUNDLE_NAME}/Contents/Resources"
    
    cp "${APP_NAME}" "${BUNDLE_NAME}/Contents/MacOS/${APP_NAME}"
    cp "assets/AppIcon.icns" "${BUNDLE_NAME}/Contents/Resources/AppIcon.icns"
    
    cat << 'EOF' > "${BUNDLE_NAME}/Contents/Info.plist"
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleInfoDictionaryVersion</key>
    <string>6.0</string>
    <key>CFBundleName</key>
    <string>2D Bus Simulator</string>
    <key>CFBundleDisplayName</key>
    <string>2D Bus Simulator</string>
    <key>CFBundleIdentifier</key>
    <string>com.gavinsun.2dbussimulator</string>
    <key>CFBundleVersion</key>
    <string>1.0.0</string>
    <key>CFBundleShortVersionString</key>
    <string>1.0.0</string>
    <key>CFBundleExecutable</key>
    <string>busgame</string>
    <key>CFBundleIconFile</key>
    <string>AppIcon</string>
    <key>NSHighResolutionCapable</key>
    <true/>
</dict>
</plist>
EOF

    touch "${BUNDLE_NAME}"
    if command -v swift >/dev/null 2>&1; then
        swift -e "
import Cocoa
if let img = NSImage(contentsOfFile: \"assets/logo_rounded_1024.png\") {
    NSWorkspace.shared.setIcon(img, forFile: \"${BUNDLE_NAME}\", options: [])
    NSWorkspace.shared.setIcon(img, forFile: \"${APP_NAME}\", options: [])
}
" >/dev/null 2>&1 || true
    fi

    echo -e "${GREEN}✅ macOS 原生应用包打包成功！${NC}"
    echo -e "   📦 应用包路径: ${GREEN}${ROOT_DIR}/${BUNDLE_NAME}${NC}"
    echo -e "   🚀 运行方式: 双击访达中的应用，或运行 ${YELLOW}open \"${BUNDLE_NAME}\"${NC}"
}

cmd_open() {
    if [ ! -d "${BUNDLE_NAME}" ]; then
        cmd_app
    fi
    echo -e "${GREEN}🎮 正在启动 macOS 原生应用包 [${BUNDLE_NAME}]...${NC}"
    open "${BUNDLE_NAME}"
}

case "$1" in
    run)
        cmd_run
        ;;
    app)
        cmd_app
        ;;
    open)
        cmd_open
        ;;
    clean)
        cmd_clean
        ;;
    test)
        cmd_test
        ;;
    test-qualify)
        cmd_test_qualify
        ;;
    test-all)
        cmd_test_all
        ;;
    cross)
        cmd_cross
        ;;
    help|--help|-h)
        print_banner
        echo "使用方法: ./build.sh [选项]"
        echo ""
        echo "选项说明:"
        echo "  (无参数)      编译生成当前系统可执行文件 ./${APP_NAME}"
        echo "  run           编译并立即在终端启动运行游戏"
        echo "  app           构建带专属高清图标的 macOS 原生应用程序包 (${BUNDLE_NAME})"
        echo "  open          构建并以 macOS 原生 App 方式启动 (在 Dock 显示专属图标)"
        echo "  test          运行基础快速测试 (平时开发秒级通过)"
        echo "  test-qualify  运行全地图全场景发布验收测试 (发布到 GitHub 前必跑)"
        echo "  test-all      运行全部单元与集成测试"
        echo "  cross         跨平台编译 (macOS ARM64/Intel, Windows, Linux) 至 dist/"
        echo "  clean         清理生成的二进制产物"
        echo "  help          显示本帮助信息"
        ;;
    build|*)
        cmd_build
        ;;
esac
