# ==============================================================================
# 2D Bus Simulator - Makefile
# ==============================================================================

APP_NAME := busgame

BUNDLE_NAME := 2D Bus Simulator.app

.PHONY: all build run dev app open test test-qualify test-all cross clean help

all: build

## build: 编译当前平台的独立可执行文件 (带体积优化)
build:
	@echo "🔨 正在编译 $(APP_NAME)..."
	go build -trimpath -ldflags="-s -w" -o $(APP_NAME) .
	@chmod +x $(APP_NAME)
	@echo "✅ 编译成功: ./$(APP_NAME) ($$(ls -lh $(APP_NAME) | awk '{print $$5}'))"

## app: 构建带专属高清图标的 macOS 原生应用程序包
app:
	@./build.sh app

## open: 以 macOS 原生应用程序形式启动
open:
	@./build.sh open

## run: 编译并直接启动可执行文件
run: build
	@echo "🎮 正在启动游戏..."
	./$(APP_NAME)

## dev: 免编译直接通过 go run 启动
dev:
	go run .

## test: 运行基础快速测试 (平时开发模式，秒级通过)
test:
	@echo "🧪 正在运行基础快速测试套件 (Fast Dev Mode)..."
	go test -short ./...

## test-qualify: 运行发布前全地图全场景验收测试 (发布到 GitHub 前必跑)
test-qualify:
	@echo "🏆 正在运行全地图多轮 Release Qualification 验收测试..."
	go test -v -run TestReleaseQualification ./...

## test-all: 运行包含基础测试与全量验收测试的所有用例
test-all:
	@echo "🧪 正在运行全部测试用例..."
	go test -v ./...

## cross: 跨平台交叉编译 (macOS ARM64/Intel, Windows, Linux)
cross:
	@mkdir -p dist
	@echo "📦 编译 macOS (Apple Silicon arm64)..."
	GOOS=darwin GOARCH=arm64 go build -trimpath -ldflags="-s -w" -o dist/$(APP_NAME)-darwin-arm64 .
	@echo "📦 编译 macOS (Intel amd64)..."
	GOOS=darwin GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o dist/$(APP_NAME)-darwin-amd64 .
	@echo "📦 编译 Windows (x86_64 exe)..."
	GOOS=windows GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o dist/$(APP_NAME)-windows-amd64.exe .
	@echo "📦 编译 Linux (x86_64)..."
	GOOS=linux GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o dist/$(APP_NAME)-linux-amd64 .
	@echo "✅ 跨平台产物已生成至 dist/ 目录"

## clean: 清理二进制及测试产物
clean:
	@rm -rf $(APP_NAME) $(APP_NAME)-* *.exe *.test *.out dist/
	@echo "🧹 清理完成"

## help: 显示帮助信息
help:
	@echo "2D Bus Simulator - 可用命令:"
	@echo "  make build        - 编译可执行文件 ./$(APP_NAME)"
	@echo "  make app          - 构建 macOS 原生应用程序包 ($(BUNDLE_NAME))"
	@echo "  make open         - 构建并以原生 App 启动 (Dock 显现高清图标)"
	@echo "  make run          - 编译并在终端直接运行可执行文件"
	@echo "  make dev          - 直接通过 go run . 启动"
	@echo "  make test         - 运行基础快速测试 (日常开发秒级通过)"
	@echo "  make test-qualify - 运行全地图全场景发布验收测试 (发布到 GitHub 前必跑)"
	@echo "  make test-all     - 运行全量测试套件"
	@echo "  make cross        - 跨平台编译至 dist/ 目录"
	@echo "  make clean        - 清理编译产物"
