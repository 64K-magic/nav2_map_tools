# PGM Map Editor (QT)

轻量的 PGM 地图编辑器，用于加载 2D PGM 地图并手动擦除黑点/噪声（将占用点改为 free），保存修改后的 PGM。

主要功能：
- 打开 PGM 文件（支持常见灰度图格式，P5/P2，并支持 maxval>255 的情形）
- 用画笔擦除黑点（默认将像素设为 free 254）
- 可调画笔大小
- 撤销（每笔为一个快照）
- 放大/缩小、平移视图（`Move` 切换）

Python 版本（快速试用）
- 依赖：Python 层依赖放在 `requirements.txt`（PySide6/Pillow/numpy）
	- 安装：
		```bash
		python3 -m pip install -r requirements.txt
		```
	- 运行（如果你的系统缺少 Qt XCB 原生依赖，请先参考下方系统依赖安装）：
		```bash
		python3 main.py
		```

C++ (Qt) 版本（更快、更独立，可编译为本地二进制）

1) 系统依赖（Debian/Ubuntu 示例）

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config \
		qtbase5-dev qtbase5-dev-tools qttools5-dev-tools
```

如果你使用 Fedora / CentOS：

```bash
sudo dnf install -y @development-tools cmake qt5-qtbase-devel qt5-qttools-devel
```

macOS (Homebrew)：

```bash
brew install cmake qt@5
export CMAKE_PREFIX_PATH="$(brew --prefix qt@5)"
```

说明：部分发行版使用 Qt6，若你想使用 Qt6，请安装对应的开发包（`qt6-base-dev` 等），并在构建时通过 `-DCMAKE_PREFIX_PATH` 指定 Qt 安装前缀。

2) 构建（在项目根目录）

```bash
# 创建构建目录并生成 Makefile
cmake -S . -B build -G "Unix Makefiles"
# 或者如果 Qt 安装在非标准路径（示例）：
# cmake -S . -B build -G "Unix Makefiles" -DCMAKE_PREFIX_PATH=/path/to/Qt/5.15.2/gcc_64

# 构建
cmake --build build -j$(nproc)

# 运行
./build/pgm_map_editor
```

3) 常见问题
- 如果 CMake 找不到 Qt，请把 Qt 的 cmake 配置目录加入 `CMAKE_PREFIX_PATH`（例如 `/opt/Qt/5.15.2/gcc_64` 或 Homebrew 的 `$(brew --prefix qt@5)`）。
- 如果 GUI 报错无法加载插件（例如 xcb），通常是缺失系统级库，请安装 `libxcb1 libx11-xcb1 libxkbcommon-x11-0 libxcb-cursor0` 等（见 `system_requirements.txt`）。

使用说明：
- 点击 `Open` 打开 PGM 地图
- 鼠标左键按住擦除（将像素设为 free）
- 滑动 `Brush Size` 调整画笔半径
- 点击 `Save` 保存文件（默认保存为二进制 P5）
- 滚轮缩放视图，使用工具栏 `Move` 切换平移模式

如果在打开地图时仍然失败，请把文件的前 200 字节（`head -c 200 your_map.pgm | xxd`）发给我，我会根据真实文件头继续改进解析器以兼容特殊变体（例如 BOM、额外元数据或非标准 header）。
