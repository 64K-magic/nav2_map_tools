# control_center_web

RTK 禁行区 **前端页面包**：在 XYZ 瓦片地图上绘制线 / 矩形 / 不规则多边形 / 圆，通过 `control_center_api` 转换坐标系并写入 SQLite。

## 目录

```
web/
├── index.html
├── css/style.css
└── js/
    ├── api.js           # HTTP 客户端
    ├── app.js           # 主界面
    └── draw/
        ├── shapes.js    # 图形模型 / Leaflet 图层
        └── tools.js     # 绘制交互
```

由 `control_center_api` 在 `/` 托管静态资源（安装后从 `share/control_center_web/web` 读取）。

## 使用

1. 启动 API（需 `--ros`，默认 `/fromLL`）：

```bash
cd ~/gps_filter_ws/src/control_center_api
source /opt/ros/humble/setup.bash
python3 scripts/run_api.py --ros
```

2. 打开 `http://localhost:8088/`
3. 填写 Origin Lat/Lon/Yaw、MapName
4. 选择工具绘制；多边形左键加点，双击 / Enter / 右键结束
5. **保存到 DB**（经 `/fromLL` 转 map 米制，写入与 Nav2 `sql_db_path` 同一库）

## 交互对应 Qt GUI

| 工具 | 操作 |
|------|------|
| 选 | 点击选中；拖动移动；拖顶点缩放；上方圆点旋转；Delete 删除 |
| 线 / 矩 / 圆 | 两点完成（十字光标） |
| 多 | 多点 + 结束手势（十字光标） |
