# 庆园阁视觉调试平台

该平台是 QYG 算法的只读旁路观察系统。C++ 视觉进程继续独立运行，通过 `/dev/shm` 发布 JPEG、状态快照和曲线；Flask 进程只读取这些数据，网页故障不会参与瞄准、云台或发弹决策。

## 快速开始

首次安装 Python 依赖：

```bash
python3 -m pip install -r web/requirements.txt
```

分别启动视觉算法和 Web 服务：

```bash
./build/QYG_sentry_debug
./scripts/start_web.sh
```

默认监听 `0.0.0.0:9000`。笔记本与车载电脑接入同一局域网后，访问：

```text
http://车载电脑IP:9000
```

端口被占用时可改用：

```bash
./scripts/start_web.sh --port 9100
```

该服务没有登录和 TLS，只允许在可信局域网使用，禁止端口映射到公网。

## 算法运行选项

`QYG_sentry_debug` 默认发布 Web 数据且不创建 OpenCV 窗口：

```bash
./build/QYG_sentry_debug [configs/sentry.yaml]
./build/QYG_sentry_debug --display [configs/sentry.yaml]
./build/QYG_sentry_debug -d [configs/sentry.yaml]
./build/QYG_sentry_debug --no-web [configs/sentry.yaml]
```

`--no-web` 完全跳过共享内存、JPEG 编码和调试图渲染，用于与 Web 开启状态进行同输入性能对照。

Web 发布异常由独立线程限频写入 `/dev/shm/qyg_web_debug_warnings.log`，不使用 Planner 控制线程的同步日志 sink。

## 无相机端到端验证

构建并运行合成发布器：

```bash
cmake -S . -B build
cmake --build build --target qyg_web_debug_synthetic -j2
./build/qyg_web_debug_synthetic
```

随后启动 `./scripts/start_web.sh`。合成发布器会持续生成移动装甲板、目标状态和曲线；按 `Ctrl+C` 停止。也可以限定运行时间：

```bash
./build/qyg_web_debug_synthetic --seconds 10
```

## 页面与接口

- 左上：带 Detector、EKF 和 Planner 标记的 MJPEG 实时画面。
- 右上：系统、检测、追踪、目标、规划、云台和耗时状态树。
- 左下：可勾选的多曲线总图，默认显示 FPS、帧延迟、Detector 等待和 Yaw/Pitch 误差。
- 右下：单字段独立曲线；窄屏自动切换为纵向排列。
- `VIDEO / DATA / LOG / PRODUCER` 独立显示在线状态；源断开时清空旧值并自动重连。

HTTP 接口：

| 接口 | 内容 |
| --- | --- |
| `/` | 调试主页 |
| `/video` | MJPEG 视频流 |
| `/data?max_points=200` | 10–600点曲线数据 |
| `/log` | 当前结构化状态 |
| `/health` | 四类数据源在线状态 |

数据超过2秒未更新时判定离线。`/data`、`/log` 和 `/video` 对缺失、损坏或过期数据返回 `503`，不会把最后一帧冒充实时数据。

## 数据协议

```text
/dev/shm/qyg_frame
/dev/shm/qyg_data.json
/dev/shm/qyg_log.json
```

图像帧头为32字节小端结构：

```text
magic[4] = QYGF
version: uint16 = 1
header_size: uint16 = 32
sequence: uint64（奇数=写入中，偶数=稳定帧）
monotonic_ns: uint64
jpeg_size: uint32
capacity: uint32
```

读端在复制 JPEG 前后两次读取帧头；只有序列相同且为偶数时才接受。视频最高约60 Hz，JSON和600点环形曲线为20 Hz。JSON 使用临时文件加原子 rename，当前 `schema_version` 为1；无效目标字段使用 `null`。

## 测试与性能验收

```bash
cmake --build build --target web_debug_test QYG_sentry_debug -j2
./build/web_debug_test
python3 -m pip install -r web/requirements-dev.txt
PYTHONPATH=. pytest -q web/tests
```

性能验收使用同一机器、配置和输入，分别运行默认模式与 `--no-web`，取稳定区间的 FPS 中位数。Web 开启后的下降不得超过5%；超出时先降低 JPEG 质量或发布频率，不允许修改 Detector、Tracker 或 Planner 行为来换取指标。

若 CMake 在 Miniforge 环境选择 fmt v12，而系统 spdlog 1.9 编译失败，可显式使用 Ubuntu 系统 fmt：

```bash
cmake -S . -B build -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt
```

## 品牌资产

“庆园阁战队”品牌资产位于 `web/static/img/`：

- `qyg-team-logo-original.jpg`：用户提供的原始文件，逐字节保留。
- `qyg-team-logo-header.webp`：页眉裁切缩放版本。
- `qyg-team-favicon.png`：由原图上方标志裁切生成的 favicon。

不得加入其他战队的名称或 Logo，也不得用生成图替换原始资产。

本功能参考 Awakening 的四宫格调试布局及共享内存传输思路。第三方版权见仓库根目录 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。
