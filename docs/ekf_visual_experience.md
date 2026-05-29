# EKF 自瞄可视化体验（Rerun 优先）

这个体验包使用 `assets/demo/demo.avi` 和 `assets/demo/demo.txt` 离线跑一遍自瞄链路，并输出给 Rerun 使用的数据（JSONL + 可选图片）。

仓库里仍然保留 `ekf_visual_experience`（ROS2 版本，给 RViz2/Foxglove 用），但链路更长、更容易踩坑；建议先把 Rerun 跑通。

## 1. 构建

```bash
cmake -S . -B build
cmake --build build --target ekf_visual_experience_rerun -j$(nproc)
```

`ekf_visual_experience_rerun` 不依赖 ROS2。

## 2. 生成体验数据

真实自瞄回放需要同时存在：

- `assets/demo/demo.avi`
- `assets/demo/demo.txt`

```bash
./build/ekf_visual_experience_rerun \
  --config-path=configs/demo.yaml \
  --output-dir=logs/ekf_visual_experience \
  --image-stride=1 \
  assets/demo/demo
```

如果当前机器缺少 `demo.avi`，可以先用模拟轨迹体验 Rerun：

```bash
./build/ekf_visual_experience_rerun \
  --synthetic \
  --end-index=300 \
  --delay-ms=30 \
  --config-path=configs/demo.yaml \
  --output-dir=logs/ekf_visual_experience \
  assets/demo/demo
```

无显示环境但有真实 demo 视频时，去掉 `--synthetic` 即可。

输出内容：

- `logs/ekf_visual_experience/frames.jsonl`
- `logs/ekf_visual_experience/images/frame_*.jpg`
## 3. Rerun

Rerun 依赖建议放在仓库本地虚拟环境：

```bash
python3 -m venv .venv-rerun
.venv-rerun/bin/pip install -r scripts/rerun_ekf_requirements.txt
```

打开回放：

```bash
.venv-rerun/bin/python scripts/rerun_ekf_experience.py logs/ekf_visual_experience
```

Rerun 中可以同时看：

- `image/reprojection`：带重投影点的图像
- `world/armor_observation`：观测点
- `world/ekf_target`：EKF 目标点
- `world/aim_point`：瞄准点
- `scalar/*`：yaw、residual、NIS/NEES 等曲线

## 附：ROS2 版本（可选）

如果你的机器 ROS2 依赖齐全，可以构建 `ekf_visual_experience`（它会发布 `/ekf_visual/*` 话题给 RViz2/Foxglove 用）。
