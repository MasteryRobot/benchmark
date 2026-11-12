# GPU 双臂灵巧操作 Benchmark

项目位于 [benchmark/](benchmark/)，包含 72 个 Sharpa 风格场景原型、10 个自定义任务和 1 个双手卷毛巾入口，共 83 项。

自定义任务：拿勺子、拿刀具、拿手机、手机横竖屏转换、手机翻面、电池调整姿态、打火机调整姿态、拿口红、棉签旋转、拿剪刀。

历史 GPU 重建环境为 RTX 5090、Isaac Sim 6.0.1-rc.7。83 个场景均有五路 RTX RGB 与几何深度，配套 85 页图册。原始 GPU 记录日期为 2026-10-31。

## 下载和运行

代码、任务配置、83 个 GPU 场景入口和验证记录保存在 Git 中。模型资产、相机数据和 PDF 从 [Release](https://github.com/MasteryRobot/benchmark/releases/tag/gpu-benchmark-20261031) 下载。

在仓库根目录执行：

```bash
gh release download gpu-benchmark-20261031 --repo MasteryRobot/benchmark --dir downloads
(cd downloads && sha256sum -c SHA256SUMS)
tar -xzf downloads/benchmark-runtime-assets.tar.gz
tar -xzf downloads/benchmark-five-camera-data.tar.gz
mkdir -p benchmark/output/pdf
cp downloads/benchmark_gpu_five_camera_views.pdf benchmark/output/pdf/
cd benchmark
python -B benchmark_cli.py list
# 使用已配置 Isaac Sim 的 Python
ISAAC_PYTHON=/path/to/isaac/python ./run_gpu_capture.sh
# 单场景 GPU 物理检查
/path/to/isaac/python gpu_physics_smoke.py
```

也可从 Release 页面手动下载。两个压缩包均在仓库根目录解压，保留 benchmark/ 相对路径。运行场景只需模型资产包；查看既有图像时下载相机数据包。资产包包含机器人、YCB 模型、共享 workcell 和主场景；排除重复的上游 tgz 下载包、Python 环境与临时缓存。

在 Isaac Sim 打开 `benchmark/output/gpu_scenes/<任务ID>.usda` 可使用显式 GPU 动力学/宽相位配置。详细的构建、依赖与任务接口见 [原项目说明](benchmark/README.md)。

## 验证范围

- [GPU 配置检查](benchmark/output/gpu_entry_validation.json)：83 个入口及各自 5 路相机。
- [源场景与图像哈希](benchmark/output/manifest.json)：83 个场景、415 张相机 RGB。
- [GPU 物理记录](benchmark/output/gpu_physics_smoke.json)：拿手机场景，54 自由度、240 步、cuda:0。
- [图册检查](benchmark/output/pdf_validation.json)：85 页，415 组 RGB/深度。

图册为初始状态静态渲染；深度为几何射线距离。现有证据不代表全部任务执行成功、全部场景动力学验证或毛巾接触行为验证。Sharpa 场景为本地参考原型，官方 demo 对应关系仍需逐项复核。

原项目 README 保留历史开发记录；其中旧环境阻塞状态，以及未随 GPU 工作区迁移的历史采集数据链接，应按历史记录理解。本次 GPU 证据集中在 benchmark/output/。上传整理仅调整两个启动脚本的本机 Python 默认路径，未重新运行 GPU 仿真。

## 资产来源

[YCB 来源和署名](benchmark/assets/external/ycb/ATTRIBUTION.md)随仓库与资产包保留。第三方资产遵循各自原始许可；发布不改变其权属，不另行授予统一开源许可。
