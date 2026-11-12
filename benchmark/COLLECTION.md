# 三项代表任务：成功轨迹与 LeRobot 数据

## 最新进展（2026-10-31）

已取得一条基础任务 `03_pick_phone` 成功轨迹，位于
`collection_runs/first_success_20261031/pinch_010`。原 TaskEvaluator 未改动；
独立逐物理步复核为 PASS。最终抬升 0.198518 m，位置误差 0.011268 m，
旋转误差 3.7045°，4 个手部接触，全部条件连续满足 0.25 s。
原始记录为 240 Hz、5262 帧（含 5 s 静置）；导出为 508 条 30 Hz 的
54 维状态/动作转移，见 `datasets/phone_success_20261031_state`。

本次物理后端为原生 NVIDIA PhysX GPU，使用原 USD 物理场景与真实接触读取；
仅通过关节位置目标控制。视频由实测刚体位姿离线渲染，未采集 Isaac RTX 五路 RGBD，
也未导出 LeRobot 图像数据集。该结果仅证明基础拿手机任务，不代表手内重抓或功能性使用任务。
验收证据与范围见 [成功报告](collection_runs/first_success_20261031/SUCCESS.md)。

## 历史状态（2026-10-29）

已实现采集入口、实验性状态反馈控制器、原始轨迹记录与官方 LeRobot 双版本导出入口。
**尚未取得真实成功轨迹，控制器和相机采集尚未在 Isaac Sim 中实测。**
本机沙箱外可识别 8 张 A800，但所选 Python 缺少 isaacsim 和 pxr；三任务启动报告为 BLOCKED。
记录见 collection_attempts/20261029T110106_671428Z/pipeline_report.json。
GPU 可见不代表 Isaac Sim 的相机渲染可用。NVIDIA 文档明确不支持无 RT Cores 的 GPU；
须在兼容的 RTX/L4 等设备与对应驱动、Isaac Sim 环境中完成实际验证。

任务为 03_pick_phone、04_phone_portrait_to_landscape、09_rotate_cotton_swab。
先要求每任务一条经评测成功的轨迹；默认每任务最多尝试三次。
这只是打通采集链路的目标，不是足以训练策略的数据规模。
实验性控制器使用仿真物体真值，执行接近、闭合、抬升及世界系重定向。
不使用物体瞬移、焊接、外力或伪造接触；不声称完成已验证的手内旋转。

## 图像和动作协议

- collection_config.json 是本次采集配置：五路 RGB，460×260，30 FPS。
- 五路相机为 Head、LeftWrist、RightWrist、LeftSide、RightSide。
- 在冻结物理时间的同一次渲染中读取五路图像，并核对渲染前后物理时钟。
- 原生低分辨率渲染；水平视场不变，垂直 aperture 适配宽高比，实现方形像素。
- 记录实际 K 和每帧 OpenCV 相机外参，腕部相机外参随手臂运动更新。
- 目标线框为检查辅助，不出现在训练 RGB 中；修改仅在 USD 会话层，不改场景文件。
- 深度是到成像平面的光轴 z 深度，单位米，float32；无效值为 NaN，同时保存有效掩码。
- LeRobot observation.state 为 54 维实测关节角，action 为对应 54 维绝对关节目标，单位弧度。
- 每行对应 observation[t]、image[t]、action[t] 以及执行后的 next.reward、next.done、next.success。
- 每个控制周期 8 个物理步，正常为 1/30 秒；末周期可能因成功/失败提前终止。
- 原始数据额外保存下一状态、速度、接触力、物体位姿、逐物理步评测和深度。
- 物体真值及接触信息保留在诊断侧文件，不混入视觉策略的 observation.state。

全局 cameras.json 的标称分辨率和已有预览图保持原样；采集用的真实分辨率与标定写入每个 episode。

## 运行

先在可用 Isaac Sim Python 环境检查：

    /path/to/isaac/python -B collect_representative.py --preflight-only

单项采集：

    /path/to/isaac/python -B collect_representative.py --task 03_pick_phone --attempts 3 --successes 1

统一入口为每项任务创建独立进程，再从同一批成功轨迹导出两份数据：

    python -B run_collection_pipeline.py --isaac-python /path/to/isaac/python --v21-python /path/to/lerobot21/python --v30-python /path/to/lerobot30/python

固定两个相互隔离的导出环境，不能在同一 Python 环境同时安装两个 LeRobot 版本：

- 数据格式 v2.1：官方 lerobot==0.3.3。
- 数据格式 v3.0：官方 lerobot==0.4.3。
- 导出器调用官方 LeRobotDataset.create/add_frame/save_episode，v3 额外 finalize。
- 编码后使用同版本官方 LeRobotDataset 逐帧读回、解码所有相机并核对帧数。
- 不执行 push_to_hub；所有数据仅保存在本地。

手动导出已有成功原始记录：

    /path/to/lerobot21/python export_lerobot.py --raw-root collection_runs --output datasets/run_lerobot_v21 --format v2.1
    /path/to/lerobot30/python export_lerobot.py --raw-root collection_runs --output datasets/run_lerobot_v30 --format v3.0

导出目录必须不存在，避免覆盖历史数据。默认 RGB 用 MP4，也可用 --media image 嵌入图片。
两版的深度、标定、物理接触、下一状态和证据均完整复制到 extras/episode_XXXXXX/。
深度不是 RGB 视频通道，普通 RGB LeRobot 训练器不会自动读取这些额外深度文件；
要训练 RGBD/点云策略，需在数据加载器中读取深度和 K。

## 原始记录与验收

每条尝试包含 episode.json、physics.jsonl 和 frames/000000/…。
每帧包含五路 PNG、transition.npz 与 evaluation.json。
失败尝试保留诊断，但不进入训练导出；没有成功记录时不创建空的训练数据集。
验收时重新执行原有 TaskEvaluator，逐物理步验证连续成功条件，不修改原成功阈值。
同时检查文件 SHA-256、完整帧序、关节顺序、状态动作时序、相机尺寸、深度掩码及时间连续性。
SHA-256 是完整性检查，不是防恶意伪造的签名；真实成功仍须来自采集程序的物理运行。

## 测试

    python -B -m unittest discover -s tests -v
    /path/to/lerobot21/python tests/lerobot_roundtrip_smoke.py --format v2.1 --report collection_checks/lerobot_v21.json
    /path/to/lerobot30/python tests/lerobot_roundtrip_smoke.py --format v3.0 --report collection_checks/lerobot_v30.json

格式测试只在临时目录创建明确标注的合成色块和数值，结束后删除；不会伪装成机器人成功轨迹。
只有报告存在且 status=PASS 才代表对应官方读写回环已经通过，代码语法通过不等于导出验证通过。

2026-10-29 本地验证结果：30 项离线单元测试通过；两版官方格式回环均 PASS。
每版测试包含 2 条合成 episode、8 行状态/动作、5 路 460×260 视频，并逐帧核对图像、
动作、episode/task/frame 索引和结束标志。对应报告为 collection_checks/lerobot_v21.json
与 collection_checks/lerobot_v30.json。合成数据不计入真实任务成功数。

本机 .venvs/lerobot-v21 和 .venvs/lerobot-v30 是数据格式检查环境，复用了系统 torch/torchvision；
它们没有安装 LeRobot 全套训练/机器人设备依赖，不能视为完整训练环境。
v3 所需 accelerate/safetensors 位于 .venvs/loader-support，由环境中的 .pth 引用。
为兼容系统已有 ONNX/protobuf，导出器在加载 LeRobot 前选择 protobuf 的 Python 实现；
不会改写系统包。跨机器运行建议在干净虚拟环境中安装相应固定版本的完整依赖。

## 官方参考

- https://github.com/huggingface/lerobot/tree/v0.3.3/src/lerobot/datasets
- https://github.com/huggingface/lerobot/tree/v0.4.3/src/lerobot/datasets
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html
