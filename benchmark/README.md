# 通用双臂灵巧操作仿真基准

## 真实扫描资产升级（2026-10-31）

已下载23个YCB官方带纹理扫描模型，按语义匹配接入28个场景、45个物品实例（基础3个、Sharpa扩展25个）；剩余55个场景尚未升级。全部83场景新总册见[更新图册](reports/benchmark_all_83_scenes_textured_update_20261031.pdf)，前后对比与剩余项见[升级报告](reports/real_assets_20261031/README.md)。来源与CC BY 4.0署名见[ATTRIBUTION](assets/external/ycb/ATTRIBUTION.md)。本次是视觉资产升级，原物理代理保留；尚未完成接触拟合与实测。原PNG/PDF为历史图像，未覆盖为新验证结果。

三项代表任务的采集与 LeRobot 导出见 [COLLECTION.md](COLLECTION.md)。2026-10-31 已取得 1 条经原判定器独立复核的拿手机成功轨迹：[状态/动作数据](datasets/phone_success_20261031_state/metadata.json)、[实验报告](collection_runs/first_success_20261031/SUCCESS.md)。本次使用原生 GPU PhysX，视频为实测位姿离线回放；五路 Isaac RGBD 与 LeRobot 图像数据采集仍未完成。

## Sharpa demo 扩展（2026-10-29）

新增 72 个本地场景原型，复用现有工作单元：25 个含刚体近似任务判定，47 个仅有
道具/布局。统一入口为 python -B benchmark_cli.py list 与
python -B benchmark_cli.py run <任务ID>；详见 sharpa/README.md、
sharpa/coverage.md 和 sharpa/gallery.html。
**官方 demo 来源仍待逐项复核；物理运行因缺少 Isaac/NVIDIA 环境而阻塞，不能称为全部复现。**
原有十任务保留在 tasks/index.json，新入口位于 sharpa/index.json；加上双手卷毛巾，共 83 个注册入口。

## 功能性抓取与手内重抓续建（2026-10-30）

设计、已有能力与缺口见 [functional/README.md](functional/README.md)。新增离线判定器使用掌心系手物相对位姿，联合接触区域、手指运动、接触变化与实际用途结果；包含防止整臂旋转误报的测试。示例契约尚未标定，未接入 Isaac 实测，不计为新增完成场景。

毛巾的 GPU 准备入口与待办见 [towel_roll/ISAAC_PREPARATION.md](towel_roll/ISAAC_PREPARATION.md)。统一命令 `python -B benchmark_cli.py run bimanual_roll_towel --prepare-only` 会检查运行环境并在可用时编写 surface-deformable 场景；成功编写不代表运行或卷起成功。

本项目用于模型训练与仿真。采用中性资产名、相对路径和无品牌材质，包含双臂、左右多指手、五路 RGBD 相机、带操作豁口的弧形桌面，以及十个独立操作任务。

## 打开场景

在 Isaac Sim 中打开 `dealer_table.usdc`，可查看完整双臂、牌桌、筷子架与双筷场景。`scene.usda` 是同一场景的轻量入口。

十个任务的入口为 `tasks/<任务目录>/scene.usda`，共用 `tasks/workcell.usdc`；移动项目时请保留整个 `tasks` 目录。任务清单与初始/目标姿态在 `tasks/index.json`，各任务单独提供 `task.json`。

| 目录 | 任务 |
| --- | --- |
| `01_pick_spoon` | 拿勺子 |
| `02_pick_knife` | 拿刀具 |
| `03_pick_phone` | 拿手机 |
| `04_phone_portrait_to_landscape` | 手机竖屏变横屏 |
| `05_flip_phone` | 手机翻面 |
| `06_reorient_battery` | 拿电池并调整姿态 |
| `07_reorient_lighter` | 拿打火机并调整至使用姿态 |
| `08_pick_lipstick` | 拿口红 |
| `09_rotate_cotton_swab` | 拿棉签并绕长轴旋转 |
| `10_pick_scissors` | 拿剪刀 |

任务中的青色线框显示目标姿态，不参与碰撞。道具的尺寸、质量是仿真设计值。剪刀用于闭合状态抓取，打火机无点火模拟。已补齐重置、关节目标控制、动作回放及成功检测代码；自动抓取策略、手指接触规划仍未实现。Isaac 适配器尚待当前场景实测，不能把离线测试或场景预览解释为任务执行成功。

## 续建进度（2026-10-29）

- 十个场景均已有独立入口、道具、夹具、目标姿态和五路 RGBD 预览。
- `task_runtime.py`：任务契约、姿态/抬升/接触/速度联合判定、持续成功计时、掉落及超时终止。
- `isaac_task_env.py`：场景加载、54 关节名称映射、物体与关节状态重置、真实接触力读取及逐物理步判定，供外部策略调用。
- `run_task_episode.py`、`run_all_task_episodes.py`：单任务/十任务隔离运行、重复重置检查和动作回放。
- `check_task_runtime.py`：无需 Isaac 的资产、初始/目标位姿、图像/深度和已保存 IK 正解复验；安装 USD 绑定后增加场景组合检查。
- 新报告记录输入文件及运行代码的 SHA-256，旧报告不再自动当作当前物理验证通过。

本次环境仅能完成 CPU 离线检查；缺少 `isaacsim`、USD 绑定和可用 NVIDIA 驱动，依赖下载亦受网络限制。因此物理运行记录为 `BLOCKED`，整体实测状态为 `INCOMPLETE`。原始迁移内容保存在项目同级的 `benchmark_before_20261029`，不属于分发包。

```bash
# 普通 Python：numpy、scipy、Pillow 即可
python -B -m unittest discover -s tests -v
python -B check_task_runtime.py

# 在可用 Isaac Sim Python 环境中重新运行
python -B run_task_episode.py 01_pick_spoon
python -B run_all_task_episodes.py
python -B check_task_scenes.py
```

批量程序以独立进程运行每个场景，详细异常日志写入系统临时目录。退出码：`0` 通过、`1` 失败、`2` 环境缺失或证据不完整。无动作文件时检查重复重置、5 秒静置和接触读取，`PASS` 不表示成功抓取。

### 控制与成功判定

先启动 `SimulationApp`，再创建 `IsaacTaskEnv(task_id, app)`；调用 `reset()` 后，使用 `step({关节名: 绝对目标弧度})` 控制。未指定的关节保持上次目标，越限或未知关节报错。观测包含关节状态、物体位姿/速度、目标位姿及指定手的逐刚体接触力；相机路径和标定由 `env.cameras` 提供，图像采集仍由已有渲染脚本负责。

每个 `task.json` 的 `runtime` 保存可编辑运行参数：240 Hz 物理步、8 步控制间隔、1200 步静置、20 秒回合上限。原任务容差仍为 1.5 cm、12°、抬高至少 10 cm；额外要求至少两个指定手刚体的接触力各达到 0.001 N，线速度不超过 0.05 m/s、角速度不超过 0.25 rad/s，连续满足 0.25 秒才成功。接触条件只是抓持代理指标，不证明力闭合；新增数值均为显式设计参数，未经实物标定。

姿态误差使用完整旋转矩阵，因此棉签绕长轴旋转不会被对称性忽略。抬升量相对本次重置后的静置位置计算；低于该位置 8 cm 判为掉落。到达目标但没有手部接触不会成功。

回放文件为 JSON，包含 `task` 和 `actions`；每项动作包含 `joint_targets_rad`（关节名到弧度的映射）以及可选正整数 `control_steps`。使用 `python -B run_task_episode.py <任务ID> --actions <文件>` 执行。该入口回放用户提供的控制目标，不会自动生成抓取动作；`runtime_status` 与 `task_success` 分开记录。

## 图像与传感器

- `images/all_tasks_five_camera_views.png`：十场景 × 五路相机大图，6624×8784；每个 1280×720 相机画面原像素保留，未裁剪。列依次为头部、左腕、右腕、桌左侧、桌右侧，左右以机器人为准。
- `images/all_tasks_five_camera_views_preview.jpg`：便于浏览的缩略版。各任务 `images/five_camera_views.png` 为单场景五视角拼图。
- `images/all_tasks_five_camera_views_manifest.json`：50 张源图路径、哈希、画面位置和核验记录。使用 `python -B compose_five_camera_gallery.py` 可重建；图像来自已有仿真渲染，本次只做排版合成。
- `images/camera_views_overview.png`：中心整体布局，周围五路机位画面及引线标注。
- `images/task_scenes_overview.png`：十项任务、整体布局与可达性对比。
- `tasks/<任务目录>/images/`：五路相机画面、理想几何深度、整体图和道具细节图。
- `cameras.json`：内参、外参、安装父刚体与成像约定。

头部相机观察全桌，左右腕相机位于掌心侧手腕根部，侧面相机紧邻桌边朝向桌面，无支架。左右方向以机器人自身为准。

RGB 标称分辨率为 1920×1080，水平/垂直视场为 86°/55°。任务预览图缩放至 1280×720，使用预览图进行投影时应同步缩放内参。深度为与 RGB 对齐的理想几何射线距离，不包含双目匹配、噪声或红外干扰模型。

## 机械臂布局

机器人整体向桌面前移 0.38 m，肩部中点的纵向坐标由 −0.43 m 调整为 −0.05 m。按 5 cm 网格、掌心朝下的固定姿态及保守桌面/底座间隙筛选，至少一只手可达的采样点从 119/535 增至 272/535（22.2% → 50.8%）。详见 `tasks/reachability.json` 与对比图。这是所测试平移候选中的结果，不代表全姿态的全局最优，也未验证整段运动的精细网格碰撞。

## 重建与验证

在已配置 Isaac Sim 的 Python 环境中运行。需要 `numpy`、`scipy`、`trimesh`、`shapely`、`Pillow`、`matplotlib` 和 USD Python 绑定；总览图另需 `cairosvg` 与中文字体。

```bash
export PYTHONDONTWRITEBYTECODE=1
python build_scene.py
python build_task_scenes.py
python check_scene.py
python render_scene.py
python check_camera_motion.py
python check_coverage.py
python render_all_tasks.py
python render_task_details.py
python run_all_task_episodes.py
python check_task_scenes.py
python compose_camera_diagram.py
python compose_task_gallery.py
python check_privacy.py
```

`render_all_tasks.py` 将运行日志写到系统临时目录，避免把运行环境路径带入项目。需要重新优化位置时，先运行 `optimize_workspace.py`，再运行 `apply_workspace_layout.py` 并重建场景；可达性检查可运行 `check_task_reach.py`。

## 中性资产与检查记录

`assets/dual_arm/robot.urdf` 是用于训练的中性组合模型，保留关节轴、限位、惯量、安装关系和碰撞几何；网格文件使用编号命名，二进制 STL 文件头已清理。`assets/rgbd_camera/robot.urdf` 保留中性相机几何与光学坐标系。原始厂商结构目录、源仓库副本、旧版本归档、身份映射、原始署名及许可记录均存放在项目外，未打包进此目录。脱敏不改变原始资产权属。

- `privacy_report.json`：文本、文件名、解码后的二进制 USD、STL 文件头和图片元数据检查。
- `model_integrity.json`：脱敏前后关节、惯量、安装姿态和三角网格一致性检查。
- `geometry_checks.json`：54 个可动关节、关节锚点和五路相机安装检查。
- `validation.json`：主场景 5 秒物理稳定性检查。
- `camera_motion_checks.json`：左右手腕相机分别随自身关节运动的验证。
- `tasks/runtime_checks.json`：本次离线资产、位姿、图像/深度、保存逆解正解复验；USD 组合是否执行单独记录。
- `tasks/runtime_summary.json` 及各任务 `runtime_validation.json`：本次重置、物理静置及接触接口检查；环境缺失明确记录 `BLOCKED`。
- `tasks/validation_summary.json`：严格汇总当前证据；缺少 USD 组合或当前物理验证时为 `INCOMPLETE`。各任务旧 `validation.json` 的物理数值来自脱敏前的同几何版本，仅保留作历史参考，不能替代本次实测或任务执行成功率。

`check_privacy.py --terms-file <外部词表路径>` 可额外检查私有身份词表（JSON 字符串数组）。词表与原始敏感值不写入项目或报告。检查器不将视觉检查等同于文本扫描；外观检查范围在 `model_integrity.json` 中单独记录。
