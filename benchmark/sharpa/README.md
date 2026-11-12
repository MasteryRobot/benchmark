# Sharpa demo 场景扩展

目标是尽可能覆盖 Sharpa 展示的 demo。目前交付 **72 个本地原型入口**，
共用既有机器人与工作台。**尚未完成全部官方 demo 复现**：本次搜索没有取得
可保存复核的官方正文，59 项桌面任务与另外 13 项演示条目仍是待核实候选。
先前对话中“已确认官方 59 项”的说法不能作为证据，官方覆盖数量暂不统计。
没有下载官方 CAD、示教数据或控制策略；发牌在桌面任务与演示任务中重复出现，
因此入口数也不是独立 demo 数。

## 能力范围

- **25 个刚体近似任务**：提供物体、目标、重置/控制接口和判定逻辑，包括发牌、
  风车零件插装、杯盘整理、插接对位、拍照、回球和接棒。
  判定逻辑已做 CPU 回归测试；未完成 Isaac 物理实测或自动操作策略。
- **47 个道具场景**：提供场景和相同控制入口，但缺少柔性、液体、螺纹、
  切割、研磨、黏附或清洁过程，任务判定为 NOT_IMPLEMENTED，不能报告任务成功。
- **现有工作单元**：中性 54 自由度双臂、桌面、灯光、五路相机均来自
  ../../tasks/workcell.usdc。此模型不是经认证的 Sharpa 最新机器人复刻。
- **可移植资产**：每个 scenes/<id>/scene.usda 引用 ../../../tasks/workcell.usdc；
  保留整个 benchmark 目录即可，原有十任务的资产、入口和执行逻辑不变。

完整参数见 index.json 和各场景 task.json；逐项能力和缺失物理见 coverage.md。
gallery.html 和 scene_layouts.jpg 是 **CPU 俯视布置示意，不是 Isaac RGBD 渲染**。
尺寸、质量、摩擦、容差均为显式本地设计值，不能视作实物标定。

## 运行

以下命令在 benchmark 根目录执行：

    python -B benchmark_cli.py list
    python -B benchmark_cli.py list --suite sharpa
    python -B build_sharpa_scenes.py
    python -B -m unittest discover -s tests -v
    python -B check_sharpa_scenes.py --offline-only
    python -B render_sharpa_layouts.py
    python -B check_sim_environment.py

在已安装并能运行的 Isaac Sim Python 环境中：

    python -B benchmark_cli.py run sharpa_card_dealing --gui
    python -B run_sharpa_suite.py
    python -B run_sharpa_suite.py --evaluated-only
    python -B check_sharpa_scenes.py

可使用已安装 Isaac Sim 的 python.sh 替换 python。CPU authoring 依赖见
requirements-authoring.txt；requirements-usd.txt 仅补 USD 组合检查，不能替代
Isaac Sim、PhysX、RTX 或 NVIDIA 驱动，也不要将另一版 USD 强行装入 Isaac 内置环境。
当前机器缺少这些运行条件，批量运行逐项记录 BLOCKED。

批量入口默认每场景独立进程、300 秒超时。退出码 0 表示重置/接触 smoke 通过，
1 表示失败，2 表示环境阻塞。**smoke PASS 不表示成功操作，更不是成功率**。
完整检查总状态保留 INCOMPLETE，直到来源、缺失物理和实际运行验证均补齐。

## 控制与观测

启动 SimulationApp 后创建 sharpa.isaac_env.SharpaScenarioEnv(task_id, app)，
调用 reset()，再调用 step({关节名: 绝对目标弧度}, capture=False)。
越限、未知关节及非有限值均被拒绝，未指定关节保持上次目标。不接受物体瞬移动作。
球、棒仅在 reset 后第一步施加一次初速度，其后由物理引擎演化。

返回关节状态、每个动态物体的世界位姿和速度、左右手逐刚体接触力与判定结果。
五路相机路径保存在 env.cameras；其真实 RGBD 采集本次没有跑通。
摄影任务额外挂载 CaptureCamera，capture=True 时必须拿到非空真实图像，
并在双手持握和目标位姿满足时才认可该事件。它不能检验照片审美或自动对焦。

用户回放文件包含 task 和 actions，每项支持 joint_targets_rad、正整数
control_steps、以及仅用于摄影场景的 capture。示例（保持关节，不会自动抓取）：

    {"task":"sharpa_camera_photography","actions":[
      {"joint_targets_rad":{},"control_steps":30},
      {"joint_targets_rad":{},"capture":true,"control_steps":1}
    ]}

    python -B run_sharpa_episode.py sharpa_camera_photography --actions actions.json

## 判定边界

发牌要求全部目标位姿、曾有手部接触、释放后低速稳定；双手持相机要求两侧接触及
当时图像事件。回球要求手持球拍时真实触球，再正确跨越回球平面；接棒要求持续
低速抓持。接触数量只是抓持代理指标，不证明力闭合。

风车采用水平装配面和竖直轴的本地简化件，没有螺纹、卡扣或自动焊接；显卡、
插头、电池的判定仅检查几何对位，没有电气功能。纸张、袋子、布、线缆目前是
刚体道具代理，不能用来验证其真实操作技能。SCENE_ASSETS_ONLY 场景从不成功。

## 证据与下一步

validation_summary.json 分开保存离线结构、初始几何、USD 组合和物理运行状态；
runtime_summary.json 与各场景 runtime_validation.json 保存运行尝试和 SHA-256。
代码或输入更改后旧记录判为 STALE。原有关键文件哈希见 baseline_hashes.json。

离线检查覆盖配方、相对引用、初始高度、保守桌面区域和道具间 AABB 重叠，
不等于机器人可达性、精细碰撞、目标接触或动态稳定性。pxr 缺失时 USD 组合为
NOT_RUN。几何示意不能作为物理证据，单元测试的合成观测也不能作为控制策略。

后续需获取可核对的官方 demo 网页/视频和时间戳，逐项校正 source_task，
在具备 NVIDIA/Isaac 的环境中做组合与动态检查，再补充柔性、液体及操作策略。
当前 catalog.py 中 URL 仅为候选定位符，不能当作成功取回的来源证明。
