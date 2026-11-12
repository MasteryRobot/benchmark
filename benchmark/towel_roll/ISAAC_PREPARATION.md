# 双手卷毛巾：Isaac 接续状态

## 当前要求：8 mm 毛巾与机械手关节交互

用户明确要求 GPU 物理仿真，使用机械手与毛巾接触将其卷起。旧 CPU 角点约束 demo 不符合此要求，不作为替代结果。

新配置 `interaction_config.json` 使用 8 mm 厚、36×40 cm、质量 240 g 的候选毛巾，放在 3 cm 高的支撑垫上。厚度同时进入物理表面材料和碰撞 rest offset；弹性与摩擦仍是待标定设计值。

`joint_plan.py` 生成 54 关节目标，包含接近、下降、夹持、抬起、65° 初始卷动尝试、松开和退回。它只通过了离线运动学检查，不证明手指抓得住、不证明整段运动无碰撞，也不是完整闭环卷毛巾策略。

`gpu_interaction.py` 是未经实际 Isaac 运行验证的 GPU 执行入口：强制检查 CUDA 布料张量；初始化机器人后只向关节驱动器发送目标；布料节点只读取，不写入目标位置，不添加附件或顶点约束。运行完成时保存实际关节状态和原生变形节点坐标，状态为 `ATTEMPT_RECORDED_NOT_VALIDATED`。目前没有实现手/布料原生接触分类，不能认证成功。

```bash
cd /data/benchmark
# 在具备 GPU 和 Isaac Sim 的执行环境中，用对应的 Python 执行。
# 输出为新目录，包含报告和实际状态；已有目录拒绝覆盖。
python -B run_towel_roll.py --backend isaac --joint-interaction --output towel_roll/attempts/thick8_gpu_run_001
```

本次实际启动的报告在 `attempts/thick8_gpu_interaction_20261030/report.json`：GPU 驱动不可访问、Isaac Sim 不可用，物理执行尚未开始。手指接触调试、重复卷动策略、成功判定和新 GPU 视频均未完成。

关节控制接口参照 [Isaac Sim articulation controller](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/robot_simulation/articulation_controller.html)，实际布料状态读取参照 [Omni Physics tensors](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/109.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html)。实际安装版本的兼容性仍需验证。

## 先前的场景准备入口

2026-10-30：统一入口参数已修复，CPU 探索依赖改为按需加载。新增 `surface.py` 和 `prepare_isaac.py`，可在具备对应 schema 的 Isaac 环境中编写独立 surface-deformable USD 层。当前只验证网格几何、入口与失败状态，未执行 USD 编写、物理仿真或闭环控制。

## 版本选择

旧 `isaac_config.json` 是 PBD particle cloth 的未验证设计参数。NVIDIA 当前迁移文档说明旧 particle cloth 被 surface deformables 替代，且两者物理模型不同，不能直接迁移弹簧常数或把结果视为相同协议：

- [NVIDIA deformable migration](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/deformables/deformable_migration.html)
- [NVIDIA surface deformable authoring](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.1/dev_guide/deformables/deformable_bodies.html)

新准备代码只复用旧文件中的摆放/尺寸变体和网格间距。FEM 材质参数在 `surface.py` 中显式给出，尚未经标定；没有声称等价于旧 PBD 配置。RSI 的冻结 Isaac 环境独立保留，不应为了毛巾更新它。

## 命令与输出

在能访问 GPU、且已安装 Isaac Sim 的 Python 下执行：

```bash
cd /data/benchmark
python -B benchmark_cli.py run bimanual_roll_towel --prepare-only
# 手动准备其他摆放变体；这不是一次物理尝试
python -B run_towel_roll.py --backend isaac --prepare-only --variant raised_nearer
```

输出默认带独立时间戳，已有报告拒绝覆盖。准备成功状态为 `AUTHORED_NOT_SIMULATED`，运行环境缺失返回退出码 2 和 `BLOCKED`。原始 `scene.usda` 保留，新层引用原场景；输出层使用本机绝对引用，迁移时需重建或重新定位引用。

本次实测报告为 `attempts/resume_all_20261030_preflight.json`：驱动不可访问、当前 Python 无 Isaac Sim，`physics_started=false`，新增物理尝试为零。

## 尚未完成

1. 在实际安装版本下验证 schema、材质绑定、自碰撞、真实厚度参数、桌面接触及静置稳定性，记录软件/驱动版本。
2. 接入双手关节控制和手/布料实测接触，不以布料顶点移动、固定约束或物体吸附替代手指夹持。
3. 从实际变形网格计算绕卷、伸长、体积范围和松手后稳定性；保存过程视频及接触证据。命令转角不是实际卷绕角。
4. 按 baseline、抬高、移近、加宽、加厚等既有顺序进行物理尝试；只有前一变体实际失败才进入后一变体。环境错误应停止，不计入物理失败或成功率。

目前不带 `--prepare-only` 的 Isaac 入口明确报告控制器与评测未接入，不会返回一次成功的假仿真。旧 CPU 约束近似仅供历史参考。
