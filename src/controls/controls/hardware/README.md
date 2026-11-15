# 实际电机焊接控制

本目录是 `controls` 包的实机执行层，与 `../simulation/` 同级。
`../controller/` 提供共享标定、速度/加速度约束、轨迹求解和 CMA-ES 搜索。
实机执行代码已经存在，尚未完成带载焊接运动的实机验证。

| 文件 | 职责 |
| --- | --- |
| `weld_ports.py` | `ArmOwner` / `TrajectoryArmPort` 对接达妙六轴；`ChassisOwner` 对接 ENCOS 底盘；`SimulatedArm` 用于离线测试 |
| `weld_control.py` | 共享执行状态机：当前实测位置 → 初始姿态 → 焊接起点 → 等待明确启动 → 焊接运动 → 保持；反馈、到位、暂停与故障处理 |
| `weld_app.py` | 控制面板和单执行线程；选择实机或模拟接口，机械臂和底盘独立开关 |
| `weld_plan.py` | 兼容导出；实际求解实现在 `../controller/weld_plan.py`，避免两套规划逻辑 |

## 实体电机指令链

机械臂指令经 `WeldController` → `ArmOwner.command()` →
`TrajectoryArmPort.set_targets()` → 现有 `ArmGroupHoldPort` / Damiao 总线
→ DM-MC02 → CAN 电机。发送的是六轴原始输出轴位置和速度，使用原生
position-velocity mode 2 / USB type 4；不叠加手动点动的 type-25 平滑。
六轴报文依次发送，尚不是固件原子同步下发。

底盘指令经 `WeldController` → `ChassisOwner.velocity()` →
`DifferentialDrive` 四轮分配 → `MotorBus.velocity()` → 现有 ENCOS 驱动。
开关仅控制输出，不增加机械臂对底盘关闭的补偿。

默认不会选择实机接口。`scripts/control_weld.py --execute` 才会选择真实
owner，并要求明确的 `--arm` / `--chassis` 和已有夹具/轮系映射确认参数。
打开实机面板不自动执行焊接：仍需选择参与部分、根据新鲜反馈计算路径、
到初始姿态和焊接起点，再明确启动焊接运动。程序不控制焊机起弧。
实机操作由操作者执行，见 [焊接流程说明](../../../../docs/WELD_MOTION.md)。

## 动画、离线执行与实机模式

- `--plan-only`：用与实机相同的求解器生成并检查完整轨迹，不创建电机 owner。
- `--simulate`：同一 `WeldController` 配模拟电机，检查顺序、指令、反馈与停止逻辑。
- `scripts/render_weld.py`：直接回放保存的轨迹并绘制 MuJoCo 模型，不经过
  电机 owner，不是实体电机实测录像，也不是负载动力学仿真。
- `--execute`：切换到已有实机适配器。此前的动画和测试没有执行这一模式。

保存的动画对应 `build/weld/cmaes-animation-plan.json`，其 12 周期完整流程已
另行通过模拟电机状态机回放。项目目标保留 2 Hz；当前模型约束下的实际计划约
1.8153 Hz。到位阶段 5°/s，六轴焊接速度上限 68.7549°/s。
重量补偿按 LOAD-01 排在首轮机械臂焊接跑通之后；传感器位置反馈和 ESO
评估记录在 FEEDBACK-01。目前已有电机内部位置环和主机跟踪监测，尚未加入
外部焊枪位置补偿、负载前馈或 ESO。

硬件适配层的既有验证在 `tests/test_weld_hardware.py`：使用原生 C 固件加
假 CAN 电机检查达妙组指令、暂停保持和反馈丢失；使用 ENCOS Python 模拟器
检查底盘进给。它们不等于实际负载、总线时序或轨迹跟踪验收。
