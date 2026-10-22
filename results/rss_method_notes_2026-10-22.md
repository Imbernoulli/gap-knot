# RSS / GaP 方法核对（2026-10-22）

这份笔记记录本轮对 RoboSimStudio、SuperDex、GaP 和 open-robot-skills 的源码核对。重点是区分已确认的引擎行为和仍需实验确认的迁移假设。本文没有修改 task 或 grader，也没有运行新仿真。

## 先更正一个结论：`scene_friction` 对当前 SuperDex 任务不生效

我最初把 VBD 电缆例子的 `scene_friction=True` 当成可以直接迁移到当前 SuperDex 任务的材料分离开关；源码核对后撤回这个结论。

已确认：

- `rss.rod(...)` 在 SuperDex backend 下注册的是 `EntityRodSuperdex`，见 [`rod/__init__.py:106-112`](../third_party/robosimstudio/robosimstudio_materials/rod/__init__.py)。该类继承 `ParticleMixin, Centerline, Entity`，见 [`superdex.py:58-67`](../third_party/robosimstudio/robosimstudio_materials/rod/superdex.py)，不是 VBD 的 `EntityRod`。
- `EntityRodSuperdex.unread_fields` 把 `mu_base`、`mu_gripper`、`mu_surface`、`surface_labels`、`scene_friction` 列为不会进入 SuperDex actor 的字段，见 [`superdex.py:69-85`](../third_party/robosimstudio/robosimstudio_materials/rod/superdex.py)。SuperDex 的 host actor 只把绳本身的 `mu_cable` 转成 `friction=sqrt(mu_cable)`，见 [`superdex.py:175-201`](../third_party/robosimstudio/robosimstudio_materials/rod/superdex.py)。
- SuperDex actor builder 的 `CONSUMES` 列表没有这些 scene-material 字段，见 [`rod/actor.py:330-358`](../third_party/robosimstudio/robosimstudio_materials/rod/actor.py)。因此改变 `scene_friction`、`mu_gripper` 或 `mu_surface` 不能把当前 SuperDex 的 YAM 指面重新标成 VBD 例子的 `60`。
- `scene_friction` 的材料重写确实存在，但属于 VBD `EntityRod` 的 `apply_model_materials`，见 [`rod/entity.py:782-837`](../third_party/robosimstudio/robosimstudio_materials/rod/entity.py)。它写 Newton `shape_material_mu`；SuperDex 类没有这段 override。VBD 的电缆例子 [`examples/08_demos/route_the_cable/board.py:95-113`](../third_party/robosimstudio/examples/08_demos/route_the_cable/board.py) 和 README 的 `mu_gripper=60, mu_surface=.02` 不能作为当前 SuperDex 的抓持证据。
- 上游测试专门审计了这个区别：[`tests/test_rod_loss_is_loud.py:67-109`](../third_party/robosimstudio/tests/test_rod_loss_is_loud.py) 将 `scene_friction` 等字段列为 SuperDex 丢失字段；[`tests/test_rod_loss_is_loud.py:145-208`](../third_party/robosimstudio/tests/test_rod_loss_is_loud.py) 还验证 SuperDex 构建、材料阶段和 tuning 不读取这些字段。

所以，当前任务的已确认摩擦轴是 `mu_cable`（以及其接触参数），而不是 `scene_friction` 分离 pad/桌面材料。不能再写“当前 YAM 指面 μ=3、打开 scene_friction 后变成 μ=60”作为 SuperDex 事实；那只适用于 VBD 路线，或仅是待验证的另一种 backend 方案。

## SuperDex 当前真正读取的物理参数

在 [`knotlab/task.py:309-350`](../knotlab/task.py) 中，任务选择 `backend("superdex", substeps=2, feedback="off")`，并给 rod 传入 `young_modulus`、`axial_stiffness`、`mu_cable`、`friction_falloff=1e-3`、`contact_penalty=4e8`、`skin_samples="P1Q6"` 和 `self_contact=True`。

- `mu_cable` 进入 SuperDex `ContactParams.coulomb_friction_coefficient` 的平方根，见 [`superdex.py:175-188`](../third_party/robosimstudio/robosimstudio_materials/rod/superdex.py) 及 [`rod/actor.py:183-205`](../third_party/robosimstudio/robosimstudio_materials/rod/actor.py)。当前 Variant 默认 `mu_cable=1.0`，训练变体约为 `0.6–1.4`，见 [`knotlab/task.py:25-47`](../knotlab/task.py) 和 [`harness/variants.py:1-45`](../harness/variants.py)。提高它会同时改变绳与桌面、绳与绳、绳与镜像指面接触，不能被解释为“只提高夹爪摩擦”。
- `friction_falloff` 是 SuperDex 的低速摩擦正则化阈值；低于该速度时摩擦按速度增长，并不存在理想静摩擦。RodSpec 的说明见 [`rod/spec.py:202-217`](../third_party/robosimstudio/robosimstudio_materials/rod/spec.py)，SuperDex 将它传入 actor 见 [`superdex.py:198-201`](../third_party/robosimstudio/robosimstudio_materials/rod/superdex.py)。它比 robot 的 `target_ke` 更直接地影响保持期间的 creep，但仍是全绳接触参数。
- `contact_threshold`、`contact_smoothing`、`contact_penalty` 和 `contact_padding` 进入 rod 的 ContactParams，见 [`rod/actor.py:163-205`](../third_party/robosimstudio/robosimstudio_materials/rod/actor.py)。镜像刚体的 smoothing/threshold 则由 `engines/superdex/mirror.py:93-137, 228-247` 另行建立；这两个接触侧的参数不能混称。镜像文档还说明低速 falloff 由正在碰撞的 deformable 一侧决定，见 [`mirror.py:138-164`](../third_party/robosimstudio/robosimstudio/engines/superdex/mirror.py)。
- `self_contact=True` 是保住已有交叉的必要条件之一。官方预置松结例子 [`examples/03_materials/03_knot.py:38-48`](../third_party/robosimstudio/examples/03_materials/03_knot.py) 明确说明关闭 self-contact 会让绳穿过自身。

## `feedback="off"` 与 `target_ke`

已确认：`feedback="off"` 并不是一个“更硬的夹爪”开关。SuperDex bridge 只有在 feedback 非 off、mirror 是 moving 且 two-way 时，才收集接触 wrench 并在下一个 control step 回传，见 [`engines/superdex/bridge.py:434-462`](../third_party/robosimstudio/robosimstudio/engines/superdex/bridge.py)。安排文档还记录了两向反馈的迟滞和不稳定抓持结果，见 [`robosimstudio/specs/arrangement.py:476-491`](../third_party/robosimstudio/robosimstudio/specs/arrangement.py)。

`ActuationSpec.target_ke/kd` 写的是 Newton 机器人 finger joint 的位置伺服增益，见 [`robosimstudio/robots/entity.py:1059-1073`](../third_party/robosimstudio/robosimstudio/robots/entity.py)。在 SuperDex `feedback="off"` 且实际 pad pose/gap 轨迹相同的前提下，它不会直接增加绳与指面的摩擦或法向接触力；它最多让指关节更接近命令宽度，从而间接改变实际夹口。VBD route 例子里 `target_ke=6000` 的成功数据来自 VBD 刚体接触、sticky contact history 和动态夹指，见 [`route_the_cable.py:183-196`](../third_party/robosimstudio/examples/08_demos/route_the_cable/route_the_cable.py)；不能把这个数当成 SuperDex 的 grip gain。

## `stress_free` 的真实行为

这个参数与 `scene_friction` 不同，SuperDex **确实读取**它。

- `EntityRodSuperdex.stress_free_nodes()` 将 `stress_free="straight"` 展开成沿首段方向、保持每段弧长的直线；给定 polyline 时检查节点数和每段长度，见 [`superdex.py:127-143`](../third_party/robosimstudio/robosimstudio_materials/rod/superdex.py)。长度不匹配会直接 `ValueError`。
- `host_actor()` 把结果放入 actor params 的 `stress_free`，见 [`superdex.py:196-197`](../third_party/robosimstudio/robosimstudio_materials/rod/superdex.py)。SuperDex actor builder 读取它作为 `free` 形状，见 [`rod/actor.py:124-163`](../third_party/robosimstudio/robosimstudio_materials/rod/actor.py)。如果它与初始 `nodes` 不同，builder 创建 stress-free 形状后通过 `teleport_rod` 将 rod 带到初始 polyline，再释放边界条件，见 [`rod/actor.py:207-242`](../third_party/robosimstudio/robosimstudio_materials/rod/actor.py)。
- 上游测试验证了该语义：同一条 L 形 rod，`stress_free=None` 保持弯折，而 `stress_free="straight"` 在悬挂后基本拉直，见 [`tests/test_superdex_contact.py:360-410`](../third_party/robosimstudio/tests/test_superdex_contact.py)。官方 Real2Sim 场景也使用 `stress_free="straight"`，见 [`examples/live/scene.py:100-115`](../third_party/robosimstudio/examples/live/scene.py)。
- `stress_free` 不能用于 closed loop；RodSpec 会拒绝 closed rod 与 stress_free 同时出现，见 [`rod/spec.py:251-261, 271-300`](../third_party/robosimstudio/robosimstudio_materials/rod/spec.py)。

对当前 knot task 的含义：当前 `rss.rod` 没有传 `stress_free`，因此它把 `layout(...)["start"]` 同时作为初始/无应力形状。把 `stress_free="straight"` 加到一个已经弯曲的 knot/loop 初始布局会产生一个新的物理问题（绳会主动回弹、改变起始几何），不能当作“从直绳学习打结”的免费初始化，也未经本任务 grader 验证。

## RSS 官方结绳与电缆路线能迁移什么

可确认的可迁移方法来自官方 [`examples/08_demos/route_the_cable/route_the_cable.py`](../third_party/robosimstudio/examples/08_demos/route_the_cable/route_the_cable.py)：

1. `_take()` 在下降和闭爪前重新读取材料节点；`_offset()` / `_held()` 从闭爪时记录材料点相对 hand/TCP 的 offset，并在运动的每一步累积最大 slip（约 [`route_the_cable.py:54-74`](../third_party/robosimstudio/examples/08_demos/route_the_cable/route_the_cable.py)）。这套诊断适用于当前 SuperDex，即使接触引擎不同。
2. `place()` 按材料局部切线测试 yaw 与 yaw+π，并用 IK 误差选姿态（约 [`route_the_cable.py:76-91`](../third_party/robosimstudio/examples/08_demos/route_the_cable/route_the_cable.py)）。
3. `plug()` 不追着入口点让 TCP 直冲，而是根据抓点到自由端的实时向量，两轮转腕、横移，再让约 30 mm 自由尾端先进入通道（约 [`route_the_cable.py:95-148`](../third_party/robosimstudio/examples/08_demos/route_the_cable/route_the_cable.py)）。这对普通结的 under-pass/退出方向是方法上的启发，不是已验证的结绳成功率。
4. 上游 `open-robot-skills` 的 `anchoring-a-free-end` 明确写出：自由端被横拉会使整条 cable 平移，先锚定一端才会弯曲；close 命令后要等待手指行程，且后续 IK 使用同一 solved chain，见 [`skills/anchoring-a-free-end/SKILL.md:40-100`](../third_party/open-robot-skills/skills/anchoring-a-free-end/SKILL.md)。
5. `seating-a-cable-crossing` 采用每次 place → settle → look，并读取真实 `seated/fraction`，不是脚本返回 done 就认为 crossing 成功，见 [`skills/seating-a-cable-crossing/SKILL.md:40-82`](../third_party/open-robot-skills/skills/seating-a-cable-crossing/SKILL.md)。它还记录任意横向加推会把 seat fraction 从 0.016 降到 0，提示修正应沿通过方向规划。

## GaP 方法的边界和当前代码的缺口

GaP 上游 roadmap 明确把 execution-feedback graph repair、bimanual manipulation、Isaac rehearsal 和非 pick-and-place 长任务列为 v1 之后内容，见 [`third_party/graph-as-policy/docs/source/developers/roadmap.md:24-50`](../third_party/graph-as-policy/docs/source/developers/roadmap.md)。因此 GaP 本身没有现成的 DLO 打结策略。

可以直接借用的是方法结构：将 `anchor → take_tail → lift → under_pass → draw_through → release` 拆成有真实后置条件的子图；每阶段记录 crossing code、材料点/TCP slip、jaw width、绳端方向、height 和 hand clearance；失败时沿 conditional edge 重感知或重抓，而不是让一个长 script 返回 `done`。当前 `graphs/thread_feedback/workflow.json` 把整段放在单一 `generic/act` script，正常出口只有 `done`，所以无法定位“抓空、穿过后结构丢失、还是释放破坏拓扑”。

## 结论：确认与假设

**已确认：** SuperDex 读取 `mu_cable`、`friction_falloff`、rod contact 参数、`self_contact` 和 `stress_free`；`scene_friction/mu_gripper/mu_surface` 属于 VBD scene-material pass，在当前 SuperDex backend 被丢弃；`feedback="off"` 不回传绳负载；`target_ke` 是机器人 finger position servo 增益；RSS cable demo 的逐步 slip、实时切线和自由端闭环方法可迁移为诊断/规划结构。

**待实验、不可当作事实：** 提高 `mu_cable` 或降低 `friction_falloff` 是否能让当前普通结穿绳成功；调整 SuperDex mirror smoothing/threshold 是否减少穿透和漂移；用 `stress_free="straight"` 是否会改善某个特定初始布局。它们都需要单独的物理 A/B，并保持严格 grader 不变。

**不应迁移：** VBD route demo 的 `scene_friction=True`、sticky contact history、`target_ke=6000` 的 16/16 routing 数据，不能直接宣称为当前 SuperDex rope 的成功抓持方案。
