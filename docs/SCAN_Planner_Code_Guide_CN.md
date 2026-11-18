# SCAN-Planner：代码结构与阅读指南

## 1. 文档定位

本文说明 SCAN-Planner 的软件结构、关键调用链、主要函数职责、论文概念与代码实现的对应关系，以及代码阅读顺序。算法模型与公式推导见[《方法原理与系统框架》](SCAN_Planner_Overview_CN.md)，轨迹执行与 Go2 仿真见[《控制器与仿真说明》](SCAN_Planner_Controller_and_Simulation_CN.md)，固定全局参考轨迹任务的执行流程见[《固定全局参考轨迹任务的执行与局部避障流程》](SCAN_Planner_Fixed_Global_Trajectory_Flow_CN.md)。

## 2. 仓库代码结构

仓库主要由 planner、simulator、tools 和 assets 四部分构成：

```text
SCAN-Planner/
├── README.md
├── LICENSE
├── assets/
│   ├── images/                     # 系统框架、实机结果等静态图像
│   └── gif/                        # 演示动画
├── tools/
│   ├── README.md
│   ├── keypoint_recorder.py        # 多关键点记录工具
│   └── keypoint.yaml               # 预设导航关键点（运行后或用户生成）
└── src/
    ├── planner/
    │   ├── plan_manage/            # ROS 节点、状态机、规划管理与控制接口
    │   ├── plan_env/               # 三维占据地图、滑动地图、膨胀与 raycasting
    │   ├── path_searching/         # 投影 A* 搜索
    │   ├── bspline_opt/            # B 样条表示、rebound 优化与求解器
    │   └── traj_utils/             # 多项式轨迹与 RViz 可视化
    └── simulator/
        ├── local_sensing/          # CPU/GPU 局部点云渲染
        ├── map_generator/          # 仿真地图生成相关节点
        ├── mockamap/               # 随机地图生成
        └── Utils/                   # waypoint、里程计显示、机器人描述等辅助包
```

### 2.1 `plan_manage`：系统编排层

该包决定何时规划、从何处起步、局部目标如何获得、结果如何发布，是理解系统数据流的首要入口。

| 文件 | 职责 | 关键内容 |
|---|---|---|
| `src/scan_planner_node.cpp` | ROS 主入口 | 创建并初始化 `SCANReplanFSM` |
| `include/plan_manage/scan_replan_fsm.h` | 状态机接口与状态数据 | 状态枚举、订阅发布器、规划入口 |
| `src/scan_replan_fsm.cpp` | 在线规划时序 | 目标处理、局部目标、重规划、安全检查、轨迹发布 |
| `include/plan_manage/planner_manager.h` | 规划管理器接口 | 地图、优化器、全局/局部轨迹数据 |
| `src/planner_manager.cpp` | 单次规划主流程 | 初值生成、高度正则、B 样条优化、可行性 refine |
| `include/plan_manage/plan_container.hpp` | 轨迹容器 | `GlobalTrajData`、`LocalTrajData` |
| `msg/Bspline.msg` | 轨迹通信消息 | 控制点、节点向量、次数、起始时间和轨迹编号 |
| `src/closed_loop_controller.cpp` | 闭环执行接口 | B 样条解析、速度前馈、位置/航向比例反馈和控制输出 |
| `src/open_loop_controller.cpp` | 开环仿真执行 | 按时间直接采样轨迹状态并发布模拟里程计 |
| `src/go2_kinematic_sim.cpp` | 简化运动学仿真 | 接收 `cmd_vel` 并更新机器人位姿 |
| `src/go2_gait_publisher.cpp` | 简化步态显示 | 根据机身运动状态生成正弦关节运动并发布 `/joint_states` |
| `launch/advanced_param.xml` | 核心参数 | 地图、机身、优化、动力学和 FSM 参数 |

### 2.2 `plan_env`：环境表示层

`GridMap` 同时承担传感器融合、栅格存储、地图滑动、增量膨胀和整机碰撞查询。核心文件为：

- `include/plan_env/grid_map.h`：数据结构、内联查询和公共接口；
- `src/grid_map.cpp`：初始化、回调、raycasting、膨胀和滑动更新；
- `include/plan_env/raycast.h`、`src/raycast.cpp`：体素射线遍历。

该模块位于感知与规划之间。上游输入是点云和位姿，下游输出不是几何点云，而是可供规划器高频查询的占据与膨胀状态。

### 2.3 `path_searching`：局部拓扑引导层

`dyn_a_star.cpp` 实现碰撞段入口到出口之间的投影 A*。其主要数据结构包括开放集、节点池、轮次标志、代价与父指针。该搜索仅服务于 B 样条 rebound 约束生成，不是系统的全局规划器。

### 2.4 `bspline_opt`：连续轨迹优化层

该包包含：

- `uniform_bspline.*`：B 样条求值、求导、参数化、可行性检查和时间延长；
- `bspline_optimizer.*`：碰撞段识别、A* 调用、rebound 几何构造、代价与梯度；
- `lbfgs.hpp`：L-BFGS 求解器实现；
- `gradient_descent_optimizer.*`：仓库保留的另一优化器实现。

不宜从 `lbfgs.hpp` 开始阅读。对于本项目，变量定义、目标函数和梯度组合比通用求解器的内部实现更为关键。

### 2.5 `traj_utils`：轨迹初始化与可视化层

`PolynomialTraj` 提供单段五次多项式和多段 minimum-snap 轨迹生成；`PlanningVisualization` 发布全局路径、初始路径、A* 路径、控制点和优化轨迹等 RViz marker。这些可视化接口对于核对规划阶段和定位失败原因具有直接价值。

---

## 3. 一次局部规划的代码调用链

从目标输入到轨迹发布，主要调用链可归纳为：

```text
scan_planner_node.cpp
└── SCANReplanFSM::init()
    ├── SCANPlannerManager::initPlanModules()
    │   ├── GridMap::initMap()
    │   ├── BsplineOptimizer::setParam()
    │   ├── BsplineOptimizer::setEnvironment()
    │   └── AStar::initGridMap()
    ├── 目标 / 路径 / 里程计订阅
    └── 定时器
        ├── SCANReplanFSM::execFSMCallback()
        │   ├── getLocalTarget()
        │   └── callReboundReplan()
        │       └── SCANPlannerManager::reboundReplan()
        │           ├── PolynomialTraj 初值或旧轨迹续接
        │           ├── applyLinearZReference()
        │           ├── UniformBspline::parameterizeToBspline()
        │           ├── BsplineOptimizer::initControlPoints()
        │           │   ├── GridMap::getInflateOccupancy()
        │           │   └── AStar::AstarSearch()
        │           ├── BsplineOptimizer::BsplineOptimizeTrajRebound()
        │           │   ├── rebound_optimize()
        │           │   ├── calcSmoothnessCost()
        │           │   ├── calcDistanceCostRebound()
        │           │   ├── calcFeasibilityCost()
        │           │   └── check_collision_and_rebound()
        │           ├── UniformBspline::checkFeasibility()
        │           ├── refineTrajAlgo()（必要时）
        │           └── updateTrajInfo()
        └── SCANReplanFSM::checkCollisionCallback()
            ├── 对待执行轨迹采样
            ├── GridMap::getInflateOccupancy()
            └── 重规划或 EMERGENCY_STOP
```

规划成功后，状态机将 `LocalTrajData::position_traj_` 的控制点和节点向量封装为 `scan_planner/Bspline`，发布至 `/planning/bspline`。控制节点解析该消息，构造位置、速度和加速度 B 样条，并按执行时间查询期望状态。

### 3.1 目标输入、全局轨迹与局部重规划

状态机并不直接将用户输入交给局部优化器，而是先将目标、路径和机器人当前状态整理为统一的规划接口。其关键函数之间的关系如下：

```text
rvizGoalCallback / pathCallback
        ↓
waypointCallback
        ↓
planGlobalTrajByWaypoints
        ↓
planGlobalTrajWaypoints
        ↓
planNextWaypoint / getLocalTarget
        ↓
callReboundReplan
        ↓
局部 B 样条轨迹发布
```

#### 目标与路径输入

现有实现中的 `SCANReplanFSM::rvizGoalCallback()` 接收单个 `geometry_msgs::PoseStamped` 目标，并将其包装为单元素 `nav_msgs::Path`，随后复用 `waypointCallback()`。在项目正式接口中，该处理逻辑应由 Action goal 回调调用；RViz 订阅仅作为现有示例入口。

`SCANReplanFSM::pathCallback()` 接收包含一个或多个位姿的 `nav_msgs::Path`。函数负责检查路径消息的有效性、提取各个位姿的位置，并将其转换为规划器使用的三维航点序列。路径中的高度信息不能被任意解释为有效机身状态：代码会结合初始高度是否已经建立以及输入高度的合法性进行筛选。对于单点目标中出现的异常负高度，例如 `z < -0.1`，函数直接拒绝该输入，用于过滤未设置高度、消息初始化异常或超出机器人工作空间的目标；该判断不承担目标修正功能。

`rviz_height_ready_` 表示系统是否已经收到可作为参考的初始机身高度。若该标志尚未建立，状态机不会接受依赖该高度的单点目标，因为此时无法可靠地确定目标高度、规划起点高度以及后续高度插值。

#### 全局航点轨迹生成

`SCANReplanFSM::planGlobalTrajByWaypoints()` 是状态机侧的封装函数。它将当前机器人状态、航点序列以及终端速度和加速度边界条件传递给规划管理器，并根据返回结果更新全局轨迹状态。该函数的职责是组织输入和处理规划结果，不负责实现 B 样条优化本身。

`SCANPlannerManager::planGlobalTrajWaypoints()` 则是规划管理器侧的多航点全局轨迹接口，其典型流程为：

1. 以当前起点位置、速度和加速度作为初始边界条件；
2. 将多个航点组织为连续的分段路径约束；
3. 根据相邻航点距离和速度限制分配各段时间；
4. 生成满足端点约束的多项式或 B 样条初始轨迹；
5. 更新 `GlobalTrajData`，供状态机提取局部目标和局部重规划参考。

`SCANPlannerManager::planGlobalTraj()` 是单次全局轨迹生成的核心管理接口，负责将起点、终点和边界状态交给轨迹初始化模块，并完成全局轨迹数据的保存。全局轨迹的作用是提供长距离任务的拓扑和时间参考；真正用于当前执行周期的局部轨迹，仍由 `reboundReplan()` 根据实时地图和局部目标重新优化得到。

#### 起始状态与局部目标推进

局部规划的起始状态包括位置、速度和加速度。`GEN_NEW_TRAJ` 通过 `SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()` 设置起始状态；`REPLAN_TRAJ` 通过 `planFromCurrentTraj()` 设置起始状态并组织重试。两个阶段均由 `callReboundReplan()` 调用局部规划器并发布轨迹。

`setStartStateFromOdomOrCurrentTraj()` 始终采用当前里程计位置作为起点，默认采用里程计速度和零加速度。若已有局部轨迹的起始时间、持续时间及当前执行时间满足有效性检查，则以该轨迹当前执行时刻的速度和加速度替换默认值；否则保留里程计边界。采用轨迹导数后，若水平速度与当前位置指向目标的方向点积为负，则将速度和加速度清零。该函数用于首次局部规划，也适用于切换航点或急停恢复等重新进入 `GEN_NEW_TRAJ` 的情形。

`planFromCurrentTraj()` 将执行时间裁剪至当前局部轨迹的时间区间，并据此提取起始状态。在 `REFERENCE_PATH` 模式下，位置、速度和加速度均取自该轨迹；在 RViz 单点目标等非参考路径模式下，位置取自当前里程计，速度和加速度取自该轨迹，水平速度背离目标时将速度和加速度清零。非参考路径模式随后刷新全局参考轨迹，完成目标占据调整后再生成局部轨迹。

非参考路径模式下，两个入口在旧轨迹有效时采用相同的起始状态选取规则，但仅 `setStartStateFromOdomOrCurrentTraj()` 提供旧轨迹有效性检查及里程计回退处理。

`SCANReplanFSM::planNextWaypoint()` 负责根据当前机器人位置和全局航点进度选择下一个局部目标。目标并非始终等于长距离任务的最终终点，而是从全局航点序列中选择当前可执行的前视目标，使局部规划器能够在有限感知范围内逐段推进。达到当前航点后，状态机再切换到后续航点，直至全局任务完成。

#### 局部轨迹初值与失败重试

`GEN_NEW_TRAJ` 首次进入时，`flag_random_poly_init` 为 `false`，规划器根据起始状态和局部目标状态生成单段五次多项式初始轨迹。多项式轨迹经采样后，结合端点导数条件参数化为 B 样条，得到初始控制点。该阶段不施加碰撞约束，轨迹安全性与动力学可行性由后续绕障搜索、B 样条优化及检查流程处理。

若 `reboundReplan()` 未能得到可行轨迹，状态机再次进入 `GEN_NEW_TRAJ`，并将 `flag_random_poly_init` 设为 `true`。规划器在起点与局部目标的中点附近加入随机偏移点，生成 minimum-snap 初始轨迹，以不同初值重新求解。

#### 目标占据调整与实时安全处理

`SCANReplanFSM::adjustGlobalTargetIfOccupied()` 用于处理目标点位于占据区域或膨胀障碍物中的情况。它通常在建立全局任务目标时进行检查，并在目标附近搜索可接受的替代位置，以避免将明显不可执行的目标直接传递给规划器。该函数解决的是“目标点本身无效”的问题，不等同于整个执行过程中的持续避障。

对于距离机器人很远、尚未进入局部感知范围的目标，初始检查只能依赖当前可用的全局地图或已有地图数据；当机器人逐渐接近目标后，新的点云会更新 `GridMap`，局部规划器和 `checkCollisionCallback()` 会对待执行轨迹重新进行碰撞检查。如果发现原轨迹进入新观测到的障碍物区域，状态机将触发局部重规划；若无法在限定次数或时间内生成安全轨迹，则转入相应的失败处理或紧急停止状态。因此，目标占据调整通常是任务初始化阶段的处理，而执行期安全性依赖持续的地图更新、轨迹碰撞检查和滚动重规划共同保证。

### 3.2 状态机中的规划状态

`GEN_NEW_TRAJ` 与 `REPLAN_TRAJ` 都会调用局部轨迹规划，但触发语义不同：

| 状态 | 进入条件 | 主要任务 | 规划起点 |
|---|---|---|---|
| `GEN_NEW_TRAJ` | 尚未建立可执行轨迹，或任务刚开始 | 首次生成当前任务的局部轨迹 | 初始里程计状态或任务起始状态 |
| `REPLAN_TRAJ` | 当前轨迹即将结束、发生碰撞风险或局部环境发生变化 | 在已有任务上下文中滚动更新轨迹 | 当前里程计状态或当前轨迹采样状态 |

两者并不代表两种不同的优化算法，而是表示轨迹生成的时序语义不同：`GEN_NEW_TRAJ` 解决“从无到有”，`REPLAN_TRAJ` 解决“执行过程中根据最新状态和地图更新”。具体规划仍由相同的局部 B 样条、碰撞约束和可行性检查流程完成。

## 4. 论文概念与代码实现对照

| 论文概念 | 主要实现位置 | 说明 |
|---|---|---|
| 三次均匀 B 样条 | `bspline_opt/uniform_bspline.*` | 求值、求导、参数化和可行性检查 |
| 航向由轨迹切线诱导 | `bspline_optimizer.cpp`、`scan_replan_fsm.cpp` | 控制点或轨迹采样段方向估计 yaw |
| 双圆柱 footprint | `GridMap::getInflateOccupancy(pos, yaw)` | 前后两个偏移中心查询膨胀栅格 |
| 非对称三维膨胀 | `GridMap::initMap()`、`updateInflation()` | 水平、向上、向下膨胀范围分别配置 |
| 高度正则化初始化 | `applyLinearZReference()` | 按累计水平弧长线性插值高度 |
| 碰撞段分割 | `BsplineOptimizer::initControlPoints()` | 线段稠密采样并归并连续碰撞区间 |
| projected A* | `AStar::AstarSearch()` | 八邻域水平扩展，z 由端点高度插值 |
| rebound anchor/direction | `initControlPoints()`、`check_collision_and_rebound()` | 从 A* 路径交点构造连续优化方向 |
| 碰撞代价 | `calcDistanceCostRebound()` | 对安全间距不足的控制点施加惩罚 |
| jerk 平滑代价 | `calcSmoothnessCost()` | 控制高阶差分 |
| 速度/加速度可行性 | `calcFeasibilityCost()`、`checkFeasibility()` | 软惩罚加显式检查 |
| 机器人中心滑动地图 | `GridMap::updateSlidingMap()` | 固定容量环形存储与地址复用 |
| 在线安全监测 | `SCANReplanFSM::checkCollisionCallback()` | 沿待执行轨迹进行双圆柱检查 |
| 闭环轨迹跟踪 | `closed_loop_controller.cpp` | B 样条速度前馈与位置/航向比例反馈 |
| 开环轨迹回放 | `open_loop_controller.cpp` | 按轨迹时间发布位置、速度、加速度和航向状态 |
| 边界回退 | 当前版本未见完整同构实现 | 论文机制与开源代码需区分 |

---

## 5. 推荐代码阅读路线

### 5.1 第一阶段：建立系统全貌

建议依次阅读：

1. `README.md`：确认系统用途、运行模式和软件依赖；
2. `plan_manage/launch/run.launch`：确认启动节点、话题和模式选择；
3. `plan_manage/launch/advanced_param.xml`：建立参数与论文符号之间的对应；
4. `scan_planner_node.cpp`：确定程序入口；
5. `scan_replan_fsm.h/.cpp`：掌握状态机、目标来源和重规划时机。

该阶段的目标是回答：系统何时开始规划、局部目标从何而来、什么时候重新规划、轨迹如何发布。

### 5.2 第二阶段：掌握单次规划主线

按以下顺序阅读：

1. `planner_manager.h` 中的成员和参数结构；
2. `SCANPlannerManager::initPlanModules()`；
3. `SCANPlannerManager::reboundReplan()`；
4. `plan_container.hpp` 中的 `GlobalTrajData` 与 `LocalTrajData`；
5. `PolynomialTraj::one_segment_traj_gen()` 与 `minSnapTraj()`；
6. `UniformBspline::parameterizeToBspline()`。

这一阶段应重点跟踪以下变量：

```text
start_pt / start_vel / start_acc
local_target_pt / local_target_vel
point_set
start_end_derivatives
ctrl_pts
ts
local_data_.position_traj_
```

### 5.3 第三阶段：理解整机碰撞与地图

阅读顺序为：

1. `grid_map.h` 中的 `MappingParameters` 与 `MappingData`；
2. `GridMap::initMap()`；
3. `GridMap::getInflateOccupancy(pos, yaw)`；
4. 原始 occupancy 更新和 `updateInflation()`；
5. `GridMap::updateSlidingMap()`；
6. `raycastProcess()` 与 `raycast.cpp`。

建议先理解查询接口和数据布局，再进入传感器融合细节。否则容易在较长的地图实现中失去主线。

### 5.4 第四阶段：理解 A* 与 rebound 约束

推荐顺序：

1. `BsplineOptimizer::initControlPoints()`；
2. `AStar::ConvertToIndexAndAdjustStartEndPoints()`；
3. `AStar::AstarSearch()`；
4. `AStar::getPath()`；
5. 回到 `initControlPoints()`，继续阅读 A* 路径到 `base_point/direction` 的转换；
6. `BsplineOptimizer::check_collision_and_rebound()`。

这一阶段应明确三类几何对象之间的关系：原始控制点、无碰撞 A* 路径和障碍边界基准点。

### 5.5 第五阶段：理解优化目标和求解过程

依次阅读：

1. `BsplineOptimizeTrajRebound()`；
2. `rebound_optimize()`；
3. `combineCostRebound()`；
4. `calcDistanceCostRebound()`；
5. `calcSmoothnessCost()`；
6. `calcFeasibilityCost()`；
7. `earlyExit()` 与优化后碰撞复查；
8. `refineTrajAlgo()` 与 `BsplineOptimizeTrajRefine()`。

只有在需要修改求解器行为时，才有必要继续阅读 `lbfgs.hpp`。对于理解 SCAN-Planner 本身，代价构造和终止逻辑更为关键。

### 5.6 第六阶段：闭合执行链

最后阅读：

1. `callReboundReplan()` 中的 B 样条消息封装；
2. `closed_loop_controller.cpp` 的 `bsplineCallback()`；
3. B 样条位置、速度和加速度的实时求值；
4. `checkCollisionCallback()` 的执行期安全监督；
5. `planning_visualization.cpp` 中各类 RViz marker 的含义。

至此可以完整解释一次规划周期从输入、地图、搜索、优化到控制输出的全过程。

---

## 6. 关键参数及其物理意义

### 6.1 机器人与地图参数

| 参数类别 | 代表参数 | 主要影响 |
|---|---|---|
| 地图分辨率 | `grid_map/resolution` | 碰撞精度、A* 粒度、内存与计算量 |
| 地图窗口 | `sliding_map_size_x/y/z` | 局部视野范围与内存规模 |
| 滑动阈值 | `map_sliding_thresh` | 地图窗口移动频率 |
| 水平膨胀 | `double_cylinder_radius` | 双圆柱横向安全间隙 |
| 纵向偏移 | `double_cylinder_offset` | 前后圆柱中心距离机器人中心的偏移 |
| 垂向膨胀 | `obstacles_inflation_z_up/down` | 机身高度、跨越能力与悬空障碍判定 |

### 6.2 轨迹与优化参数

| 参数类别 | 代表参数 | 主要影响 |
|---|---|---|
| 控制点间距 | `manager/control_points_distance` | 轨迹自由度和优化规模 |
| 规划视距 | `fsm/planning_horizon`、`manager/planning_horizon` | 局部目标距离和局部问题规模 |
| 安全距离 | `optimization/dist0` | rebound 约束要求的障碍间隙 |
| 平滑权重 | `lambda_smooth` | 轨迹曲折与 jerk |
| 碰撞权重 | `lambda_collision` | 远离障碍的强度 |
| 可行性权重 | `lambda_feasibility` | 速度、加速度超限惩罚 |
| 最大速度/加速度 | `max_vel`、`max_acc` | 时间参数化与执行能力 |

这些参数相互耦合。例如，提高地图分辨率而不调整控制点间距，会改变每段控制点间的采样密度；增大双圆柱半径后，`dist0` 的额外安全裕度是否仍合理也应重新评估。参数整定不应被视为相互独立的一维调节。
