# 固定全局参考轨迹任务的执行与局部避障流程

## 1. 文档定位

本文面向如下任务模式：上层导航模块接收单次目标请求，调用全局路网规划生成全局路径；任务执行期间，全局路径保持不变，SCAN-Planner 沿该路径推进，并依据实时环境信息生成局部无碰撞轨迹。

任务目标通过上层导航 Action 接口传入。外部不直接发布 `/initial_path`，而是通过 Action 提交单次任务目标，由状态机调用 `planGlobalTraj()` 建立全局参考轨迹。仓库中的 `rvizGoalCallback()` 仅用于现有单点目标示例，不作为项目正式通信接口。

本文区分以下两种状态：

- 目标点不变：任务终点 `end_pt_` 不变；
- 全局轨迹不变：保存的全局参考轨迹对象在任务期间不重新生成。

现有目标点模式的 `planFromCurrentTraj()` 仍会调用 `planGlobalTraj()`。因此，当前实现保持的是目标点不变，而非全局轨迹对象严格不变。下文分别说明项目所需的固定全局参考轨迹流程及其与现有实现之间的差异。

## 2. 项目任务分层

```text
一次任务目标点
        ↓
外部全局路网规划
        ↓
固定的全局路径/全局参考轨迹
        ↓
沿全局轨迹选取有限视距局部目标
        ↓
生成局部 B 样条轨迹
        ↓
控制器执行
        ↓
实时地图更新与碰撞检查
        ↓
局部重规划绕障
```

| 层级 | 职责 | 任务期间是否改变 |
|---|---|---:|
| 全局路网规划 | 提供总体路线和拓扑约束 | 否 |
| 全局参考轨迹 | 将路线组织为连续参考 | 原则上不变 |
| 局部规划 | 在有限感知范围内绕开障碍物 | 是 |
| 控制器 | 跟踪当前局部 B 样条 | 持续执行 |

局部重规划不改变全局路线，仅在其邻域内修正当前局部段，并在障碍物通过后恢复对全局参考方向的跟踪。

## 3. 初始化与目标输入

主节点 `scan_planner_node.cpp` 的流程为：

```cpp
SCANReplanFSM scan_replan;
scan_replan.init(nh);
ros::spin();
```

`SCANReplanFSM::init()` 将 FSM 设置为 `INIT`，初始化规划管理器、地图、B 样条优化器和 A* 搜索器，并创建：

```cpp
execFSMCallback()        // 0.01 s，100 Hz，FSM 主循环
checkCollisionCallback() // 0.05 s，20 Hz，执行期碰撞检查
```

项目节点应注册目标 Action Server。Action goal 回调负责接收任务目标，cancel/preempt 回调负责取消当前任务。现有代码保留 RViz 示例入口：

```text
/move_base_simple/goal
```

现有代码尚未包含该任务对应的 Action Server 实现。项目实现应增加 Action 接口，并将 goal/cancel 回调接入 FSM。正式任务目标及其约束以 Action goal 为准。初始里程计到达后，`odometryCallback()` 更新：

```text
odom_pos_、odom_vel_、odom_orient_、have_odom_
```

并在手动目标模式下设置：

```cpp
rviz_goal_height_ = odom_pos_(2);
rviz_height_ready_ = true;
```

因此，Action goal 中的目标 (x,y) 来自上层导航请求，目标高度采用初始机身高度。初始里程计未到达前，目标回调应拒绝该任务。

任务采用互斥执行策略：当前任务处于执行、重规划或安全停止流程时，新的 Action goal 不得覆盖当前任务，也不得触发 `REPLAN_TRAJ`。仅当当前任务完成，或上层取消请求已完成安全停止流程后，系统才接受下一项任务。

## 4. Action 目标到达后的调用链

```text
Action goal
        ↓
Action goal callback
        ↓  提取目标点并转换为内部统一格式
waypointCallback() / 项目目标处理函数
        ↓
planGlobalTraj()
        ↓
adjustGlobalTargetIfOccupied()
        ↓
GEN_NEW_TRAJ
        ↓
callReboundReplan()
        ↓
getLocalTarget()
        ↓
reboundReplan()
        ↓
发布第一条局部 B 样条
        ↓
EXEC_TRAJ
```

### 4.1 Action goal 回调

Action goal 回调负责接收并校验任务目标，并复用统一的航点处理接口。若内部接口继续采用 `nav_msgs::Path`，可构造单元素路径：

```cpp
nav_msgs::PathPtr path(new nav_msgs::Path);
path->header = msg->header;
path->poses.push_back(goal_pose);
waypointCallback(path);
```

它不直接执行局部规划，也不直接驱动 FSM 主循环。ROS 通过回调队列分发 Action 回调和定时器回调；状态机的周期性推进仍由 `execFSMCallback()` 完成。

### 4.2 `waypointCallback()`

在单目标 Action 模式下，统一目标处理函数：

1. 检查消息指针和 `poses` 容器；
2. 过滤异常的目标高度；
3. 设置 `trigger_ = true`，记录收到目标时的位置；
4. 使用目标 (x,y) 和 `rviz_goal_height_` 构造 `end_pt_`；
5. 调用 `SCANPlannerManager::planGlobalTraj()` 生成全局参考轨迹；
6. 调用 `adjustGlobalTargetIfOccupied()` 检查终点；
7. 设置 `have_target_ = true` 和 `have_new_target_ = true`；
8. 根据当前 FSM 状态触发首次规划或在线重规划。

该全局轨迹用于路线引导和局部目标选取，不直接作为发送给底层控制器的局部避障轨迹。

## 5. 目标输入后的状态切换

### 5.1 目标在 `INIT` 阶段到达

如果目标到达时 FSM 仍处于 `INIT`，`waypointCallback()` 会设置目标标志并生成全局轨迹，但不会直接切换到 `GEN_NEW_TRAJ`，因为当前状态不是 `WAIT_TARGET` 或 `EXEC_TRAJ`。

之后由 `execFSMCallback()` 推进：

```text
INIT
  ↓ have_odom_ && trigger_
WAIT_TARGET
  ↓ have_target_
GEN_NEW_TRAJ
```

### 5.2 目标在 `WAIT_TARGET` 阶段到达

如果目标到达时 FSM 已经处于 `WAIT_TARGET`，`waypointCallback()` 会直接执行：

```cpp
changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
```

状态转移为：

```text
WAIT_TARGET → GEN_NEW_TRAJ
```

### 5.3 执行期间收到新目标

任务执行期间，Action Server 在 `EXEC_TRAJ`、`REPLAN_TRAJ` 或局部规划失败重试阶段收到新的 goal 时，应拒绝该请求，并返回“已有任务正在执行”的结果：

```text
新 Action goal → 拒绝
当前任务继续执行
```

系统仅在当前任务成功到达终点，或取消请求完成安全停止并返回 canceled 结果后接受下一目标。

## 6. Action 任务生命周期与取消

Action 层负责目标接受、目标拒绝、取消请求、反馈和最终结果；FSM 负责轨迹生成、执行、局部重规划和安全停止。两者应保持职责分离：

```text
Action goal 到达
    ├── 当前无活动任务 → 接受并启动规划
    └── 当前已有任务 → 拒绝

当前任务执行
    ├── 持续发布执行反馈
    ├── 收到 cancel → 执行安全停止
    ├── 到达目标 → 返回 succeeded
    └── 无法安全规划 → 返回 aborted

cancel 完成
    ↓
清理当前任务状态并回到 WAIT_TARGET
    ↓
允许接受下一任务
```

取消当前任务不能仅将 `have_target_` 设置为 `false`，还需明确处理：

1. 是否发布静止轨迹或进入 `EMERGENCY_STOP`；
2. 是否等待机身速度下降到安全阈值；
3. 当前 Action 是否返回 canceled 结果；
4. 全局参考轨迹、局部轨迹和任务标志是否清理；
5. 何时重新允许 Action Server 接受新的 goal。

## 7. 首次局部规划：`GEN_NEW_TRAJ`

在该任务模式中，`GEN_NEW_TRAJ` 仅负责任务启动阶段的首次局部轨迹生成，在建立第一条局部 B 样条之前不可省略。

进入该状态后，`execFSMCallback()` 调用：

```cpp
setStartStateFromOdomOrCurrentTraj();
callReboundReplan(true, flag_random_poly_init);
```

首次规划通常使用当前里程计建立规划边界。局部规划过程包括：

1. `getLocalTarget()`：从全局参考轨迹选取有限视距目标；
2. `reboundReplan()`：生成初始轨迹并执行局部优化；
3. 地图碰撞查询和碰撞段检测；
4. projected A* 搜索和 rebound 几何约束构造；
5. 平滑性、碰撞安全和动力学可行性优化；
6. 可行性检查并发布 `/planning/bspline`。

规划成功后：

```text
GEN_NEW_TRAJ → EXEC_TRAJ
```

如果失败，则在 `GEN_NEW_TRAJ` 中重试；连续失败超过阈值后进入 `EMERGENCY_STOP`。

## 8. 正常执行：`EXEC_TRAJ`

第一条局部 B 样条发布后，控制器开始执行，FSM 进入 `EXEC_TRAJ`。此时：

- 全局参考轨迹用于路线推进和局部目标选择；
- 局部 B 样条用于当前有限范围内的实际运动；
- `execFSMCallback()` 检查执行进度和重规划时机；
- `checkCollisionCallback()` 检查新障碍物是否侵入待执行轨迹。

固定全局轨迹任务的运行期主链为：

```text
EXEC_TRAJ
   ├── 达到滚动重规划条件 → REPLAN_TRAJ
   ├── 发现新障碍物 → 局部重规划或 EMERGENCY_STOP
   └── 到达任务终点 → WAIT_TARGET / 任务结束
```

## 9. 运行期局部重规划：`REPLAN_TRAJ`

### 9.1 由执行进度触发

当当前局部轨迹执行到需要更新的位置，`execFSMCallback()` 将状态切换为：

```text
EXEC_TRAJ → REPLAN_TRAJ
```

随后调用 `planFromCurrentTraj()`，从当前局部 B 样条中提取当前时刻的位置、速度和加速度，使新轨迹能够与当前运动连续衔接。

### 9.2 由新障碍物触发

`checkCollisionCallback()` 沿剩余局部轨迹采样，并调用：

```cpp
map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next));
```

如果发现碰撞，会首先尝试：

```cpp
planFromCurrentTraj();
```

结果处理为：

```text
重规划成功 → EXEC_TRAJ
重规划失败且障碍物较远 → REPLAN_TRAJ，继续尝试
重规划失败且障碍物过近 → EMERGENCY_STOP
```

### 9.3 固定全局轨迹下的理想逻辑

```text
固定全局参考轨迹
        ↓
沿全局轨迹选取局部目标
        ↓
从当前执行状态建立局部规划起点
        ↓
使用最新局部地图生成局部 B 样条
        ↓
绕开障碍物并回到全局路线附近
        ↓
继续执行
```

此处的重规划仅更新局部 B 样条，不重新运行外部全局路网规划。

## 10. 当前实现与固定全局轨迹要求的差异

当前 `planFromCurrentTraj()` 在 `REFERENCE_PATH` 模式下主要复用当前局部轨迹；但在目标点模式下，会执行：

```cpp
planner_manager_->planGlobalTraj(
    start_pt_, start_vel_, start_acc_, end_pt_, ...);
```

因此，当前目标点模式的实际行为更接近：

```text
目标点固定
    ↓
局部障碍变化
    ↓
重新生成当前位置到目标点的全局参考轨迹
    ↓
重新生成局部 B 样条
```

而项目要求的是：

```text
全局轨迹生成一次
    ↓
保存全局轨迹
    ↓
后续只更新局部目标和局部 B 样条
    ↓
不重新调用全局路网规划
```

如果“全局路径不变”仅表示终点和总体任务路线不变，现有代码可以继续使用；如果要求全局轨迹的几何形状和参数化对象严格不变，则项目实现应满足：

1. 任务开始时调用一次 `planGlobalTraj()`；
2. 后续 `REPLAN_TRAJ` 不再调用全局规划；
3. 从固定全局轨迹中调用 `getLocalTarget()` 选取局部目标；
4. 只使用当前局部状态和最新地图调用 `reboundReplan()`；
5. 将局部重规划限制在全局路线允许的偏离范围内。

## 11. 项目实现中的推荐函数流程

### 11.1 任务启动

```text
scan_planner_node.cpp
    ↓
SCANReplanFSM::init()
    ↓
odometryCallback()
    ↓
Action goal callback
    ↓
waypointCallback()
    ↓
SCANPlannerManager::planGlobalTraj()   // 任务期间原则上只调用一次
    ↓
adjustGlobalTargetIfOccupied()
    ↓
GEN_NEW_TRAJ
    ↓
setStartStateFromOdomOrCurrentTraj()
    ↓
callReboundReplan()
    ↓
getLocalTarget()
    ↓
SCANPlannerManager::reboundReplan()
    ↓
发布第一条局部 B 样条
```

### 11.2 正常执行和局部避障

```text
EXEC_TRAJ
    ↓
控制器跟踪当前局部 B 样条
    ↓
execFSMCallback() 检查执行进度
    ↓
满足条件后进入 REPLAN_TRAJ
    ↓
planFromCurrentTraj()
    ↓
从固定全局轨迹选取局部目标
    ↓
reboundReplan()
    ↓
发布新的局部 B 样条
    ↓
EXEC_TRAJ
```

## 12. 项目实现约束

1. 外部全局路网输出是离散路径还是连续参考轨迹；
2. 全局参考轨迹是否只允许在任务开始时生成一次；
3. `getLocalTarget()` 是否始终从同一条全局轨迹取点；
4. `REPLAN_TRAJ` 是否会意外修改全局轨迹数据；
5. 局部轨迹允许偏离全局路线的最大横向距离；
6. 局部重规划失败时是继续尝试、停止，还是请求上层重新规划；
7. 到达目标后是回到 `WAIT_TARGET`，还是向上层发布任务完成事件。

## 13. 总结

对于本项目，推荐将完整任务理解为：

```text
一次目标点
    ↓
全局轨迹生成一次
    ↓
GEN_NEW_TRAJ：首次局部规划
    ↓
EXEC_TRAJ：沿全局路线执行
    ↕
REPLAN_TRAJ：基于实时障碍物进行局部避障
    ↓
到达目标或安全停止
```

任务开始后，运行期的核心确实是 `EXEC_TRAJ ↔ REPLAN_TRAJ`；但 `GEN_NEW_TRAJ` 仍必须保留，用于从固定全局轨迹建立第一条可执行的局部 B 样条。若要求全局轨迹严格不变，则必须进一步处理当前目标点模式下 `planFromCurrentTraj()` 对 `planGlobalTraj()` 的再次调用。
