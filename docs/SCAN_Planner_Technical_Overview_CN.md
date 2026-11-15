# SCAN-Planner：方法原理、系统实现与代码结构

## 摘要

SCAN-Planner 是面向四足机器人长距离导航的空间碰撞感知局部规划系统。其研究目标是在有限计算资源和局部感知范围内，统一处理狭窄通道中的整机碰撞、悬空结构与楼梯等三维环境、连续平滑轨迹生成，以及全局粗路线引导下的在线重规划。系统以机器人中心三维占据栅格为环境表示，以航向感知的双圆柱模型近似四足机器人躯干，以投影 A* 搜索生成局部绕障引导，并通过均匀 B 样条优化获得满足平滑性、碰撞安全性和动力学可行性的连续轨迹。

本文结合原论文与当前开源仓库，对 SCAN-Planner 的问题定义、数学模型、算法流程、软件架构、运行时数据流和核心源文件作系统说明，并给出适合初次接触该项目的代码阅读路线。本文侧重解释“系统为何采用这一设计、论文概念如何映射到代码，以及各模块如何协同”，而非提供编译安装教程。

> **版本说明**：论文描述的是完整方法框架，仓库则是该框架的具体开源实现。二者在核心设计上保持一致，但并非论文中的每一项机制都能在当前代码版本中找到完全同构的独立实现。本文将对论文机制与代码事实作必要区分。

---

## 1. 研究问题与设计目标

四足机器人局部规划不同于典型的二维移动机器人规划，也不同于面向无人机的自由空间三维规划。其主要困难来自以下四个方面。

### 1.1 航向相关的整机碰撞

四足机器人躯干通常具有明显的纵向尺度，其平面投影不是各向同性的。若以圆或单一膨胀半径近似机器人，则存在两种相反的误差：

1. 采用较小半径时，机器人中心可以通过，但躯干前后端仍可能与障碍物碰撞；
2. 采用覆盖机身长度的大半径时，狭窄但实际可通行的区域会被错误排除。

因此，碰撞区域应随机器人航向角变化。SCAN-Planner 的目标是在不显式扩充航向状态维度的前提下，使局部轨迹优化仍具备航向相关的整机碰撞判定能力。

### 1.2 地面约束下的三维环境推理

二维占据地图不能表达楼梯、桌面、货架和多层结构；单值高程图又难以同时描述地面和悬空障碍。完整三维占据地图能够保留这些结构，但若直接沿用无人机式三维规划，轨迹可能通过抬升或下沉绕过障碍，不符合接触支撑式运动的物理含义。

为此，规划器既要保留三维碰撞推理能力，又应保持地面跟随的垂向趋势，使绕障主要发生在水平方向。

### 1.3 连续、平滑且动力学可行的轨迹

离散图搜索可以提供无碰撞路径，但其输出通常包含折线、不连续曲率和栅格化误差，不能直接作为四足机器人控制器的参考轨迹。局部规划器需要进一步生成连续时间轨迹，并约束速度、加速度与高阶平滑性。

### 1.4 局部地图上的长距离导航

高分辨率三维地图若随运行区域无限增长，将带来不可接受的存储和更新开销。SCAN-Planner 因而只维护机器人附近的固定尺寸滑动地图，并由全局粗路线持续提供局部目标。该设计要求规划器能够处理地图窗口移动、局部目标不可达和在线重规划等问题。

---

## 2. 方法总体框架

SCAN-Planner 接收点云、机器人位姿或里程计，以及全局路线或导航目标。系统的主要处理链如下：

```text
点云 / 深度数据 + 机器人状态 + 全局目标或参考路线
                         │
                         ▼
              机器人中心三维滑动占据地图
                         │
                         ▼
              非对称三维障碍膨胀表示
                         │
                         ▼
              全局参考轨迹与局部目标选取
                         │
                         ▼
         多项式或上一周期轨迹产生局部初始路径
                         │
                         ▼
        高度正则化与均匀三次 B 样条参数化
                         │
                         ▼
       航向感知双圆柱碰撞检测与碰撞段分割
                         │
                         ▼
          投影 A* 搜索与 rebound 约束构造
                         │
                         ▼
       平滑性 + 碰撞安全 + 动力学可行性优化
                         │
                         ▼
               安全检查、时间重分配与发布
                         │
                         ▼
                   四足机器人控制器
```

该系统不是以 A* 搜索结果直接控制机器人。A* 只负责为发生碰撞的局部轨迹段提供拓扑和几何引导；最终执行轨迹由 B 样条优化器生成。图搜索与连续优化的职责分别是：

- **图搜索**：确定从障碍物哪一侧绕行；
- **连续优化**：在该拓扑选择下生成平滑、安全且动力学可行的轨迹。

---

## 3. 轨迹表示与航向建模

### 3.1 均匀 B 样条位置轨迹

规划器只显式优化位置轨迹

$$
\mathbf p(t)\in\mathbb R^3,
$$

并以次数为 $p_b$、节点间隔为 $\Delta t$ 的均匀 B 样条表示：

$$
\mathbf p(t)=\sum_{i=0}^{N_c-1}\mathbf Q_i B_{i,p_b}(t),
$$

其中 $\mathbf Q_i\in\mathbb R^3$ 为控制点，$B_{i,p_b}(t)$ 为 B 样条基函数。论文与代码均采用三次 B 样条，即 $p_b=3$。轨迹时长为

$$
T=(N_c-p_b)\Delta t.
$$

均匀 B 样条的导数仍可表示为 B 样条，其控制点可由相邻位置控制点差分得到。按照论文中的简化记号：

$$
\mathbf V_i=\frac{\mathbf Q_{i+1}-\mathbf Q_i}{\Delta t},\qquad
\mathbf A_i=\frac{\mathbf V_{i+1}-\mathbf V_i}{\Delta t},\qquad
\mathbf J_i=\frac{\mathbf A_{i+1}-\mathbf A_i}{\Delta t}.
$$

代码中的严格导数计算还会包含样条次数和节点向量所引入的比例因子，相关逻辑位于 `UniformBspline::getDerivativeControlPoints()`、`getDerivative()` 和 `checkFeasibility()`。

### 3.2 由轨迹切线诱导航向

SCAN-Planner 不将航向角作为独立优化变量，而是由位置轨迹的局部切线估计：

$$
\psi_i=\operatorname{atan2}\!\left(
\mathbf e_y^\top(\mathbf Q_{i+1}-\mathbf Q_{i-1}),
\mathbf e_x^\top(\mathbf Q_{i+1}-\mathbf Q_{i-1})
\right),
$$

其中 $\mathbf e_x=[1,0,0]^\top$、$\mathbf e_y=[0,1,0]^\top$。该耦合方式具有两项意义：

1. 避免在优化变量中额外引入航向控制点，控制问题规模；
2. 使机器人躯干方向与轨迹前进方向一致，适合在狭窄通道中减少横向占用。

代码中的对应接口为：

- `BsplineOptimizer::estimateSegmentYaw()`；
- `BsplineOptimizer::estimateControlPointYaw()`；
- `SCANReplanFSM::estimateYawFromSegment()`；
- A* 邻居扩展中的 `atan2(dy, dx)`。

需要注意，该模型默认机器人在执行时大体沿轨迹切向定向。若实际控制系统允许显著侧向运动，或机器人朝向与速度方向长期解耦，则该航向近似需要重新审视。

---

## 4. 航向感知的双圆柱整机模型

### 4.1 几何近似

机器人躯干由两个竖直圆柱近似。两个圆柱中心在机体坐标系中沿纵轴对称分布：

$$
\mathbf s_{b,1}=[d_{\mathrm{off}},0,0]^\top,\qquad
\mathbf s_{b,2}=[-d_{\mathrm{off}},0,0]^\top,
$$

其中 $d_{\mathrm{off}}$ 为圆柱中心相对于机器人中心的纵向偏移。给定轨迹检查点 $\mathbf Q_k$ 和诱导航向 $\psi_k$，圆柱中心在世界坐标系中的位置为

$$
\mathbf s_{k,j}=\mathbf R_z(\psi_k)\mathbf s_{b,j}+\mathbf Q_k,
\qquad j\in\{1,2\}.
$$

这种近似比单圆模型更能反映细长机身的方向性，同时又显著低于多面体整机碰撞检测的计算复杂度。

### 4.2 由地图膨胀转化为稀疏点查询

地图预先按照圆柱半径和垂向间隙进行膨胀。完成膨胀后，圆柱与原始障碍物的体积相交问题可转化为圆柱中心在膨胀地图中的点查询：

$$
H_k(\mathbf Q_k,\psi_k)
=\max_{j\in\{1,2\}}\chi(\mathbf s_{k,j}),
$$

其中 $\chi(\cdot)$ 表示膨胀占据状态。若 $H_k=0$，则该检查点处的双圆柱模型无碰撞。

仓库在 `GridMap::getInflateOccupancy(pos, yaw)` 中直接计算前后圆柱中心：

```cpp
heading = [cos(yaw), sin(yaw), 0];
front = pos + offset * heading;
rear  = pos - offset * heading;
```

随后查询 `occupancy_buffer_inflate_`。因此，代码中的 `double_cylinder_radius` 主要体现在障碍膨胀核中，`double_cylinder_offset` 则体现在两个查询点的位置变换中。

### 4.3 非对称垂向膨胀

地图参数分别设置向上和向下膨胀范围，而非采用垂向对称半径。其物理含义是：

- 向上间隙覆盖机器人躯干及传感器的最大高度；
- 向下间隙结合机体中心高度和最大可跨越台阶高度设定；
- 较低且可跨越的障碍不必等价为完整阻挡；
- 悬空结构只有在侵入机体高度范围时才构成碰撞。

相关参数集中在 `advanced_param.xml` 的 `grid_map` 命名空间中。

---

## 5. 初始轨迹与高度正则化

### 5.1 初始轨迹来源

局部规划的初始路径有两类来源：

1. **多项式初始化**：首次规划、新目标到达或需要强制重置时，由起点状态和局部目标状态生成单段五次多项式或最小 snap 轨迹；
2. **上一周期轨迹初始化**：重规划时保留尚未执行的旧轨迹片段，再以多项式延伸到新的局部目标，以提高轨迹连续性。

这两条路径均在 `SCANPlannerManager::reboundReplan()` 中组织，底层多项式生成位于 `PolynomialTraj::one_segment_traj_gen()` 与 `PolynomialTraj::minSnapTraj()`。

### 5.2 基于水平弧长的高度正则化

设初始路径采样点为

$$
\mathcal R=\{\mathbf r_0,\mathbf r_1,\ldots,\mathbf r_{N_p}\}.
$$

首先计算其水平累计弧长：

$$
\ell_i=\sum_{m=1}^{i}
\left\|\mathbf r_{m,xy}-\mathbf r_{m-1,xy}\right\|,
\qquad L=\ell_{N_p}.
$$

然后以水平进度线性插值高度：

$$
z_i=z_s+\alpha_i(z_g-z_s),
$$

$$
\alpha_i=
\begin{cases}
\ell_i/L,&L>0,\\
i/N_p,&L=0.
\end{cases}
$$

该操作保留初始路径的 $xy$ 投影，同时赋予稳定的垂向参考。代码对应 `planner_manager.cpp` 文件内的 `applyLinearZReference()`。其目的并不是恢复精确地形表面，而是在进入 rebound 优化之前抑制无意义的垂向振荡。

### 5.3 B 样条参数化

完成采样和高度正则化后，系统调用

```cpp
UniformBspline::parameterizeToBspline(...)
```

根据路径点以及首尾速度、加速度边界条件求解三次均匀 B 样条控制点。此处得到的是优化初值，而非最终执行轨迹。

---

## 6. 碰撞段检测、投影 A* 与 rebound 约束

### 6.1 碰撞段分割

`BsplineOptimizer::initControlPoints()` 沿相邻控制点之间的线段进行稠密采样，并使用相应线段方向估计航向。每个采样点通过双圆柱膨胀查询判断是否碰撞。连续占据区间被归并为碰撞段，每段记录自由空间入口和出口控制点索引。

优化过程中若出现新的碰撞，`BsplineOptimizer::check_collision_and_rebound()` 会再次执行相似的碰撞段提取，并为新障碍补充绕障约束。这种“检测—优化—再检测”的方式避免了预先构建全局连续距离场。

### 6.2 投影 A* 搜索面

对于碰撞段入口 $\mathbf x^{\mathrm{in}}$ 与出口 $\mathbf x^{\mathrm{out}}$，论文不在完整三维栅格中自由扩展，而是将搜索限制在由两端高度确定的倾斜曲面上。对任意水平候选点 $\mathbf x_{xy}$，定义投影比例

$$
\beta(\mathbf x_{xy})=
\Pi_{[0,1]}\left(
\frac{(\mathbf x_{xy}-\mathbf x^{\mathrm{in}}_{xy})^\top
(\mathbf x^{\mathrm{out}}_{xy}-\mathbf x^{\mathrm{in}}_{xy})}
{\|\mathbf x^{\mathrm{out}}_{xy}-\mathbf x^{\mathrm{in}}_{xy}\|^2}
\right),
$$

并令

$$
z(\mathbf x_{xy})=z^{\mathrm{in}}+
\beta(\mathbf x_{xy})(z^{\mathrm{out}}-z^{\mathrm{in}}).
$$

因此，搜索状态主要由二维网格坐标决定，$z$ 坐标由上述插值关系给出。当前代码在 `AStar::AstarSearch()` 中仅枚举 $dx,dy\in\{-1,0,1\}$ 的八邻域，并通过 `interpolateZIndexOnSearchPlane()` 计算邻居的高度索引，构成论文所述的 projected A*。

邻居方向同时确定碰撞检查航向：

$$
\psi_\Delta=\operatorname{atan2}(\Delta y,\Delta x).
$$

只有双圆柱模型在该航向下不发生碰撞，候选节点才会进入开放集。

### 6.3 从 A* 路径构造 rebound 几何量

设 A* 返回无碰撞路径 $\Gamma$。对于受影响控制点 $\mathbf Q_i$，优化器在 A* 路径上寻找与控制点局部法平面的交点，并沿控制点到该交点的方向反向搜索障碍边界。由此构造：

- 基准点（代码中的 `base_point`，论文中的 anchor）$\mathbf a_{ij}$；
- 单位回弹方向（代码中的 `direction`）$\mathbf v_{ij}$。

控制点相对于障碍的投影距离定义为

$$
d_{ij}=(\mathbf Q_i-\mathbf a_{ij})^\top\mathbf v_{ij}.
$$

与直接对二值占据栅格求导相比，这种构造将离散搜索结果转换为连续优化可用的方向信息，同时保留了 A* 所选择的绕障拓扑。

---

## 7. 轨迹优化目标

总体目标函数写为

$$
\min_{\mathbf Q}\;J(\mathbf Q)
=\lambda_cJ_c+\lambda_sJ_s+\lambda_fJ_f,
$$

当前代码还保留轨迹拟合项及 refine 阶段使用的附加代价，因此实现中的组合项比论文主公式略丰富。各主要代价如下。

### 7.1 碰撞代价

设期望安全间隙为 $s_f$，定义

$$
c_{ij}=s_f-d_{ij}.
$$

碰撞代价为

$$
J_c=\sum_{i,j}\rho(c_{ij}),
$$

其中 $\rho$ 为单边、连续可微的惩罚函数。当控制点与障碍间距不足时，梯度沿 $\mathbf v_{ij}$ 将控制点推出障碍：

$$
\frac{\partial J_c}{\partial\mathbf Q_i}
=-\sum_j\rho'(c_{ij})\mathbf v_{ij}.
$$

代码对应 `BsplineOptimizer::calcDistanceCostRebound()`。二值 occupancy 仅用于碰撞判定和约束构造，不直接作为可微目标。

### 7.2 平滑性代价

论文以 jerk 控制点平方和衡量轨迹平滑性：

$$
J_s=\sum_k\|\mathbf J_k\|^2.
$$

其实现位于 `BsplineOptimizer::calcSmoothnessCost()`。该项抑制高频形变和控制点折曲，使轨迹更适合连续跟踪。

### 7.3 动力学可行性代价

对于速度上限 $v_m$ 和加速度上限 $a_m$，使用单边平方惩罚

$$
F(c,c_m)=\max(c-c_m,0)^2,
$$

从而得到

$$
J_f=\sum_kF(\|\mathbf V_k\|,v_m)
+\sum_kF(\|\mathbf A_k\|,a_m).
$$

代码中的 `calcFeasibilityCost()` 按各坐标轴对超限速度和加速度计算代价与梯度。优化后，`UniformBspline::checkFeasibility()` 还会进行一次显式检查；若仍不满足限制，`SCANPlannerManager::refineTrajAlgo()` 通过调整时间分配并再次优化提高可行性。

### 7.4 垂向变化抑制

论文强调在优化更新中抑制 $z$ 方向形变，使避障主要发生于水平面。当前仓库可直接确认的实现包括：

1. 初始路径的 `applyLinearZReference()` 高度正则化；
2. projected A* 将搜索限制在端点高度插值曲面；
3. 碰撞引导方向由该投影路径产生。

这些机制共同限制了不必要的垂向绕障。评价代码时，应将其理解为贯穿初始化、搜索和优化引导的结构性约束，而非单一函数中的独立“z 梯度开关”。

---

## 8. 三维占据地图与滑动存储

### 8.1 概率占据更新

`GridMap` 使用三维均匀栅格表示环境。传感器射线穿过的体素获得 miss 更新，射线终点获得 hit 更新；占据概率以 log-odds 形式存储并限制在给定上下界。其一般形式为

$$
L_t(m)=\operatorname{clip}
\left(L_{t-1}(m)+\ell(z_t,m),L_{\min},L_{\max}\right),
$$

其中 $m$ 为体素，$\ell(z_t,m)$ 为当前观测带来的 hit 或 miss 增量。上界防止长期占据体素变得难以清除，下界使自由空间和动态变化能够在后续观测中得到更新。

主要缓冲区包括：

- `occupancy_buffer_`：原始占据 log-odds；
- `occupancy_buffer_inflate_`：膨胀后的二值占据状态；
- `occupancy_buffer_inflate_cnt_`：支持增量膨胀更新的计数；
- raycasting 过程中使用的命中、穿越和缓存标志。

### 8.2 增量障碍膨胀

当体素的占据状态发生变化时，地图调用 `updateInflation()` 增量维护膨胀层，而不是每帧重新构建全部膨胀地图。膨胀核由水平半径、向上高度和向下高度共同决定。

### 8.3 机器人中心滑动地图

地图窗口尺寸固定，并随机器人位置移动。`GridMap::updateSlidingMap()` 首先将新的地图中心转换为全局体素索引，计算窗口平移量。若平移超过阈值，则：

1. 保留新旧窗口重叠区域对应的存储；
2. 收集移出窗口的环形缓冲区地址；
3. 撤销这些地址对膨胀层的旧贡献；
4. 将地址重置为未知状态，供新进入窗口的体素复用；
5. 更新地图原点和边界索引。

如果单次位移超过整个窗口尺寸，则直接重置地图数据。该设计使内存开销仅与局部窗口体素数有关，而不随导航距离增长。

### 8.4 论文中的边界回退与当前代码

论文描述了一种 dead-end recovery：在滑动地图边界外临时增加一层仅用于假设搜索的虚拟自由体素，并将假设路径离开真实地图的位置选为回退目标。

当前仓库中可以直接确认的相关机制包括：

- 固定尺寸滑动地图与边界检查；
- 局部目标占据时沿全局轨迹前后寻找可用目标；
- A* 起终点占据时沿连线方向调整；
- 规划失败后依次尝试旧轨迹初始化、多项式初始化和随机多项式初始化；
- 连续失败达到阈值后进入紧急停止。

但当前代码中未见与论文描述完全同构的“地图外一体素虚拟自由层”独立实现。因此，在分析开源版本时，不宜将该论文机制不加区分地归入 `dyn_a_star.cpp`。它可能属于未公开分支、后续版本，或在当前版本中由其他回退逻辑部分替代。

---

## 9. 全局路线、局部目标与滚动重规划

### 9.1 三种导航输入模式

仓库在 `run.launch` 和 `SCANReplanFSM` 中定义三种导航模式：

1. **交互目标模式**：接收 RViz 的 `/move_base_simple/goal`；
2. **预设关键点模式**：从关键点文件读取多楼层或多段导航目标；
3. **参考路径模式**：订阅 `/initial_path`，沿外部系统给出的路线执行局部避障。

因此，本仓库中的“全局规划”更多表现为目标间多项式轨迹或外部参考路径，而非在完整地图上自行运行全局图搜索。

### 9.2 局部目标选取

`SCANReplanFSM::getLocalTarget()` 首先在全局参考轨迹上找到距离当前规划起点最近的位置，再沿轨迹向前累计弧长，达到 `planning_horizon` 后选为局部目标。若该目标在当前双圆柱膨胀地图中不可用，则沿全局轨迹前后搜索可行目标。

这一过程将长距离导航分解为连续的有限视距局部规划问题：

$$
\text{long-range route}
\longrightarrow
\{\text{local target}_k\}_{k=0}^{\infty}.
$$

### 9.3 有限状态机

`SCANReplanFSM` 以定时器驱动以下状态：

```text
INIT
  └─等待里程计与触发信号
       ↓
WAIT_TARGET
  └─等待有效目标
       ↓
GEN_NEW_TRAJ
  └─从当前状态生成新轨迹
       ↓
EXEC_TRAJ
  ├─接近起点或终点时继续执行
  └─满足重规划条件时进入 REPLAN_TRAJ
       ↓
REPLAN_TRAJ
  └─从当前轨迹或里程计状态重新规划

任意执行状态 ──紧急碰撞或连续失败──> EMERGENCY_STOP
```

状态机主循环周期为 0.01 s，安全检查定时器周期为 0.05 s。安全检查沿待执行轨迹以更细时间步采样，一旦发现双圆柱碰撞，先尝试即时重规划；若碰撞时间余量小于 `emergency_time`，则进入紧急停止。

`GEN_NEW_TRAJ` 与 `REPLAN_TRAJ` 最终都会调用 `SCANPlannerManager::reboundReplan()`，因此二者使用的是同一套局部轨迹优化算法；差异主要在于触发场景和初始轨迹的构造方式。

| 状态 | 主要含义 | 典型触发条件 | 初始轨迹来源 | 设计重点 |
|---|---|---|---|---|
| `GEN_NEW_TRAJ` | 为新导航任务建立局部轨迹 | 首次收到目标、切换 waypoint、紧急停止后恢复 | 通常由起点状态和局部目标重新生成多项式轨迹，再参数化为 B 样条 | 建立新的规划任务，必要时允许随机初值恢复规划 |
| `REPLAN_TRAJ` | 对当前执行任务进行滚动重规划 | 执行过程中达到重规划条件或发现前方新障碍 | 优先截取当前 B 样条的剩余部分，并以当前执行时刻的位置、速度和加速度作为边界状态 | 保持轨迹连续性，在原目标或参考路径方向上修正后续轨迹 |

在代码中，`GEN_NEW_TRAJ` 直接调用 `callReboundReplan(true, ...)`，通常强制使用多项式初始化；`REPLAN_TRAJ` 通过 `planFromCurrentTraj()` 进入重规划流程，在参考路径模式下优先复用当前轨迹，失败后才退回到多项式或随机多项式初始化。因此，两种状态的区别不是优化器不同，而是“重新建立轨迹”和“接续并修正当前轨迹”之间的区别。

### 9.4 重规划回退链

重规划时，系统优先使用当前未执行完的 B 样条片段作为初值。若失败，则依次尝试：

1. 标准多项式初始化；
2. 带随机中间点的多项式初始化；
3. 持续失败达到阈值后发布定点 B 样条并进入紧急停止。

随机初始化不是最终规划策略，而是改变初始轨迹拓扑、帮助连续优化跳出局部极小值的恢复手段。

---

## 10. 仓库代码结构

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

### 10.1 `plan_manage`：系统编排层

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
| `launch/advanced_param.xml` | 核心参数 | 地图、机身、优化、动力学和 FSM 参数 |

### 10.2 `plan_env`：环境表示层

`GridMap` 同时承担传感器融合、栅格存储、地图滑动、增量膨胀和整机碰撞查询。核心文件为：

- `include/plan_env/grid_map.h`：数据结构、内联查询和公共接口；
- `src/grid_map.cpp`：初始化、回调、raycasting、膨胀和滑动更新；
- `include/plan_env/raycast.h`、`src/raycast.cpp`：体素射线遍历。

该模块位于感知与规划之间。上游输入是点云和位姿，下游输出不是几何点云，而是可供规划器高频查询的占据与膨胀状态。

### 10.3 `path_searching`：局部拓扑引导层

`dyn_a_star.cpp` 实现碰撞段入口到出口之间的投影 A*。其主要数据结构包括开放集、节点池、轮次标志、代价与父指针。该搜索仅服务于 B 样条 rebound 约束生成，不是系统的全局规划器。

### 10.4 `bspline_opt`：连续轨迹优化层

该包包含：

- `uniform_bspline.*`：B 样条求值、求导、参数化、可行性检查和时间延长；
- `bspline_optimizer.*`：碰撞段识别、A* 调用、rebound 几何构造、代价与梯度；
- `lbfgs.hpp`：L-BFGS 求解器实现；
- `gradient_descent_optimizer.*`：仓库保留的另一优化器实现。

阅读时不应首先深入 `lbfgs.hpp`。理解本项目所定义的变量、目标函数和梯度组合，比研究通用求解器内部细节更重要。

### 10.5 `traj_utils`：轨迹初始化与可视化层

`PolynomialTraj` 提供单段五次多项式和多段 minimum-snap 轨迹生成；`PlanningVisualization` 发布全局路径、初始路径、A* 路径、控制点和优化轨迹等 RViz marker。这些可视化接口对于核对规划阶段和定位失败原因具有直接价值。

---

## 11. 一次局部规划的代码调用链

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

### 11.1 轨迹执行控制器

规划器输出的是连续位置轨迹，而不是直接可执行的足端或关节命令。仓库提供两种轨迹执行方式，二者共享 B 样条消息格式，但反馈结构不同。

#### 闭环执行：速度前馈与比例反馈

`closed_loop_controller.cpp` 接收 `/planning/bspline`，由位置 B 样条求得速度和加速度导数，并通过里程计获得当前机器人位置和 yaw。平面速度指令采用速度前馈与位置误差比例修正的形式：

$$
\mathbf v_{\mathrm{cmd}}^{w}
=\mathbf v_{\mathrm{ff}}^{w}
+K_p\left(\mathbf p_{\mathrm{des}}^{w}-\mathbf p_{\mathrm{odom}}^{w}\right),
$$

其中 $\mathbf v_{\mathrm{ff}}^{w}$ 是 B 样条一阶导数，误差项用于补偿执行偏差。随后根据当前机体 yaw 将世界坐标系速度转换到机体坐标系，并分别施加 `max_vx`、`max_vy` 限幅。

期望 yaw 由轨迹前方的 look-ahead 点与当前位置确定；航向误差经过角度归一化后采用比例控制：

$$
\omega_z=\operatorname{sat}\left(K_{p,\psi}\,
\operatorname{wrap}(\psi_{\mathrm{des}}-\psi_{\mathrm{odom}})\right).
$$

当航向误差超过 `heading_error_threshold` 时，控制器暂时冻结 B 样条执行时间，停止平移并仅保留角速度，使机器人原地对齐轨迹方向。该机制使轨迹参数时间不会在明显偏航时继续向前推进。

因此，闭环控制器的性质是：

```text
B 样条速度前馈
    + 平面位置 P 反馈
    + 航向 P 反馈
    + 航向误差门控
    + 速度和角速度饱和
```

它不是 PID、MPC 或四足关节级控制器。控制输出为 `geometry_msgs/Twist`，具体的足端、姿态和关节跟踪仍由下游机器人控制系统负责。

#### 开环执行：按时间回放轨迹状态

`open_loop_controller.cpp` 不读取当前位置误差来修正轨迹，而是根据消息中的 `start_time` 计算当前轨迹时间，直接查询：

$$
\mathbf p(t),\qquad \mathbf v(t)=\dot{\mathbf p}(t),\qquad
\mathbf a(t)=\ddot{\mathbf p}(t).
$$

水平速度方向用于生成 yaw：

$$
\psi(t)=\operatorname{atan2}(v_y(t),v_x(t)),
$$

当水平速度足够大时，yaw rate 由速度和加速度计算：

$$
\dot\psi(t)=
\frac{v_xa_y-v_ya_x}{v_x^2+v_y^2}.
$$

该节点主要用于仿真中的轨迹回放和状态发布，不具备位置误差反馈能力。闭环仿真中，`go2_kinematic_sim.cpp` 订阅闭环控制器发布的 `cmd_vel`，使用简化运动学模型更新机器人位姿，再将位姿反馈给闭环控制器；这构成了一个用于验证轨迹跟踪链路的简化闭环，而不是完整四足动力学控制器。

#### 控制器的精度边界

该执行层适合验证局部导航轨迹的跟踪和避障效果，但其精度受到以下因素共同限制：

1. 闭环控制器以比例反馈为主，没有积分补偿，可能存在稳态位置误差；
2. 控制器输出的是机体速度指令，没有显式建模足端接触、摩擦、步态和关节动力学；
3. 当前代码对水平位置有直接反馈，而垂向运动主要依赖规划高度、里程计和底层四足控制器；
4. 速度限幅、通信延迟、定位误差和底层执行器响应都会降低实际轨迹精度；
5. 开环模式只能复现时间参数化轨迹，不能抵抗实际机器人偏离。

因此，控制器应理解为面向导航的轻量级轨迹跟踪接口，而不是厘米级精密控制器或足端落足控制器。评价执行精度时，应分别统计平面位置误差、航向误差、速度饱和比例、目标终点误差和重规划次数，不能仅依据规划轨迹本身判断机器人实际运动精度。

---

## 12. 论文概念与代码实现对照

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

## 13. 推荐代码阅读路线

### 13.1 第一阶段：建立系统全貌

建议依次阅读：

1. `README.md`：确认系统用途、运行模式和软件依赖；
2. `plan_manage/launch/run.launch`：确认启动节点、话题和模式选择；
3. `plan_manage/launch/advanced_param.xml`：建立参数与论文符号之间的对应；
4. `scan_planner_node.cpp`：确定程序入口；
5. `scan_replan_fsm.h/.cpp`：掌握状态机、目标来源和重规划时机。

该阶段的目标是回答：系统何时开始规划、局部目标从何而来、什么时候重新规划、轨迹如何发布。

### 13.2 第二阶段：掌握单次规划主线

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

### 13.3 第三阶段：理解整机碰撞与地图

阅读顺序为：

1. `grid_map.h` 中的 `MappingParameters` 与 `MappingData`；
2. `GridMap::initMap()`；
3. `GridMap::getInflateOccupancy(pos, yaw)`；
4. 原始 occupancy 更新和 `updateInflation()`；
5. `GridMap::updateSlidingMap()`；
6. `raycastProcess()` 与 `raycast.cpp`。

建议先理解查询接口和数据布局，再进入传感器融合细节。否则容易在较长的地图实现中失去主线。

### 13.4 第四阶段：理解 A* 与 rebound 约束

推荐顺序：

1. `BsplineOptimizer::initControlPoints()`；
2. `AStar::ConvertToIndexAndAdjustStartEndPoints()`；
3. `AStar::AstarSearch()`；
4. `AStar::getPath()`；
5. 回到 `initControlPoints()`，继续阅读 A* 路径到 `base_point/direction` 的转换；
6. `BsplineOptimizer::check_collision_and_rebound()`。

这一阶段应明确三类几何对象之间的关系：原始控制点、无碰撞 A* 路径和障碍边界基准点。

### 13.5 第五阶段：理解优化目标和求解过程

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

### 13.6 第六阶段：闭合执行链

最后阅读：

1. `callReboundReplan()` 中的 B 样条消息封装；
2. `closed_loop_controller.cpp` 的 `bsplineCallback()`；
3. B 样条位置、速度和加速度的实时求值；
4. `checkCollisionCallback()` 的执行期安全监督；
5. `planning_visualization.cpp` 中各类 RViz marker 的含义。

至此可以完整解释一次规划周期从输入、地图、搜索、优化到控制输出的全过程。

---

## 14. 关键参数及其物理意义

### 14.1 机器人与地图参数

| 参数类别 | 代表参数 | 主要影响 |
|---|---|---|
| 地图分辨率 | `grid_map/resolution` | 碰撞精度、A* 粒度、内存与计算量 |
| 地图窗口 | `sliding_map_size_x/y/z` | 局部视野范围与内存规模 |
| 滑动阈值 | `map_sliding_thresh` | 地图窗口移动频率 |
| 水平膨胀 | `double_cylinder_radius` | 双圆柱横向安全间隙 |
| 纵向偏移 | `double_cylinder_offset` | 前后圆柱中心距离机器人中心的偏移 |
| 垂向膨胀 | `obstacles_inflation_z_up/down` | 机身高度、跨越能力与悬空障碍判定 |

### 14.2 轨迹与优化参数

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

---

## 15. 方法特点与适用边界

### 15.1 主要特点

1. **以较低代价实现航向相关整机碰撞**：双圆柱和膨胀地图将复杂体积碰撞转化为两个点查询；
2. **兼顾完整三维环境与地面运动特性**：三维 occupancy 保留悬空结构，projected A* 和高度正则抑制非物理垂向绕行；
3. **离散搜索与连续优化互补**：A* 负责拓扑选择，B 样条优化负责平滑性和动力学可行性；
4. **适合在线局部重规划**：碰撞约束按需构造，无需预先维护全局 ESDF；
5. **内存规模与场景长度解耦**：滑动地图支持长距离运行；
6. **工程安全链较完整**：执行期碰撞检查、分级重规划和紧急停止共同工作。

### 15.2 适用边界与局限

1. 双圆柱是机身近似，不等价于腿部、摆动足和全关节构型碰撞；
2. 航向由轨迹切线诱导，未显式优化独立 yaw；
3. 高度正则化和投影搜索不等同于完整地形可通行性分析；
4. 最大可跨越高度由地图膨胀参数间接表达，系统本身不求解落足点或接触序列；
5. 对高速动态障碍，基于占据更新和周期重规划的方法仍受感知延迟与预测能力限制；
6. 局部优化仍可能受初值和局部极小值影响，随机多项式初始化只能缓解而不能从理论上消除该问题；
7. 执行层主要是速度前馈与比例反馈，未显式处理四足接触动力学、足端落点和垂向闭环；
8. 开环执行仅适合仿真或轨迹回放，实际机器人运行应关注闭环跟踪误差和底层控制器响应；
9. 论文中的边界回退在当前仓库中未发现完全对应的公开实现，复现实验时应以代码实际行为为准。

因此，SCAN-Planner 更准确的定位是：**面向四足导航的连续局部机身轨迹规划器**，而不是完整的落足规划器、全身运动规划器或地形动力学求解器。

---

## 16. 总结

SCAN-Planner 以位置 B 样条为连续轨迹主体，通过轨迹切线诱导航向，并以航向感知双圆柱模型完成整机近似碰撞检查。发生碰撞时，系统在端点高度插值曲面上执行 projected A*，将无碰撞路径转化为控制点的 rebound 基准点与方向，再以平滑性、碰撞安全和动力学可行性为目标优化轨迹。机器人中心滑动三维地图为该过程提供固定内存规模的高分辨率环境查询，全局参考轨迹与有限状态机则负责局部目标推进、滚动重规划和执行期安全管理；轨迹发布后，系统可通过带速度前馈和比例反馈的闭环接口进行导航跟踪，也可在仿真中按时间参数开环回放。

从软件结构看，项目形成了较清晰的层次：`plan_manage` 负责时序和系统编排，`plan_env` 负责地图及碰撞查询，`path_searching` 负责局部拓扑引导，`bspline_opt` 负责连续轨迹优化，`traj_utils` 提供初始化和可视化。理解该仓库的关键不在于逐文件顺序通读，而在于首先沿 `SCANReplanFSM → SCANPlannerManager::reboundReplan → BsplineOptimizer → GridMap/AStar → B 样条发布` 建立完整调用链，再分别深入地图、搜索和优化模块。

在这一框架下，论文中的主要贡献可以概括为：以紧凑的表示和有限计算代价，将细长四足机器人的航向相关机身碰撞、三维障碍推理、地面跟随运动和长距离路线引导统一到一个可实时运行的局部规划系统中。

---

## 参考资料

1. Han Zheng, Zhe Chen, Yiwen Fu, Ming Yang, Tong Qin. *SCAN-Planner: Spatial Collision-Aware Local Planning for Route-Guided Long-Range Quadruped Navigation*. arXiv:2606.19555, 2026.
2. SCAN-Planner repository, project `README.md` and source code under `src/planner/`.
3. X. Zhou et al. *EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors*. IEEE Robotics and Automation Letters, 2021.
4. Y. Ren et al. *ROG-Map: An Efficient Robocentric Occupancy Grid Map for Large-Scene and High-Resolution LiDAR-based Motion Planning*. IROS, 2024.

