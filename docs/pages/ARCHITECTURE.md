# maplab 2.0 — 系统架构文档

> 本文档由代码自动分析生成，覆盖 maplab 2.0 各核心模块的算法实现及数据流。

---

## 目录

1. [系统总览](#1-系统总览)
2. [核心数据结构 — VI-Map](#2-核心数据结构--vi-map)
3. [视觉前端 — aslam_cv2](#3-视觉前端--aslam_cv2)
4. [在线建图前端 — ROVIOLI](#4-在线建图前端--rovioli)
5. [通用建图节点 — maplab-node](#5-通用建图节点--maplab-node)
6. [IMU 预积分](#6-imu-预积分)
7. [特征追踪与回环检测](#7-特征追踪与回环检测)
8. [地标三角化](#8-地标三角化)
9. [地图优化 — Bundle Adjustment](#9-地图优化--bundle-adjustment)
10. [多机器人建图 — maplab-server](#10-多机器人建图--maplab-server)
11. [离线控制台与插件系统](#11-离线控制台与插件系统)
12. [稠密重建](#12-稠密重建)
13. [地图稀疏化与摘要](#13-地图稀疏化与摘要)
14. [消息流系统](#14-消息流系统)
15. [坐标系约定](#15-坐标系约定)
16. [典型工作流](#16-典型工作流)

---

## 1. 系统总览

maplab 2.0 由三个主要运行时部分构成：

```
┌─────────────────────────────────────────────────────────────────────┐
│                         maplab 2.0 总体架构                          │
│                                                                     │
│  ┌──────────────────┐    ┌──────────────────┐    ┌───────────────┐  │
│  │  Mapping Node    │    │  Mapping Server  │    │    Console    │  │
│  │ (ROVIOLI /       │───▶│ (多机器人融合)    │    │ (离线优化)    │  │
│  │  maplab-node)    │    │                  │    │               │  │
│  └──────────────────┘    └──────────────────┘    └───────────────┘  │
│         │                        │                       │          │
│         ▼                        ▼                       ▼          │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                    VI-Map (核心数据结构)                       │   │
│  │   Missions | Vertices (Keyframes) | Edges | Landmarks         │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

### 模块目录映射

| 目录 | 功能 |
|------|------|
| `applications/rovioli` | 在线 VIO 前端（ROVIO + 建图） |
| `applications/maplab-node` | 通用多传感器在线建图节点 |
| `applications/maplab-server-node` | 多机器人地图服务器 |
| `applications/maplab-console` | 交互式离线地图处理控制台 |
| `map-structure/vi-map` | VI-Map 核心数据结构 |
| `map-structure/posegraph` | 位姿图顶点/边抽象 |
| `map-structure/sensors` | 传感器模型与外参 |
| `aslam_cv2/` | 相机模型、特征检测、追踪、三角化 |
| `algorithms/feature-tracking` | ORB 检测 + BRISK/FREAK 描述子追踪 |
| `algorithms/imu-integrator-rk4` | RK4 IMU 预积分 |
| `algorithms/landmark-triangulation` | 多视图线性三角化 |
| `algorithms/loopclosure/` | 回环检测完整流水线 |
| `algorithms/map-optimization` | Ceres Bundle Adjustment |
| `algorithms/ceres-error-terms` | 所有优化残差定义 |
| `algorithms/map-sparsification` | 关键帧剪枝与地标稀疏化 |
| `algorithms/online-map-builders` | 流式地图构建器 |
| `algorithms/map-anchoring` | 多任务锚定 |
| `algorithms/dense-reconstruction` | 立体视觉/深度稠密重建 |
| `common/vio-common` | VIO 核心数据类型 |
| `common/message-flow` | 异步 pub/sub 消息系统 |
| `backend/map-manager` | 地图文件 I/O |
| `console-plugins/` | 控制台命令插件 |

---

## 2. 核心数据结构 — VI-Map

### 2.1 VI-Map 层次结构

```
VIMap
├── Mission (一次连续建图会话)
│   ├── MissionId
│   ├── baseframe T_G_M  (任务坐标系到全局坐标系)
│   ├── NCamera (多相机外参)
│   └── root_vertex_id
│
├── Vertex (关键帧节点，位于 posegraph)
│   ├── VertexId
│   ├── MissionId
│   ├── VisualNFrame (图像帧 + 特征)
│   │   └── VisualFrame[0..N-1]
│   │       ├── keypoints (2×K matrix)
│   │       ├── descriptors (BRISK/FREAK binary)
│   │       ├── track_ids
│   │       └── observed_landmark_ids
│   ├── IMU state: T_M_I, v_M, bias_acc, bias_gyro
│   └── landmark_store (归属于本节点的地标)
│
├── Edge (约束/连接)
│   ├── ViwlsEdge   — IMU 预积分边（最常见）
│   ├── LoopClosureEdge — 回环约束
│   ├── WheelOdometryEdge — 轮式里程计
│   └── TransformationEdge — 6DoF 相对位姿
│
├── Landmark (3D 地图点)
│   ├── LandmarkId
│   ├── p_B_fi (在 base vertex 坐标系下的 3D 位置)
│   ├── quality: kUnknown / kGood / kBad
│   └── observations: [(VertexId, FrameIdx, KeypointIdx), ...]
│
└── LandmarkIndex (全局 LandmarkId → VertexId 索引)
```

### 2.2 关键 API

```cpp
// 遍历所有任务的所有顶点
vi_map.forEachVertex([](const vi_map::Vertex& vertex) { ... });

// 获取顶点的观测到的地标
vertex.getFrameObservedLandmarkIds(frame_idx, &landmark_ids);

// 获取地标的所有观测
landmark.forEachObservation([](const KeypointIdentifier& obs) { ... });

// 获取顶点全局位姿
Eigen::Quaterniond q_G_I;
Eigen::Vector3d p_G_I;
vi_map.getVertex_T_G_I(vertex_id, &q_G_I, &p_G_I);
```

### 2.3 地图序列化格式

地图保存为目录结构：
```
map_folder/
├── metadata
├── vi_map/
│   ├── vertices0, vertices1, ...  (分片 Protobuf)
│   ├── edges
│   ├── missions
│   ├── landmark_index
│   └── other_fields
├── resource_info
└── resources/
    └── {sensor_id}/{resource_type}/{timestamp}.{ext}
```

---

## 3. 视觉前端 — aslam_cv2

### 3.1 相机模型

支持三种相机模型，共享统一接口 `project3()` / `backProject3()`：

| 模型 | 参数 | 适用场景 |
|------|------|----------|
| `PinholeCamera` | fu, fv, cu, cv | 标准针孔相机 |
| `UnifiedProjectionCamera` | xi, fu, fv, cu, cv | 鱼眼/折反射全向相机 |
| `Camera3DLidar` | h_res, v_res, v_center, h_center | 机械旋转 LiDAR (如 Ouster) |

支持四种畸变模型：`RadTan`（k1,k2,p1,p2）、`Equidistant`（k1-k4，Kannala-Brandt）、`Fisheye`（w 参数）、`NullDistortion`。

### 3.2 帧数据结构

```
VisualNFrame (多相机同步帧)
└── VisualFrame[0..N-1]
    ├── Channel: VISUAL_KEYPOINT_MEASUREMENTS   (2×K Matrix)
    ├── Channel: DESCRIPTORS                    (binary, 支持多种描述子块)
    ├── Channel: TRACK_IDS                      (-1 = 未追踪)
    ├── Channel: VISUAL_KEYPOINT_3D_POSITIONS   (深度传感器专用)
    ├── Channel: VISUAL_KEYPOINT_TIME_OFFSETS   (滚动快门补偿)
    └── raw_image (cv::Mat)
```

### 3.3 特征追踪流水线

```
图像输入
    │
    ▼
VisualNPipeline (多相机异步处理，线程池)
    │── VisualPipeline[0] → VisualFrame (去畸变 + 特征提取)
    │── VisualPipeline[1] → VisualFrame
    └── ... 同步聚合为 VisualNFrame
         │
         ▼
GyroTracker (特征追踪，每相机)
    ├── Step 1: GyroTwoFrameMatcher
    │           使用陀螺仪预测关键点位置 → 描述子匹配 (Hamming 距离)
    │           Lowe 比率阈值: 0.8
    └── Step 2: LK 光流 (cv::calcOpticalFlowPyrLK)
                追踪未匹配特征点 → 提取描述子
         │
         ▼
TrackManager → 分配 track_id (跨帧连续追踪 ID)
```

### 3.4 PnP 位姿估计

`PnpPoseEstimator` 支持：
- 单相机 RANSAC PnP（`absolutePoseRansac`）
- 多相机 RANSAC PnP（`absoluteMultiPoseRansac`）
- 3D-3D 对应（LiDAR 特征）
- 可选非线性精化（所有内点上运行 Ceres）

---

## 4. 在线建图前端 — ROVIOLI

**ROVIOLI = RObust Visual Inertial Odometry with Localization Integration**

### 4.1 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         ROVIOLI Node                            │
│                                                                 │
│  DataSource (ROS topic / rosbag)                               │
│      │ IMU measurements                                        │
│      │ Image                                                   │
│      ▼                                                          │
│  ImuCameraSynchronizer                                          │
│      │ SynchronizedNFrameImu                                   │
│      ▼                                                          │
│  ┌───────────────────┐    ┌──────────────────────────────────┐  │
│  │ FeatureTracking   │    │ RovioFlow (ROVIO iEKF 估计器)    │  │
│  │ (BRISK/FREAK +    │    │ 输入: 图像 + IMU                 │  │
│  │  GyroTracker)     │    │ 输出: T_M_I, v_M, biases        │  │
│  └───────────────────┘    └──────────────────────────────────┘  │
│           │                           │                          │
│           └───────────┬───────────────┘                          │
│                       ▼                                          │
│               VioUpdateBuilder                                   │
│                (组合特征 + ROVIO 位姿)                            │
│                       │ VioUpdate                               │
│                       ▼                                          │
│  ┌────────────────────────────────────────────────────┐          │
│  │ MapBuilderFlow → StreamMapBuilder → VIMap          │          │
│  │ (构建位姿图: 顶点 + ViwlsEdge + 特征观测)            │          │
│  └────────────────────────────────────────────────────┘          │
│                       │                                          │
│                       ▼                                          │
│  ┌───────────────────────────────────┐                           │
│  │  (可选) LocalizerFlow             │                           │
│  │  投影地图点 → PnP RANSAC → T_G_M  │                           │
│  └───────────────────────────────────┘                           │
│                       │                                          │
│               DataPublisherFlow                                  │
│               (ROS topic 发布位姿、地图)                          │
└─────────────────────────────────────────────────────────────────┘
```

### 4.2 ROVIO 估计器

ROVIO 是基于**迭代扩展卡尔曼滤波 (iEKF)** 的视觉-惯性里程计：
- 跟踪基于图像 patch 的特征（photometric error）
- 状态量：IMU 位姿、速度、偏置、特征 patch 位置
- ROVIO 的特征**不用于建图**，只用于位姿估计
- 建图使用独立的 `FeatureTracking` 模块提取 BRISK/FREAK 特征

### 4.3 数据流关键注意

- 地图在运行时**只构建顶点和边**，地标在关机时批量三角化
- 因此 ROVIOLI **不支持**在线定位到当前正在建的地图
- 支持定位到**预先构建的地图**（LOC 模式）

---

## 5. 通用建图节点 — maplab-node

maplab-node 是更通用的在线建图节点，支持外部 VIO 估计器（任意来源，非 ROVIO）。

### 5.1 架构差异（相对于 ROVIOLI）

```
外部 VIO 估计器  (任意 odometry source)
      │ T_M_I, v_M, biases (via ROS topic)
      ▼
Synchronizer (同步 NFrame + IMU + odometry)
      │
      ▼  MapUpdate (含 ViNodeState)
StreamMapBuilder → VIMap
```

主要差异：
- `OdometryEstimate` 替代 `RovioEstimate`（接受外部位姿）
- `Synchronizer` 比 ROVIOLI 的 `ImuCameraSynchronizer` 功能更多（支持更多传感器类型）
- 支持 LiDAR、外部特征等更多传感器的同步

### 5.2 StreamMapBuilder 数据流

```
MapUpdate 到达
    │
    ├── 创建 Vertex（VIO 位姿 + VisualNFrame）
    ├── 创建 ViwlsEdge（与前一顶点，含原始 IMU 数据）
    ├── 附加绝对位姿约束（GPS/AprilTag，时间最近原则）
    ├── 附加轮式里程计边
    ├── 附加 LiDAR 点云资源
    └── 附加外部特征（含外点剔除）
```

---

## 6. IMU 预积分

**包路径：** `algorithms/imu-integrator-rk4`

### 6.1 状态表示

| 维度 | 变量 | 说明 |
|------|------|------|
| 4 | `q_I_M` | JPL 四元数（IMU 到 mission 帧） |
| 3 | `b_g` | 陀螺仪偏置 (rad/s) |
| 3 | `v_M` | 速度（mission 帧） |
| 3 | `b_a` | 加速度计偏置 (m/s²) |
| 3 | `p_M_I` | 位置（mission 帧） |

### 6.2 RK4 积分

连续时间状态导数：

```
q_dot  = 0.5 * Omega(ω - b_g) * q
v_dot  = R_M_I * (a - b_a) - g
p_dot  = v
b_dot  = 0  (随机游走，在协方差中建模)
```

RK4 使用**线性插值** IMU 读数于中间时刻（k2, k3 阶段）。

### 6.3 协方差传播

同时用 RK4 积分 **15×15 协方差矩阵**，连续时间系统矩阵 `F_c` 结构：

```
F_c[rot, rot]  = -skew(ω)
F_c[rot, bg]   = -I
F_c[vel, rot]  = -R * skew(a)
F_c[vel, ba]   = -R
F_c[pos, vel]  = I
```

过程噪声 `Q_c = diag(σ_g², σ_bg², σ_a², σ_ba²)`（对角结构）。

---

## 7. 特征追踪与回环检测

### 7.1 特征检测（`algorithms/feature-tracking`）

- **检测器：** ORB（OpenCV），支持网格化均匀分布检测 (`GriddedDetector`)
  - 图像分割为 N×M 格，每格独立并行检测，局部 NMS
- **描述子：** FREAK 或 BRISK（二值描述子，48/64 字节）
- **追踪：** `GyroTracker`（陀螺仪辅助描述子匹配 + LK 光流）

### 7.2 回环检测流水线（`algorithms/loopclosure`）

```
查询帧特征
    │ binary descriptor (BRISK/FREAK)
    ▼
descriptor-projection
    ├── SIMD bit extraction（SSE/NEON，16 字节/批）
    └── 学习型投影矩阵 → 低维实数向量（约 10D）
    │
    ▼
最近邻搜索（三种后端，可配置）
    ├── IMI  (Inverted Multi-Index，乘积量化)
    ├── IMI-PQ  (带完整乘积量化的 IMI)
    └── HNSW (Hierarchical Navigable Small World 图)
    │
    ▼
评分函数
    ├── Accumulation（简单计数）
    └── Probabilistic（负对数似然，二项分布模型）
    │
    ▼
共视过滤（BFS 连通子图搜索）
    │
    ▼
PnP RANSAC（几何验证）
    ├── 输入: 2D-3D 对应（观测关键点 ↔ 地图地标）
    └── 输出: T_G_I + 内点掩码
    │
    ▼
地标合并 / 回环边添加
```

### 7.3 词汇树（`vocabulary-tree`）

- 分层 k-means 树（`VocabularyTree<Feature, Distance>`）
- 支持 L2 距离（投影描述子）和 Hamming 距离（二值描述子）
- 用于 IMI 子空间量化中心

---

## 8. 地标三角化

**包路径：** `algorithms/landmark-triangulation`  + `aslam_cv2/aslam_cv_triangulation`

### 8.1 视觉地标（相机）

线性 N 视图三角化（DLT）：

```
输入: bearing_vectors[i] (单位向量), p_G_C[i] (相机全局位置)
方程: p_G_fi = p_G_C[i] + t * bearing_vector[i]  (对所有 i)
求解: A * p_G_fi = 0 (超定线性方程组，SVD 最小化)
```

- 使用 `PoseInterpolator` 对子帧时刻的位姿进行 IMU 插值（滚动快门补偿）
- 质量检查：最小基线、最大重投影误差 → 标记为 `kGood` / `kBad`

### 8.2 LiDAR 地标

直接平均所有观测的 3D 全局坐标：
```
p_G = mean(R_G_I[i] * p_I[i] + p_G_I[i])
```

---

## 9. 地图优化 — Bundle Adjustment

**包路径：** `algorithms/map-optimization` + `algorithms/ceres-error-terms`

### 9.1 残差项汇总

| 残差 | 类 | 微分方式 | 残差维度 |
|------|-----|----------|----------|
| 视觉重投影误差 | `VisualReprojectionError` | 解析 | 2 |
| IMU 预积分误差 | `InertialErrorTerm` | 解析 | 15 |
| 回环闭合约束 | `LoopClosureEdgeErrorTerm` | 自动微分 | 6 |
| 6DoF 里程计约束 | `SixDoFBlockPoseErrorTerm` | 自动微分 | 6 |
| 绝对位姿约束 | Block pose prior | — | 6 |
| 可切换先验 | `SwitchPriorErrorTerm` | — | 1 |

### 9.2 视觉重投影误差参数块

```
参数块[0]: p_B_fi       — 地标位置（base 顶点坐标系），3D
参数块[1]: T_B_LM       — 地标 base 顶点位姿，7D (q+t)
参数块[2]: T_G_LM       — 地标任务基坐标系，7D
参数块[3]: T_G_M        — 观测者任务基坐标系，7D
参数块[4]: T_I_M        — 观测者 IMU 位姿，7D
参数块[5]: q_C_I        — 相机-IMU 旋转，4D
参数块[6]: p_C_I        — 相机-IMU 平移，3D
参数块[7]: 相机内参      — 维度由相机类型决定
参数块[8]: 畸变参数      — 维度由畸变模型决定

残差 = (project(p_C_fi) - keypoint_measurement) / pixel_sigma
损失函数: Huber (delta=3 局部地标, delta=10 全局地标)
```

### 9.3 IMU 误差项（15 维）

```
状态向量: [q_I_M(4), b_g(3), v_M(3), b_a(3), p_M_I(3)]  →  16D
误差状态: [δθ(3), δb_g(3), δv(3), δb_a(3), δp(3)]        →  15D

积分: 使用 ImuIntegratorRK4 从 begin_state 向前积分
协方差: Q_accum (15×15)，Cholesky 分解用于白化
Jacobians: φ_accum (15×15 状态转移矩阵)
```

### 9.4 可观性分析与规范化约束

优化前自动分析每个任务簇的可观性：

| 信息来源 | 可观量 |
|----------|--------|
| IMU | 尺度、roll/pitch |
| 轮式/6DoF 里程计 | 尺度 |
| 绝对 6DoF 约束（GPS 等） | 全局位置、偏航、roll/pitch |
| 多个绝对约束 | 尺度 |

根据分析，对第一个顶点施加不同的四元数参数化约束：
- `JplYawQuaternionParameterization` — 仅优化偏航
- `JplRollPitchQuaternionParameterization` — 仅优化 roll/pitch

### 9.5 带外点剔除的优化流程

```
初始化位姿图
    │
    ▼
每 N 次迭代:
    ├── 求解 Ceres 问题
    ├── 将状态写回 VIMap
    ├── 检测外点地标 (相机后方 / 重投影误差 > 阈值)
    ├── 禁用外点地标的代价函数
    └── 标记地标为 kBad
```

### 9.6 位姿图松弛（Pose Graph Relaxation）

用于快速多任务对齐（仅优化顶点位姿，固定地标/内参/外参）：

1. 对所有任务运行回环检测，添加回环边
2. 以**可切换约束 (Switchable Constraints)** 建模回环边
   - 每边有 `switch_variable ∈ [0,1]`
   - `SwitchPriorErrorTerm` 将其拉向 1（有效）
   - 外点回环的 switch 被驱向 0（失效）
3. 优化后移除临时回环边

---

## 10. 多机器人建图 — maplab-server

**包路径：** `applications/maplab-server-node`

### 10.1 架构

```
Robot 1 (maplab-node) ──▶┐
Robot 2 (maplab-node) ──▶├──▶  MaplabServerNode
Robot N (maplab-node) ──▶┘         │
                               接收子图（SubMap）
                                    │
                                    ▼
                          SubMapHandler
                          ├── 存储到磁盘
                          ├── 合并到全局地图
                          │   ├── 锚定 (map-anchoring)
                          │   ├── 位姿图松弛
                          │   └── 全局 VI-map BA
                          └── 发布更新后的 T_G_M 给各机器人
```

### 10.2 子图合并流程

1. 接收新子图 → 保存到磁盘（容灾恢复）
2. 调用 `map-anchoring` 将子图锚定到现有全局地图
3. 运行位姿图松弛（若检测到回环）
4. 运行全局 VI-map Bundle Adjustment（可选）
5. 将更新的任务基坐标系 `T_G_M` 回传给对应机器人

---

## 11. 离线控制台与插件系统

**包路径：** `applications/maplab-console` + `console-plugins/`

### 11.1 插件注册机制

每个插件继承 `common::Console` 插件基类，在构造时注册命令：

```cpp
addCommand({"lc", "loop_closure"},
    [this]() -> int { return runLoopClosure(); },
    "Detect and add loop closures", ...);
```

### 11.2 主要离线操作命令

| 插件 | 关键命令 | 功能 |
|------|----------|------|
| `loop-closure-plugin` | `lc`, `lc_merge_landmarks` | 回环检测 + 地标合并 |
| `map-optimization-plugin` | `optimize_vi`, `relax` | VI-map BA、位姿图松弛 |
| `map-sparsification-plugin` | `keyframe_pruning`, `sparsify_landmarks` | 地图稀疏化 |
| `map-anchoring-plugin` | `anchor_missions` | 多任务对齐 |
| `dense-reconstruction-plugin` | `stereo_dense_reconstruction`, `integrate_depth` | 稠密重建 |
| `vi-map-basic-plugin` | `load`, `save`, `ls`, `select` | 地图文件管理 |
| `pose-graph-manipulation-plugin` | `merge_maps`, `split_map` | 位姿图手工操作 |
| `rosbag-plugin` | `build_map` | 从 rosbag 构建地图 |
| `vi-map-visualization-plugin` | `visualize` | RViz 可视化 |
| `statistics-plugin` | `map_stats` | 地图统计信息 |

### 11.3 典型离线优化流程

```bash
load --map_folder=my_map
lc                          # 检测回环
lc_merge_landmarks          # 合并匹配地标
optimize_vi                 # Bundle Adjustment（含外点剔除）
retriangulate_landmarks     # 重新三角化
save --map_folder=my_map_opt
```

---

## 12. 稠密重建

**包路径：** `algorithms/dense-reconstruction/` + `console-plugins/dense-reconstruction-plugin`

### 12.1 立体稠密重建

基于 OpenCV `StereoSGBM`：
1. 从 VIMap 中读取立体相机标定（内参 + 外参）
2. 对每个立体关键帧对进行图像整流（去畸变 + 行对齐）
3. 运行 SGBM（Semi-Global Block Matching）→ 视差图
4. 深度图投影为点云
5. 可选：存储为深度资源到 map

### 12.2 深度集成（`depth_integration`）

将深度图或点云投影并累积为环境模型：
- 时间戳对齐到最近顶点（`PoseInterpolator`）
- 支持 TSDF / 散点云 两种积分后端

### 12.3 PMVS/CMVS 接口（`interfaces/pmvs-interface`）

导出地图为 PMVS2 格式，供外部多视图立体重建工具使用：
- 输出去畸变图像 + PMVS 相机文件
- 输出稀疏点云作为 PMVS 种子点

---

## 13. 地图稀疏化与摘要

**包路径：** `algorithms/map-sparsification`

### 13.1 关键帧剪枝

保留满足任意一个条件的顶点作为关键帧：
- 与上一关键帧的移动距离 > 阈值
- 与上一关键帧的旋转角度 > 阈值
- 均匀间隔（每隔 N 个顶点）
- 与最近关键帧的共视地标数 < 最小值

非关键帧被删除，其 IMU 数据拼接到前后边中。

### 13.2 地标稀疏化

**启发式贪心方法：**
- 按观测数量或描述子方差评分地标
- 贪心选择满足"每关键帧最少可见 K 个地标"约束的最优子集

**ILP（整数线性规划）方法：**
- 使用 lp_solve 库
- 二值决策变量（保留/丢弃）
- 每关键帧最少共视约束
- 最大化加权地标质量

---

## 14. 消息流系统

**包路径：** `common/message-flow`

### 14.1 架构

`MessageFlow` 是一个类型安全的异步 pub/sub 系统，将各处理模块解耦：

```cpp
// 定义 Topic
struct kImageTopic {
    using MessageType = ImageMeasurement;
    static constexpr char kTopicName[] = "IMAGE";
};

// 注册发布者
auto pub = flow->registerPublisher<kImageTopic>();

// 注册订阅者(异步回调)
flow->registerSubscriber<kImageTopic>(
    "image_processor", DeliveryOptions(),
    [](const ImageMeasurement& msg) { ... });

// 发布消息
(*pub)(image_measurement);
```

### 14.2 并发控制

- 每个订阅者有独立的 FIFO 消息队列
- `MessageDispatcherFifo`：线程池分发，基于**排他性组 (exclusivity groups)** 保证同一订阅者的消息有序投递
- 不同订阅者可并行处理

---

## 15. 坐标系约定

### 15.1 坐标系定义

| 符号 | 参考系 | 说明 |
|------|--------|------|
| `G` | Global | 全局惯性系（通常为 ENU 或任意固定系） |
| `M` | Mission | 任务坐标系，相对于 G 由 `T_G_M` 描述 |
| `I` | IMU | IMU 本体系（通常与机器人体系对齐） |
| `C` | Camera | 相机坐标系（每个相机独立） |
| `B` | Base vertex | 地标定义时所在顶点的 IMU 体系 |
| `LM` | Landmark mission | 地标所属任务坐标系 |

### 15.2 变换命名规则

`T_A_B` 表示**将向量从 B 系变换到 A 系**，即：

```
p_A = T_A_B * p_B
T_A_B = [R_A_B | p_A_AB]
```

### 15.3 四元数约定

全代码库统一使用 **JPL 四元数**（非 Hamilton）：

```
q = [x, y, z, w]   其中 w 为标量部分
```

与 Eigen 内存布局 (`coeffs()`) 保持一致。**注意：** Eigen 的 `Quaterniond(w, x, y, z)` 构造函数参数顺序不同于存储顺序。

---

## 16. 典型工作流

### 16.1 单机单会话建图

```
1. 启动 ROVIOLI（VIO 模式）
   rosrun rovioli rovioli --ncamera_calibration=... --datasource_type=rosbag ...

2. 离线后优化（maplab console）
   load --map_folder=raw_map
   lc                     # 回环检测
   lc_merge_landmarks    # 地标合并
   optimize_vi           # Bundle Adjustment
   retriangulate_landmarks
   save --map_folder=opt_map
```

### 16.2 多会话地图融合

```
# 各次会话分别建图后，在 console 中：
load_all_maps --maps_folder=sessions/
anchor_missions          # 跨任务锚定
relax                    # 位姿图松弛
lc                       # 跨任务回环
lc_merge_landmarks
optimize_vi
save_all_maps
```

### 16.3 多机器人实时建图（maplab-server）

```
# 各机器人运行：
rosrun maplab_node maplab_ros_node ... --export_map_periodically_s=60

# 服务器运行：
rosrun maplab_server_node maplab_server_node ...
# 自动接收、合并、优化子图
```

### 16.4 定位模式（ROVIOLI LOC）

```
# 先用 console 将地图优化为定位地图：
optimize_map_to_localization_map

# 然后以定位模式运行 ROVIOLI：
rosrun rovioli rovioli ... --localization_map=opt_map ...
```

---

## 附录：关键算法参数参考

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `feature_tracker_matching_threshold` | 0.8 | Hamming 比率阈值 |
| `lc_ransac_pixel_sigma` | 2.0 px | 回环 PnP RANSAC 像素误差 |
| `ba_visualterm_pixel_sigma` | 1.0 px | BA 重投影误差标准差 |
| `ba_max_reprojection_error_px` | 5.0 px | 外点剔除阈值 |
| `imu_gyro_noise_density` | sensor-specific | 陀螺仪随机游走（rad/s/√Hz） |
| `imu_acc_noise_density` | sensor-specific | 加速度计随机游走（m/s²/√Hz） |
| `kf_distance_threshold_m` | 0.2 m | 关键帧最小距离间隔 |
| `kf_rotation_threshold_deg` | 5.0° | 关键帧最小旋转间隔 |

---

*本文档自动生成于 2026-03-13，基于 maplab 2.0 代码库（commit 0b4868e）分析。*
