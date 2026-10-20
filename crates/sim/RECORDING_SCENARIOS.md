# 录制驱动的交互仿真

`scenario-extract` 从不可变 MCAP 选择场景，`scenario-run` 将同一 Runtime 的控制量送入辨识后的自车模型。其他车辆仅从录制读取初始状态，之后沿车道图按 IDM 行驶并响应自车，不再照抄录制中的未来位置。它与 `replay-drive` 的开环回放有不同用途；两者均不连接游戏输入，均不计入实车验收。

```powershell
etsads-cli scenario-extract runs/example --map region.json --policy region-policy.json --output library.json
etsads-cli scenario-run runs/example --library library.json --map region.json --policy region-policy.json --config replay-config.json --calibration vehicle.json --output artifacts/scenario-run-01
# 后续版本逐项比较上次保留的完整指标，不可省略不利样本。
etsads-cli scenario-run runs/example --library library.json --map region.json --policy region-policy.json --config replay-config.json --calibration vehicle.json --baseline artifacts/scenario-run-01/metrics.json --output artifacts/scenario-run-02
```

当前命令使用一个已导出的区域 JSON 和对应 RegionPolicy，配置必须为 `execution: "Replay"`、显式区域路线。原生 GPS 换图仍由 `replay-drive` 独立验证；此交互模型不模拟泊车换挡/接触任务。配置、标定、原始 MCAP/meta 的 SHA256 和车道图 SHA256 保存在结果中；修改源文件、替换地图或把 synthetic 来源改为 game 均会拒绝执行。

`metrics.json` 是含身份信息的基线文档，绑定全部源文件哈希、地图、完整切片窗口及配置/标定摘要。传入另一来源的同名场景、失败基线或未跑完固定窗口的结果都会被拒绝。仅更换算法代码时可直接比较；需要修改场景或模型时应另建基线，保留旧结果供审阅。

默认前窗10秒、后窗5秒。自动触发包括 `/stack` 的控制请求关闭边沿、减速度超过3 m/s²、同向前方25米内红灯接近候选，以及进入明确指定的环岛车道。`--settings settings.json` 可配置 `pre_seconds`、`post_seconds`、`roundabout_lane_ids`。红灯接近仅供筛选，不判断实际违章；没有 `/stack` 时接管次数未知，未提供环岛分类时该类别未评估。暂停、时钟回退和超过200 ms的物理采样间断分割片段，前后窗不完整会保留标记。空库不会被当作仿真通过。

自车使用标定曲率表、输入延迟、一阶转向滞后和纵向系数，20 ms步长。模型保持平路；单挂车沿已辨识铰接模型运动，其原始遥测参考保持种子中相对轴点的偏移。交通保留初始车道内横向偏移；未可靠匹配车道的车辆停止在原处，并明确计数。立交高度不同的车辆不互相作为前车。交通挂车缺少挂点参数，使用刚性组合包络；分叉使用最小航向变化，信号采用原录制时间表。这些简化是离线模型限制，不能认证城市路口或收费行为。

每个场景流式输出 `steps.jsonl`、配置与指标 `report.json`，整库输出 `metrics.json` 和 `summary.json`。故障、碰撞、缺失基准场景、指标退化均导致命令失败；失败证据仍保留。比较距离、完整时长、碰撞帧、最大减速度、横向误差、最小前方约束间距及地图覆盖；不以提前终止来获得更好数字。`minimum_agent_gap_m` 包含地图末端这一静态停止约束，并非只对车辆计算。碰撞计数为20 ms离散模型采样，不声称连续物理碰撞证明。

资源限制为每库128段、每段最长61秒、611个信号样本、200个参与者、每次车道前方查询最多32节点。超限会报错，不静默删去场景。CI 的 workspace 测试生成完整的四类触发录制 fixture，经 MCAP→ReplayAdapter→Runtime→动力学/IDM 运行，并逐项比较 `testdata/recording-baseline.json`。fixture 的环岛类别是显式合成标签；实际环岛行为由独立 Runtime 回归覆盖。CI 基准不能代替录制的真实危险场景库。
