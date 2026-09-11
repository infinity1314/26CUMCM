# 问题 3/4 独立优化版

本目录不修改上级目录中的原始源码和 `dist/cumcm_robot.exe`。优化版只替换两个策略钩子，
HTTP 协议、定位多边形、异常处理及最终覆盖清除仍复用原实现。

## 改动

1. 问题 3：下一测点从“信息量绝对优先”改为“保证接收与最低信息量约束下路程优先”。
   候选点必须通过原程序的严格保证接收判定；在达到最佳最坏信息量 10% 的点中选择最近点。
   若没有候选点，自动退回原最坏直径选点算法。
2. 问题 4：将 `1+6+12+12=31` 个搜索站改为 `1+5+10+12=28` 个搜索站，三个环半径分别为
   900、1400、1900 米。开路径经 2-opt 后由约 24.87 km 降到约 19.08 km。
3. 全部原有完备性兜底保留：最小包围圆、保证接收的定向测点对、19 米三角格覆盖清除。

## 验证结果

连续定向发现证书检查 129601 个单元；单元扰动上界为 9.3597 米，局部站点凸包最小边界
裕量为 31.2398 米，因此 28 站布局仍覆盖整个半径 1800 米圆域中的任意源位置和任意发射方向。

100 组配对离线随机案例结果如下。每对案例的目标数、频道、位置、接收半径和方向完全相同：

| 问题 | 原版平均虚拟时间 | 优化版平均虚拟时间 | 平均配对降幅 | 全清率 |
|---|---:|---:|---:|---:|
| 3 | 4400.2 s | 4096.6 s | 6.67% | 100/100 |
| 4 | 9224.6 s | 7885.8 s | 14.51% | 100/100 |

这些是题设分布未知条件下的离线压力测试，不是正式模拟器成绩。复现命令：

```powershell
python offline_verify_optimized.py
python benchmark_optimized.py --cases 100
```

## 运行

先由人工启动模拟器并等待接口开放，再运行：

```powershell
.\dist\cumcm_robot_optimized.exe --problem 3 --robot-id YOUR_TEAM_ID --log output\p3_optimized.jsonl
.\dist\cumcm_robot_optimized.exe --problem 4 --robot-id YOUR_TEAM_ID --log output\p4_optimized.jsonl
```

如模拟器端口不是 2026，追加 `--base-url http://127.0.0.1:端口`。

## 论文依据

- Vander Hook, Tokekar, Isler, *Cautious Greedy Strategy for Bearing-based Active Localization*,
  ICRA 2012, DOI: `10.1109/ICRA.2012.6225244`。用于保证可观测性与移动代价的折中。
- Tokekar, Isler, *Sensor Placement and Selection for Bearing Sensors with Bounded Uncertainty*,
  ICRA 2013, DOI: `10.1109/ICRA.2013.6630920`。用于楔形交集和最坏定位误差。
- Vander Hook, Tokekar, Isler, *Algorithms for Cooperative Active Localization of Static Targets
  With Mobile Bearing Sensors Under Communication Constraints*, IEEE TRO 2015,
  DOI: `10.1109/TRO.2015.2432612`。用于主动测点与任务路线联合考虑。
- Sung, Tokekar, *GM-PHD Filter for Searching and Tracking an Unknown Number of Targets With a
  Mobile Sensor With Limited FOV*, IEEE T-ASE 2021, DOI: `10.1109/TASE.2021.3073938`。
  其“最近高斯/最大方差高斯”分别对应跟踪和覆盖；本题利用其思想在已发现源精化与未知频道
  覆盖之间切换，但没有照搬需要概率先验的 GM-PHD 滤波器。
- Sung et al., *Distributed Simultaneous Action and Target Assignment for Multi-Robot Multi-Target
  Tracking*, ICRA 2018, DOI: `10.1109/ICRA.2018.8460974`。其动作与目标联合分配思想对应本程序
  的多源开路径 2-opt 服务顺序。

GM-PHD 的目标数估计没有直接加入程序：本题每个频道至多一个静止源、频道可作为确定的数据
关联标签，而且题目没有给出位置先验。引入 PHD 会增加参数和概率假设，却不能替代为“确保全部
清除”所需的确定性覆盖证书。
