# 运行说明

程序不会启动模拟器。请先在模拟器中登录并选择问题 3 或问题 4 的演练测试，等待 5 秒倒计时结束、界面显示本机接口已经开放，然后在另一个 PowerShell 窗口运行程序。

## 1. 离线查看策略文件

以下命令只生成策略和搜索站点，不会连接模拟器：

```powershell
python robot_strategy.py --problem 3 --robot-id YOUR_TEAM_ID --write-policy output/p3_policy.json
python robot_strategy.py --problem 4 --robot-id YOUR_TEAM_ID --write-policy output/p4_policy.json
```

目录中已提供使用 `<TEAM_ID>` 占位符的 `output/p3_policy.json` 和 `output/p4_policy.json`。

## 2. 运行演练测试

将 `YOUR_TEAM_ID` 换成模拟器当前登录的参赛队号：

```powershell
python robot_strategy.py --problem 3 --robot-id YOUR_TEAM_ID --log output/p3_practice.jsonl
python robot_strategy.py --problem 4 --robot-id YOUR_TEAM_ID --log output/p4_practice.jsonl
```

程序会依次完成 `/enter`、搜索测向、定位清除和 `/exit`。每次只发送一条请求，收到反馈后才生成下一条请求。`--log` 指定日志文件名前缀，程序会自动追加运行时间，例如生成 `p3_practice_20260910_213500.jsonl`。该 JSONL 文件记录本局每一条完整请求及响应，这才是对应随机案例的实际指令序列文件。

任务结束后，终端会显示成功清除数量、总虚拟时间及“总虚拟时间 / 成功清除数量”得到的平均每个干扰源定位清除时间。

打包后的第三问和第四问程序已经分开，均不再需要 `--problem` 参数：

```powershell
.\dist\cumcm_robot_p3.exe --robot-id YOUR_TEAM_ID --log output/p3_practice.jsonl
.\dist\cumcm_robot_p4.exe --robot-id YOUR_TEAM_ID --log output/p4_practice.jsonl
```

两个 EXE 分别固定执行问题 3 和问题 4，传入 `--problem` 会被拒绝。拆分前的统一版已保存在 `backup/25_point_dual_ring_unified/`。

如果模拟调试器端口不是 2026，例如改为了 2027，则增加：

```powershell
--base-url http://127.0.0.1:2027
```

网络超时重试会复用完全相同的请求内容和 `request_id`，不会重复执行已经被模拟器接受的动作。

## 3. 离线验证

以下命令只执行本地几何和状态机测试，不连接模拟器：

```powershell
python offline_verify.py
```

正式测试结束后，还需从模拟器导出加密 `.jlog` 文件且不要修改模拟器生成的文件名。本文交付没有启动任何演练或正式测试。
