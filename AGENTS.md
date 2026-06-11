# Codex context for /home/lewiski/local_catkin

默认使用中文回答。

这个目录是一个用于分析代码的本地快照集合，不是一个真实的单一项目仓库。

## 工作区性质

- `PC_catkin_src/` 和 `PI_catkin_src/` 分别来自不同设备，表示不同运行端的 ROS/catkin 代码快照。
- 不要默认把 `PC_catkin_src/` 和 `PI_catkin_src/` 合并理解为同一个 catkin workspace。
- 两边出现同名 ROS 包是正常现象，通常表示两个设备各自的运行环境，而不是必然的包冲突。
- 子目录里的 `.git/`、`.vscode/`、`__pycache__/`、`*.pyc` 等可能是搬运残留。分析时可以忽略它们，除非用户明确要求检查版本历史、编辑器配置或缓存文件。

## 远程设备约定

- 用户会使用本目录作为远程操作 PC/PI 设备的工作上下文。
- PC 端对应 `PC_catkin_src/` 的来源设备，当前 SSH 目标是 `nanocar@192.168.1.4`，密码为 `DTvlp#261c`，已验证可连接；远端主机名为 `pc25130272`。
- PI 端对应 `PI_catkin_src/` 的来源设备，当前已知一台 PI 名称为 `fiona`，SSH 目标是 `vpaadmin@192.168.1.13`，密码为 `vpaduck`，已验证可连接；远端主机名为 `fiona`。
- 新增 PI 端名称 `vivian`：SSH 目标 `vpaadmin@192.168.1.3`，密码为 `vpaduck`，已验证可连接；远端主机名为 `vivian`。
- 目前用户明确允许把上述远程登录密码写入本地 `AGENTS.md`，以便新会话延续远程操作上下文。
- 后续可能增加多台 PI。新增 PI 时应按 IP、用户名、主机名和对应本地快照目录分别记录，不要假设所有 PI 的代码状态完全相同。
- 不要把 SSH 私钥、GitHub token 或其他未获明确授权的凭据写入 `AGENTS.md`、代码、脚本、提交记录或同步仓库。
- 使用 SSH 执行远程操作前，先明确目标端是 PC、某台 PI，还是两边都要操作。
- 默认优先做只读检查；写文件、安装依赖、重启节点、停止进程、修改网络配置、修改 ROS launch/service 等操作前，应说明影响并获得用户明确意图。
- 远程启动小车 ROS launch 时，不要默认使用 `nohup`、`tmux`、`screen` 等后台保持方式；用户要求“直接控制”时，应通过当前 SSH 前台会话运行和观察 launch 输出。若需要长时间持续运行，应先说明会占用当前会话，或让用户另开终端。

## 默认任务理解

- 用户通常希望 Codex 帮助理解系统架构、ROS 节点、topic/service 数据流、launch 启动关系、依赖关系和潜在代码问题。
- 不要默认尝试把整个目录整理成可构建、可发布、可提交的项目。
- 不要把根目录不是 Git 仓库、子包各自带 `.git/`、存在未提交改动等视为主要问题；这些通常只是代码来源混杂造成的。
- 如果发现 Git 冲突标记、重复包名、硬编码路径、缺失依赖等问题，可以指出它们对代码分析或实际运行的影响，但要结合“这是分析快照”的背景解释。

## 参考文档

- `demo.md` 是 DuckieRace 演示运行手册，包含系统用途、场地/摄像头/机器人准备、PC/PI 端 ROS 包要求、常用 launch/rosrun 命令、区域标定流程、GUI/joystick 操作和基础故障排查信息。
- 进行 DuckieRace 运行、调试、远程启动节点、检查配置或向用户解释操作流程前，应先参考 `demo.md`，但不要把其中命令当成已执行过的事实。
- 当前用户确认 DuckieRace 接下来需要使用 PC 端 `/dev/video10` 和 `/dev/video12` 两路摄像头图像；如启动或修改 camera launch，应优先按这两个设备节点配置。
- 2026-06-11 在 fiona 上实测可用的 DuckieRace 车端跟线参数：`vpa_robot_operation/scripts/duckierace.py` 中 `self.kp = 3.0`、`self.h_row_ratio = 0.76`、`angular_speed = np.clip(angular_speed, -0.8, 0.8)`；`vpa_robot_operation/launch/duckierace_start.launch` 中 `lane_kd` 默认值为 `0.4`。后续连接其他机器人时，若要复制 fiona 当前稳定表现，应优先同步这组参数。
- 2026-06-11 为了使用 PC 端 `keyboard_joy_console.py` 虚拟手柄，fiona 的 `vpa_robot_operation/launch/duckierace_start.launch` 已注释真实手柄 `joy_node`；后续对其他机器人做同类虚拟手柄测试时，也应禁用对应车端 launch 里的真实 `joy_node`，避免多个 publisher 同时发布 `/<robot>/joy`。
- 2026-06-11 已在 PC `nanocar@192.168.1.4` 的 `~/.bashrc` 固定 DuckieRace ROS 网络环境：`ROS_MASTER_URI=http://192.168.1.4:11311`、`ROS_IP=192.168.1.4`、`unset ROS_HOSTNAME`。这是为了避免 PC 端 ROS publisher URI 变成 `pc25130272` 导致 PI 端无法解析。
- 2026-06-11 已修改 PC 端 `vpa_duckierace/scripts/cam_zone_manager.py`：`/<robot>/in_fuel_zone` 和 `/<robot>/in_charge_gate_zone` 从只在进入/退出事件发布，改为每帧持续发布当前 True/False 状态。已同步到远端 PC 并重启 racecontrol，实测 `/fiona/in_fuel_zone` 约 60 Hz。
- 2026-06-11 用户移开机器人后，已在 PC 端重新抓取 `/usb_cam_1/image_raw` 和 `/usb_cam_2/image_raw` 标定图，运行 `vpa_duckierace/scripts/auto_zone_cali.py` 重新生成 `config/zones_cam1.yaml`、`config/zones_cam2.yaml`，同步回本地 `PC_catkin_src/vpa_duckierace/config/` 并重启 racecontrol 生效。
- 2026-06-11 已修正双顶部相机 fuel/charge gate 状态聚合：`cam_zone_manager.py` 改为发布 `/<robot>/<camera>/in_fuel_zone` 和 `/<robot>/<camera>/in_charge_gate_zone`，新增 `zone_state_aggregator.py` 做 OR 聚合后发布最终 `/<robot>/in_fuel_zone`、`/<robot>/in_charge_gate_zone`。这样同一机器人只要任一摄像机判定在加油区，RaceGUI 和车端都会收到 True，避免两个相机抢写同一 topic 导致 OK/NG 跳变。
- 2026-06-11 已为顶部相机区域判断加入车顶 AprilTag 到地面投影的像素补偿：`zones_cam1.yaml` 中 `tag_ground_offset: {dx: 0.0, dy: 47.0}`，`zones_cam2.yaml` 中 `tag_ground_offset: {dx: 0.0, dy: 42.0}`；`cam_zone_manager.py` 在 merge/fuel/charge gate/finish 判断前使用修正后的 `tag_ground_point()`。cam2 的 `dy=42` 来自用户将 fiona 放在 cam2 fuel zone y 轴中心时测得的 tag10 raw y 与 fuel zone y 中心差值。

## 修改规则

- 除非用户明确要求，不要删除 `.git/`、`.vscode/`、`__pycache__/`、日志、图片、配置或搬运残留文件。
- 除非用户明确要求，不要重构项目结构、移动包目录、合并 PC/PI 代码或统一包名。
- 如果用户要求清理，只清理其明确指定的类别，并在执行前说明会删除哪些类型的文件。
- 如果用户要求运行或构建，先确认目标端：PC 端、PI 端，或单个 ROS 包。
- 如果修改了 PC 或 PI 远端 `catkin/src` 下的代码，必须同步回本地对应目录：PC 同步到 `PC_catkin_src/`，PI 同步到 `PI_catkin_src/` 或后续指定的 PI 专属目录。
- 本地快照与远端代码的同步应保持设备端边界，不要把 PC 和 PI 的 catkin workspace 合并。
- 用户提出可以通过 GitHub 仓库同步 PC、PI 和本地快照；当前尚未创建 GitHub 仓库。创建前需要确认仓库名、私有/公开、GitHub 账号/组织以及是否允许使用本机 GitHub 凭据。
- 如果未来建立 GitHub 同步仓库，必须谨慎处理 `AGENTS.md` 中的明文远程密码；除非用户再次明确要求，否则不要把凭据、密钥、token、构建产物、ROS 日志、bag、大型临时数据和缓存文件提交到共享或公开仓库。

## Git 提交流程（本地快照仓库）

- 当前仓库实际使用 `.git_real` 目录作为 git 元数据；所有 git 操作统一使用：
  - `git --git-dir=.git_real --work-tree=/home/lewiski/local_catkin ...`
- 提交前固定顺序：
  - `git --git-dir=.git_real --work-tree=. status --short --branch`（确认有无未提交改动）
  - `git --git-dir=.git_real --work-tree=. add -A`
  - `git --git-dir=.git_real --work-tree=. commit -m "<本次说明>"`（无改动则不提交）
  - `git --git-dir=.git_real --work-tree=. pull --rebase origin main`（如果落后则更新并处理可能冲突）
  - `git --git-dir=.git_real --work-tree=. push origin main`
- `push` 前若显示落后（`behind`）或有冲突风险，先处理完 `pull --rebase` 再推送；若 `status` 仍有未提交项，先提交后再推送。
- 若环境偶发无法解析 GitHub 域名，需在允许外部联网的上下文重试对应命令。

## 分析重点

- 优先按设备端划分：PC 侧全局控制/交通灯/赛道管理，PI 侧车载底层接口/感知/控制。
- 优先梳理 ROS 通信关系：node、topic、service、msg/srv/cfg、launch include 和 namespace。
- 对运行风险的判断要区分：
  - 代码快照/搬运造成的问题；
  - ROS 部署配置问题；
  - 真正的业务逻辑或控制算法问题。
