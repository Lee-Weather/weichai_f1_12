[English](README.md) | 中文

# 人形机器人运动控制：智元 X1 与 Weichai F1

本仓库基于智元 X1 开源运动控制代码，使用 Isaac Gym 进行强化学习训练。目前注册了两个任务：原有的 `x1_dh_stand`，以及 `weichai_f1_12_dh_stand`。

Weichai F1 任务沿用 X1 的训练与控制框架，并为 Weichai F1 自主设计、接入了 12DOF 下肢。F1 使用独立的机器人模型资源和任务配置，环境实现复用 X1 的 12DOF 行走环境。本仓库提供仿真和策略训练流程，**不包含完整的真机/AimRT 部署接口**。

## 机器人与任务

| 任务名 | 机器人模型资源 | 环境/配置 |
| --- | --- | --- |
| `x1_dh_stand` | `resources/robots/x1/` | `humanoid/envs/x1/` |
| `weichai_f1_12_dh_stand` | `resources/robots/weichai_f1_12/` | `humanoid/envs/weichai_f1_12_config.py` |

F1 模型资源当前版本为 **v1.1**，已重建小腿碰撞凸包，为踝关节带动脚掌运动的范围留出间隙。详见[模型说明与几何验证](resources/robots/weichai_f1_12/README.md)。

F1 当前控制 12 个下肢关节，顺序为左腿 6 个、右腿 6 个；每条腿依次为髋俯仰、髋滚转、髋偏航、膝俯仰、踝俯仰、踝滚转。Isaac Gym 使用 URDF，MuJoCo sim2sim 使用 MJCF。修改模型时，需要保持关节名、关节顺序、执行器顺序和配置数组一致。

F1 当前默认关节角为零位；观测、控制、奖励和 PPO 设置暂时继承 X1，作为初始训练基线，尚未针对真机完成标定。策略需要针对所选任务和机器人模型训练，不能默认 X1 策略可直接控制 F1。

仓库包含一段[运动学参考对比视频](doc/reference_gait_comparison.mp4)，用于展示关节轨迹；它不是训练策略或物理仿真验证结果。

## 环境与安装

本项目使用较旧的 Isaac Gym 技术栈。仓库原有配置为 Python 3.8、PyTorch 1.13.1/CUDA 11.7、NumPy 1.23.5 和 Isaac Gym Preview 4。Isaac Gym 需要受支持的 NVIDIA GPU/Linux 环境。项目 Python 包还固定安装 MuJoCo 2.3.6 及其他依赖。

1. 创建并启用环境，安装 PyTorch 和 NumPy：

   ```bash
   conda create -n humanoid python=3.8
   conda activate humanoid
   conda install pytorch==1.13.1 torchvision==0.14.1 torchaudio==0.13.1 pytorch-cuda=11.7 -c pytorch -c nvidia
   conda install numpy=1.23.5
   ```

2. 从 NVIDIA 安装 Isaac Gym Preview 4，并在它的 `isaacgym/python` 目录执行：

   ```bash
   pip install -e .
   ```

   可运行 Isaac Gym 示例（例如 `1080_balls_of_solitude.py`）检查安装。

3. 回到本仓库根目录安装项目：

   ```bash
   pip install -e .
   ```

## 训练和运行策略

以下命令均从仓库根目录运行。通过任务名选择 X1 或 F1。

### 训练

```bash
python humanoid/scripts/train.py --task=weichai_f1_12_dh_stand --run_name=baseline --headless
```

训练 X1 时使用 `--task=x1_dh_stand`。常用参数包括 `--num_envs`、`--max_iterations`、`--seed` 和 `--run_name`。检查点保存在：

```text
logs/<experiment_name>/exported_data/<时间戳><run_name>/model_<迭代次数>.pt
```

F1 的 experiment name 为 `weichai_f1_12_dh_stand`。

### 播放训练好的检查点

```bash
python humanoid/scripts/play.py --task=weichai_f1_12_dh_stand --load_run=<训练目录名> --checkpoint=-1
```

`<训练目录名>` 是 `logs/weichai_f1_12_dh_stand/exported_data/` 下的目录名；`--checkpoint=-1` 表示选该训练目录中的最新检查点。不传 `--load_run` 时，使用任务配置默认的最新训练目录。`play.py` 使用 Isaac Gym viewer；连接了手柄时会读取手柄输入。

### 导出策略

从训练目录导出 DH JIT 策略：

```bash
python humanoid/scripts/export_policy_dh.py --task=weichai_f1_12_dh_stand --load_run=<训练目录名> --checkpoint=-1
```

输出位于 `logs/weichai_f1_12_dh_stand/exported_policies/<时间戳>/policy_dh.jit`。如需从已导出的 JIT 策略生成 ONNX：

```bash
python humanoid/scripts/export_onnx_dh.py --task=weichai_f1_12_dh_stand --load_run=<导出目录名> --checkpoint=-1
```

`<导出目录名>` 是 `logs/weichai_f1_12_dh_stand/exported_policies/` 下的时间戳目录。
ONNX 文件输出到 `logs/weichai_f1_12_dh_stand/exported_onnx/<时间戳>/`。

### MuJoCo sim2sim

先导出 DH JIT 策略，再运行：

```bash
python humanoid/scripts/sim2sim.py --task=weichai_f1_12_dh_stand
```

默认从 `logs/<任务名>/exported_policies/` 选择最新的策略目录。指定其他导出版本时，使用 `--load_model <导出目录名>`。F1 的 MJCF 文件为 `resources/robots/weichai_f1_12/mjcf/weichai_f1_12.xml`；sim2sim 不会直接读取 URDF。

sim2sim 是 MuJoCo 策略仿真，不是真机控制器。用于真机前，需确认模型、关节映射、增益、限位和策略行为。

## 添加或修改机器人模型

1. 将机器人 URDF 和它引用的 meshes 放到 `resources/robots/<机器人名>/`。
2. 为 MuJoCo 准备 MJCF；sim2sim 读取 MJCF，不直接读取 URDF。
3. 在 `humanoid/envs/` 下添加任务配置，设置资源路径、默认关节角、增益、刚体名称及任务参数。
4. 在 `humanoid/envs/__init__.py` 注册任务。
5. 确保 URDF/MJCF 的关节和执行器顺序与策略动作顺序、配置数组一致。

## 目录结构

```text
humanoid/
  algo/                 PPO 算法与策略网络
  envs/                 基础环境、X1 任务和 F1 配置
  scripts/              训练、播放、策略导出和 MuJoCo sim2sim
  utils/                任务注册与公共工具
resources/robots/       X1 和 Weichai F1 的 URDF、meshes、MJCF
logs/                   运行时生成的检查点和导出文件
doc/                    文档资源
```

## 参考项目

- [legged_gym](https://github.com/leggedrobotics/legged_gym)
- [rsl_rl](https://github.com/leggedrobotics/rsl_rl)
- [humanoid-gym](https://github.com/roboterax/humanoid-gym)
