English | [中文](README.zh_CN.md)

# Humanoid Locomotion: AgiBot X1 and Weichai F1

This repository contains an Isaac Gym reinforcement-learning project derived from the open-source AgiBot X1 locomotion codebase. It currently registers two tasks: the original `x1_dh_stand` task and `weichai_f1_12_dh_stand`.

The Weichai F1 task uses the X1 training and control framework with a custom-designed 12DOF lower body for the Weichai F1 robot. Its robot assets and task configuration are separate from X1's, while its environment implementation reuses the X1 12DOF walking environment. The repository provides simulation and policy-training workflows; it does not include a complete real-robot/AimRT deployment interface.

## Robots and tasks

| Task | Robot assets | Environment/configuration |
| --- | --- | --- |
| `x1_dh_stand` | `resources/robots/x1/` | `humanoid/envs/x1/` |
| `weichai_f1_12_dh_stand` | `resources/robots/weichai_f1_12/` | `humanoid/envs/weichai_f1_12_config.py` |

F1 model resources are **v1.1**, with calf collision hulls rebuilt to clear the ankle-foot sweep. See the [model notes and geometry validation](resources/robots/weichai_f1_12/README.md).

The F1 robot has 12 controlled leg joints, ordered as six left-leg joints followed by six right-leg joints: hip pitch, hip roll, hip yaw, knee pitch, ankle pitch, and ankle roll on each side. Isaac Gym uses the URDF; MuJoCo sim2sim uses the MJCF. Keep joint names, joint order, actuator order, and the configuration arrays aligned when changing the model.

F1 currently uses a zero-angle nominal joint pose and inherits X1 observation, control, reward, and PPO settings as an initial training baseline. These settings have not been calibrated for real hardware. A policy must be trained for the selected task and robot model; an X1 policy should not be assumed to control F1 correctly.

The repository includes a [kinematic reference comparison video](doc/reference_gait_comparison.mp4). It visualizes joint trajectories and is not a trained-policy or physics-validation result.

## Requirements and installation

This is a legacy Isaac Gym project. The configuration documented by the source project is Python 3.8, PyTorch 1.13.1 with CUDA 11.7, NumPy 1.23.5, and Isaac Gym Preview 4. Isaac Gym requires a supported NVIDIA GPU/Linux setup. The Python package also pins MuJoCo 2.3.6 and installs the project's supporting libraries.

1. Create and activate the environment, then install PyTorch and NumPy:

   ```bash
   conda create -n humanoid python=3.8
   conda activate humanoid
   conda install pytorch==1.13.1 torchvision==0.14.1 torchaudio==0.13.1 pytorch-cuda=11.7 -c pytorch -c nvidia
   conda install numpy=1.23.5
   ```

2. Install Isaac Gym Preview 4 from NVIDIA. In its `isaacgym/python` directory, run:

   ```bash
   pip install -e .
   ```

   Check the Isaac Gym installation by running one of its examples, such as `1080_balls_of_solitude.py`.

3. From this repository's root directory, install this project:

   ```bash
   pip install -e .
   ```

## Train and run a policy

Run the commands below from the repository root. Replace the task name to select X1 or F1.

### Train

```bash
python humanoid/scripts/train.py --task=weichai_f1_12_dh_stand --run_name=baseline --headless
```

For X1, use `--task=x1_dh_stand`. The main options include `--num_envs`, `--max_iterations`, `--seed`, and `--run_name`. Checkpoints are written under:

```text
logs/<experiment_name>/exported_data/<timestamp><run_name>/model_<iteration>.pt
```

F1's experiment name is `weichai_f1_12_dh_stand`.

### Play a trained checkpoint

```bash
python humanoid/scripts/play.py --task=weichai_f1_12_dh_stand --load_run=<run-folder> --checkpoint=-1
```

`<run-folder>` is the directory name under `logs/weichai_f1_12_dh_stand/exported_data/`. `--checkpoint=-1` selects the latest checkpoint in that run. Omit `--load_run` to use the task configuration's default latest run. The play script uses Isaac Gym's viewer and will use a connected joystick if one is available.

### Export a policy

Export the task's DH JIT policy from a training run:

```bash
python humanoid/scripts/export_policy_dh.py --task=weichai_f1_12_dh_stand --load_run=<run-folder> --checkpoint=-1
```

The output is saved under `logs/weichai_f1_12_dh_stand/exported_policies/<timestamp>/policy_dh.jit`. To export ONNX from an exported JIT policy, run:

```bash
python humanoid/scripts/export_onnx_dh.py --task=weichai_f1_12_dh_stand --load_run=<export-folder> --checkpoint=-1
```

Here `<export-folder>` is a timestamp directory under `logs/weichai_f1_12_dh_stand/exported_policies/`.
The ONNX file is written under `logs/weichai_f1_12_dh_stand/exported_onnx/<timestamp>/`.

### MuJoCo sim2sim

Export the DH JIT policy first, then run:

```bash
python humanoid/scripts/sim2sim.py --task=weichai_f1_12_dh_stand
```

By default, sim2sim selects the latest policy directory under `logs/<task>/exported_policies/`. To select a specific export, pass its directory name with `--load_model <export-folder>`. The F1 MJCF is `resources/robots/weichai_f1_12/mjcf/weichai_f1_12.xml`; sim2sim does not load the URDF directly.

The sim2sim script is a MuJoCo policy simulation, not a hardware controller. Verify the model, joint mapping, gains, limits, and policy behavior before any real-robot use.

## Adding or changing a robot model

1. Place the robot's URDF and referenced meshes under `resources/robots/<robot_name>/`.
2. Provide an MJCF for MuJoCo simulation. The sim2sim script reads MJCF, not URDF.
3. Add a task configuration under `humanoid/envs/` with asset paths, default joint angles, gains, body names, and task-specific settings.
4. Register the task in `humanoid/envs/__init__.py`.
5. Keep the URDF/MJCF joint and actuator order consistent with the policy's action order and the configuration arrays.

## Repository layout

```text
humanoid/
  algo/                 PPO algorithms and policy networks
  envs/                 Base environment, X1 task, and F1 configuration
  scripts/              train, play, policy export, and MuJoCo sim2sim
  utils/                task registry and shared utilities
resources/robots/       X1 and Weichai F1 URDF, meshes, and MJCF
logs/                   generated checkpoints and exports (created at runtime)
doc/                    documentation assets
```

## References

- [legged_gym](https://github.com/leggedrobotics/legged_gym)
- [rsl_rl](https://github.com/leggedrobotics/rsl_rl)
- [humanoid-gym](https://github.com/roboterax/humanoid-gym)
# weichai_f1_12
