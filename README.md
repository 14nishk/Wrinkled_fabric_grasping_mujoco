# RGBD Fabric Grasping WITH ROBOT IN MuJoCo

A deep reinforcement learning implementation for robotic fabric manipulation using RGBD sensing and dual-arm control as master thesis. The inspiration of the project credited to : [ICRA 2024 Cloth Manipulation Challenge](https://airo.ugent.be/cloth_competition/)

## Overview

This project implements a dual-arm robotic fabric manipulation system using deep reinforcement learning with RGBD (RGB + Depth) sensing. The system uses MuJoCo for physics simulation and PyTorch for deep learning.

![System Overview](docs/system_overview.png)

## Features

- Dual-arm robotic control with collision avoidance
- RGBD-based perception pipeline
- Custom reward shaping for fabric manipulation
- Real-time visualisation and monitoring
- Tensorboard integration for training metrics
- Automated video recording of training episodes
- Optuna-based hyperparameter tuning and analysis

## Architecture

The system consists of several key components:

```
Directory Structure
-------------------
- pipeline.py                : Main training pipeline script (entry point).
- diana_env.py               : Custom Gymnasium environment for the DIANA robot.
- diana_unified_controller.py: Unified controller for simulation and rendering.
- ik_solver.py               : Inverse kinematics solver for the robot arms.
- trainer.py                 : Training logic and neural network backbone.
- utils.py                   : Utility functions for reward, vision, etc.
- monkeypatch.py             : Monkeypatches for bugfixes and logging.
- capture_diana_images.py    : Script for capturing images from the simulation.
- requirements.txt           : Python dependencies.

- src/
    - diana_setup.xml        : Main MuJoCo model (included by scene.xml).
    - scene.xml              : Top-level MuJoCo scene file.
    - meshes/urdffile/       : STL/OBJ mesh files and URDF/XACRO robot descriptions.
```

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/Wrinkled_fabric_grasping_mujoco.git
cd codebase
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up MuJoCo:
```bash
# Install MuJoCo
pip install mujoco==3.2.7

# Set environment variables
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path/to/mujoco/lib
```

## Usage

1. Start training:
```bash
python codebase/pipeline.py --image_width 224 --image_height 224 --num_timesteps 1000000
```

2. Monitor training:
```bash
tensorboard --logdir ~/tensorboard_logs
```

## Training Pipeline

The training process involves several stages:

1. Environment Reset
   - Initialize robot arms
   - Place fabric in random position
   - Generate target grasp points

2. Action Execution
   - Process RGBD input
   - Generate robot actions
   - Execute movement
   - Check for collisions

3. Reward Computation
   - Distance-based rewards
   - Grasp success rewards
   - Collision penalties

## Results

Sample training results showing learning progress:
![Result 1](docs/hanging_c.png)
![Result 2](docs/lower_grasp.png)

![Training Results](resnet_training_results/learning_curve.png)

## Visualization

The system provides real-time visualization of:
- Robot state
- RGBD input
- Grasp points
- Collision warnings

![Visualization](docs/images/visualization.png)

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the CC0 1.0 Universal - see the [LICENSE](LICENSE) file for details.

## Citation

If you use this code in your research, please cite:

```bibtex
@thesis{wrinkled2025fabric,
  title={Wrinkled Fabric Grasping With A Robotic Arm Using Deep Reinforcement Learning},
  author={Kanishk Dwivedi},
  supervisor={Prof. Dr. Jianwei Zhang, Dr. Norman Hendrich und MSc Niklas Fiedler},
  year={2025}
}
```

## Contact

Kanishk Dwivedi - kanishk.dwivedi@studium.uni-hamburg.de / kanishk.dwivedi@yahoo.com

Project Link: [https://github.com/14niskh/Wrinkled-fabric-grasping](https://github.com/14nishk/Wrinkled_fabric_grasping)

