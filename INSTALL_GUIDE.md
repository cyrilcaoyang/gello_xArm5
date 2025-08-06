# GELLO Installation Guide

Simple installation using `pyproject.toml`.

## Installation

```bash
git clone https://github.com/wuphilipp/gello_software.git
cd gello_software
git submodule init && git submodule update

# Install GELLO (includes all robot support)
pip install -e .

# Install DynamixelSDK for GELLO device
pip install -e third_party/DynamixelSDK/python
```

## Development Installation

```bash
# Install with development tools (black, flake8, pytest, etc.)
pip install -e ".[dev]"
```

## What's Included

**Core installation includes:**
- All robot support (xArm, UR, Franka)
- GELLO device support (Dynamixel)
- Camera support (Pillow, etc.)
- Simulation support (dm_control, pygame)
- Math utilities (quaternions)
- All required dependencies

**Development installation adds:**
- Code formatting (black, isort)
- Linting (flake8, mypy, pyright)
- Testing (pytest)
- Debugging (ipdb)
- Interactive development (jupyterlab)

## Platform Notes

- **RealSense cameras**: May require manual installation: `pip install pyrealsense2`
- **macOS**: All dependencies should work out of the box
- **Linux**: May need system packages for cameras/GUI
- **Windows**: Use appropriate Python distribution

## Verification

```python
# Test installation
import gello
from gello.robots.xarm5_robot import XArm5Robot
from gello.robots.dynamixel import DynamixelRobot
print("GELLO installed successfully!")
```

## Troubleshooting

**Import errors:**
```bash
# Make sure you're in the project directory
cd /path/to/gello_software
pip install -e .
```

**Camera issues:**
```bash
# Install RealSense separately if needed
pip install pyrealsense2
``` 