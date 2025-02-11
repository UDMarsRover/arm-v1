# arm-v1
The repository for the control code associated with UDMRT's first in-house robotic arm. 

## Basic Arm Modeling
In the `basic_arm_modeling` folder, there is a URDF to define the model for our arm. There is also a simulation program with basic inverse kinematics for simple control. 

To test this, from the `arm-v1` (base) directory, first source the python virtual environment:
```bash
source .venv/bin/activate
```
Then, run the script:
```bash
python3 basic_arm_modeling/sim.py
```
In this simulation, you can drag the sliders to see what the different controls do.
- **Rotation**: Rotates the whole arm about the base.
- **Extent**: Controls the horizontal reach (x, y plane) of the arm. Greater extent will reach further.
- **Height**: Controls the height of the end effector relative to the base (z-axis).
- **Wrist**: Controls the pitch angle of the wrist relative to the horizontal plane.
- **Hand**: Controls the roll angle of the end effector relative to the wrist. 
