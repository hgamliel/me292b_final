# quadsim
Simulation environment for quadrupedal robots.

## Installation
1. Install a Python environment control package such as Anaconda or Miniconda. For MATLAB users, please install MATLAB before continuing doing the following steps.
2. Create a new Python environment by
    ```
    conda create -n quadsim python=3.8
    ```
3. Once the environment `quadsim` is created, activate it by
    ```
    conda activate quadsim
    ```
4. Install dependencies required by the package by
    ```
    pip install -r requirements.txt
    ```
5. Install the package by
    ```
    pip install -e .
    ```
6. For MATLAB users, install MATLAB Python engine by
    ```
    pip install matlabengine
    ```

## Usage
1. Dive into the project directory by
    ```
    cd project
    ```
2. Run the simulation by
    ```
    python run.py
    ```
You will see an A1 quadrupedal robot standing up and falling down as the dummy controller outputs zero torques to the robot. You can then take a look at `control.py` under the current directory. The types, dimensions, units, etc. of the inputs and outputs of the control function are covered in the comments in detail. The default control time step `DT` is defined in `run.py` and you should not use a time step greater than 0.005 seconds, meaning that your control frequency should always be larger than 200 Hz. The duration for the simulation is defined by `SIM_DURATION`. The simulation uses the Python control function by default.

For MATLAB users, simply change the `USE_MATLAB` flag in `run.py` to `True` and then write your code in `control.m`. Run the simulation by `python run.py` in the same way as with the Python controller. The types, dimensions, units, etc. of the inputs and outputs of the control function in MATLAB are identical to those in Python. Please also refer to `control.m` for detailed information.

The simulation accepts either torques or positions as inputs to the robot. Please see `control.py` or `control.m` for examples.

## Extra point about compilation
If you use Windows, please make sure that you install [Microsoft Visual Studio Build Tools](https://deepakjogi.medium.com/how-to-install-pybullet-physics-simulation-in-windows-e1f16baa26f6) before running the commands above.
