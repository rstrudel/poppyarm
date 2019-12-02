# Poppy Ergo Jr Robot

This is a simple repository containing (Poppy Ergo Jr)[https://github.com/poppy-project/poppy-ergo-jr/] robot with a working URDF model. The robot is then plugged in (Pinocchio)[https://github.com/stack-of-tasks/pinocchio] to perform forward kinematics and self collision checking in this repository.

To visualize the robot in random collision free configuration, install the repository then run:
```
python -m poppyarm.main
```

You can visualize the robot at (http://127.0.0.1:7000/static/)[http://127.0.0.1:7000/static/] .

# Installation instructions

Clone the repository and move to the root of the folder. To install the dependencies, you can then create a conda environment with `conda env create -f environment.yml` or update the current one with `conda env update --file environment.yml` then run `pip install .` .
