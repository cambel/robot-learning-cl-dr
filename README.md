# Gazebo Gym Environment for learning force control with peg insertion tasks (Universal Robots)

This repository is based on our public repositories [ur3](https://github.com/cambel/ur3) and [ur_openai_gym](https://github.com/cambel/ur_openai_gym).

## Installation
Tested on ROS Melodic - Ubuntu 18.04

Recommended installation using Docker: follow [wiki](https://github.com/cambel/robot-learning-cl-dr/wiki/Install-with-Docker) instructions
## Usage: Peg-In-Hole tasks

### Training

On one terminal, launch a gazebo environment for an specific peg:
```shell
roslaunch ur3_gazebo ur_peg.launch peg_shape:=cuboid
```
valid shapes [`cylinder`, `hexagon`, `cuboid`, `triangular`, `trapezoid`, `star`]

On a different terminal, execute the training script:
```shell
rosrun ur3e_rl tf2rl_sac.py -e0
```
The training should start: 
- The robot goes to a given initial position, 
- a taskboard spawned randomly within a defined workspace, 
- the robot moves towards the `goal pose`, defined with respect to the current pose of the taskboard
- the training lasts 100,000 time steps.

There are 3 sample environments defined in the training script `tf2rl_sac.py`
<ol start="0">
  <li>For a training environment with a single peg, which does not change during the entire training session.</li>
  <li>For a training environment with multiple pegs, the entire robot is reset when the peg is changed. **Note** in this case the simulation tends to be unstable without a powerful CPU/GPU. The `real time factor` start degrading with time until it barely moves.</li>
  <li> A sample configuration for training/testing with a real robot.</li>
</ol>

The hyper-parameters of each environment is defined in a YAML config file, which can be found on the package`ur3e_rl/config`

After a training session is started, training data will be store in a folder `results/[datetime]_**dir_suffix**` under the current directory. More information can be found in the RL repository [tf2rl](https://github.com/keiohta/tf2rl)

### Testing
A learned policy can be executed using the helper script `evaluate_policy.py`:
```shell
rosrun ur3e_rl evaluate_policy.py -n 50 results/[datetime]_**dir_suffix**
```
Useful parameters of this script
- `--training`: actions are taken similarly as during training, with a randomness to it, i.e. a sample from a Gaussian distribution. Without this, the actions are taken as the mean of the Gaussian distribution, a lot more stable behavior.
- `-n`: number of test to execute
- `-s`: store/record observations from tests.  


## References

If you find this code useful, consider citing our work:
```cite
@article{
  author = {Beltran-Hernandez, Cristian C. and Petit, Damien and Ramirez-Alpizar, Ixchel G. and Harada, Kensuke},
  title = {Accelerating Robot Learning of Contact-Rich Manipulations: A Curriculum Learning Study},
  doi = {10.48550/ARXIV.2204.12844},
  url = {https://arxiv.org/abs/2204.12844},
  publisher = {arXiv},
  year = {2022},
  copyright = {Creative Commons Attribution Non Commercial No Derivatives 4.0 International}
}
```
ArXiv: https://arxiv.org/abs/2204.12844

Supplemental Video: https://youtu.be/_FVQC5OcGjs
