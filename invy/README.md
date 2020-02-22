Inverse Kinematics
==============================

### How to interact with the controller
Once you launched the application in [**app/scripts**](/app/scripts), establish a RPC communication with the controller by doing:
```sh
$ yarp rpc /tutorial_inverse-kinematics-controller/cmd:rpc
```
Then, you can operate through the commands listed below.

#### Changing target position
```sh
>> target x y
```
where `x` and `y` are the new Cartesian coordinates of the target in the range **[-250,250]**.

#### Changing IK algorithm
```sh
>> mode m
```
where `m` is a string specifying the new mode among the following options:
- `t` for the Jacobian Transpose
- `inv` for the Jacobian (Pseudo-)inverse



