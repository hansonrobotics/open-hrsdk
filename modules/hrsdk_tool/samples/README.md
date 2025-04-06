# Hanson Robot SDK Samples

This folder contains the samples for Hanson Robot SDK. 

# Run the samples

## Run the samples in the pre-built docker container

```
docker pull 222132024866.dkr.ecr.ap-east-1.amazonaws.com/hansonrobotics/samples:kinetic
docker run --network host -it --rm \
  222132024866.dkr.ecr.ap-east-1.amazonaws.com/hansonrobotics/samples:kinetic

# then in the container we can choose one sample and run it with python
root@hostname:/samples# python actuator_control.py
```
## Run the samples on the host machine

You just need to have ROS (Kinetic/Melodic) installed on the host and source the ROS enviornment. And everything else is the same. 
