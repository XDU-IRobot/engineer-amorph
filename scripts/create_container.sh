#!/bin/bash
sudo docker run --privileged --net=host --name amorph_container -v /home/nvidia:/home/nvidia -it ros:jazzy