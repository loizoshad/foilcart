#!/bin/bash

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000 loopback on
sudo ip link set can0 up
