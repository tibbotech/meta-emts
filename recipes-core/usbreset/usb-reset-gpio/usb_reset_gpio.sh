#!/bin/sh

echo 2 > /sys/class/gpio/export
echo "out" > /sys/class/gpio/P0_02/direction
echo 1 > /sys/class/gpio/P0_02/value
sleep 0.5
echo 0 > /sys/class/gpio/P0_02/value
sleep 0.5
echo 1 > /sys/class/gpio/P0_02/value
