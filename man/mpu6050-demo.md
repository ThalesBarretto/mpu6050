---
title: mpu6050-demo
section: 1
header: User Manual
footer: mpu6050-damo 1.0
date: August 19, 2022
---

NAME
====

mpu6050-demo - Demo application of libmpu6050 driver for MPU6050

SYNOPSIS
========

`mpu6050-demo [--reset] [--calibrate] [--rate=] [--arange=] [--grange=] [--dlpf=]`

DESCRIPTION
===========

mpu6050-demo is a sample application of the libmpu6050 unofficial userpace driver for the 6-DOF Inertial Measuring Unit (IMU) MPU-6050.

It should be compiled and run in a Raspberry Pi 3B, while it is connected though the I2C inferface to a breakbout board GY-521 (or similar)  where the MPU-6050 sensor is mounted.

OPTIONS
=======

`--quiet`
: Supress output of data to the current terminal

`--reset`
: Resets the device and sets its registers and settings to safe defaults.

`--calibrate`
: Performs a very simple calibration routine that lasts about ten seconds (10s). During the calibration the sensor must be undisturbed and leveled, possibly on a flat surface.

`--rate=[50|100|200]`
: Set the sensor sampling rate in samples per second (Hz)

`--arange=[2|4|8|16]`
: Set Accelerometers range in +-G's (standard gravity multiples)

`--grange=[250|500|1000|2000]`
: Set Gyroscopes range in +-dps (degrees per second)

`--dplf=[0-6]`
:Set Embedded Digital Low Pass Filter

EXAMPLES
========

Whenever you turn the device on and off it is wise to reset and recalibrate:

`mpu6050-demo --reset --calibrate`

To read at 100Hz in the range of +-2Gs, +-250dps, no DLPF:

`mpu6050-demo --rate=100 --arange=2 --grange=250 --dlpf=0`

To get everything ready in the first run

`mpu6050-demo --reset --calibrate --rate=100 --arange=2 --grange=250 --dlpf=0`

To troubleshoot device misbehaviour:

`mpu6050-demo --reset`

BUGS
====

Please report bugs at https://github.com/ThalesBarretto/mpu6050-demo/issues

AUTHOR
======

Thales Antunes de Oliveira Barretto <thales.barretto.git@gmail.com>

LICENSE
=======

MIT License.

COPYRIGHT
========

Copyright (c) Thales Antunes de Oliveira Barretto

