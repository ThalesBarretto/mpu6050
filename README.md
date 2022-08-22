# MPU6050 demo application

This is a demo application for the libmpu6050 driver.

You can attach the sensor to the RPi, install, and run the demo.

## What you can do with it

- Get raw sensor data
- Get orientation estimation from sensor data

## Building, installing, removing

You must build inside the Raspberry Pi 3.

0. Install the depedendencies:

`libi2c` - runtime dependecy
`libi2c-dev` - make dependecy
`pandoc` - make dependency for documentation

```bash
pi@pi ~ $> sudo apt update 
pi@pi ~ $> sudo apt install libi2c libi2c-dev pandoc
```

1. Clone with `--recurse-submodules`
2. Make and install the modules
3. Make and install the application
4. You should have installed limbpu6050, libmtx and mpu6050-demo

```bash
pi@pi ~ $> mkdir repo && cd repo
pi@pi ~/repo $> git clone --recurse-submodules https://github.com/ThalesBarretto/mpu6050.git
pi@pi ~/repo $> cd mpu6050
pi@pi ~/repo/mpu6050 $> make modules 
pi@pi ~/repo/mpu6050 $> make modules_install
pi@pi ~/repo/mpu6050 $> make
pi@pi ~/repo/mpu6050 $> make install
```

5. To remove:

```bash
pi@pi ~/repo/mpu6050 $> make modules_uninstall
pi@pi ~/repo/mpu6050 $> make uninstall
```

## Running

Example

```bash
$> mpu6050-demo --reset --calibrate --arange=2 --grange=250 --dlpf=0 --rate=100
```

**NOTE**: While calibrating, let sensor rest leveled for approximately 10s.


Useful command line options:

- `--reset` Reset sensor settings and calibration

- `--arange=[2|4|8|16]` Set Accelerometers range in +-G's (standard gravity multiples)

- `--grange=[250|500|1000|2000]` Set Gyroscopes range in +-dps (degrees per second)

- `--dplf=[0-6]` Set Embedded Digital Low Pass Filter

- `--rate=[50|100|200]` - Set Sampling Rate in Hz (samples per second)

- `--calibrate` Perform a very basic calibration routine

- `--quiet` Supress output to local terminal


## Contributing

Contributions are welcome on github, on three channels:

* Open an [issue](<https://github.com/ThalesBarretto/mpu6050/issues>).
* Start or enter a [discussion](<https://github.com/ThalesBarretto/mpu6050/discussions>).
* Send a [pull request](<https://github.com/ThalesBarretto/mpu6050/pulls>).
