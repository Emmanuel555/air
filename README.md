# air

## Connecting the TROne to Pixhawk 2.1

The Teraranger One (TROne) communicate to the Pixhawk 2.1 via I2C Bus. There are 3 buses on the Pixhawk, Bus 1 (in GPS 1), Bus 2 (in I2C 2) and Bus 3 (in GPS 2). 

Since the Pixhawk is currently programmed to only scan I2C Bus 1, we will connect the TROne to the I2C pins inside the GPS 1 port.

This TROne is used for native altitude filtering by the Pixhawk.

*Note the altitude is only published as local_position_z_ned when the UAV is armed in altitude, position control, loiter, auto or offboard mode.

## Telemetry Radio

The companion computer link to the Pixhawk occupies the default TELEM2 port assigned for telemery radio. A quick workaround is to connect the radio to TELEM1 port instead.

A new startup script have to be added at `\etc\extras.ext\` to change the baudrate and rates of the telemetry radio.

## TROne I2C Address 

The I2C address of the TROne can be changed to the following convention

    Front
(35) --- (30)
(34) --- (31)
      -
      -
      -
(33) --- (32)

## Pixhawk 2.1 Amendments for Offboard Control

```
# std of xy-pose estimate
param set LPE_VIS_XY 0.1

# somehow needed for fusion
param set FAKE_ORIGIN 1

# fuse vision yaw
param set ATT_EXT_HDG_M 1
param set ATT_W_HDG 1.0
```

## ROS-MATLAB Setup

Edison needs the following configuration for MATLAB to resolve the DNS correctly.

In `.bashrc`, add the following lines

```
# replace with your edison ip accordingly
export ROS_IP=192.168.0.106
export ROS_MASTER_URI=http://192.168.198.106:11311
 ```



