# [`universal-robots` module](https://app.viam.com/module/viam/universal-robots)

This repo is a [module](https://docs.viam.com/registry/#modular-resources) that implements the [`rdk:component:arm` API](https://docs.viam.com/components/arm/) resource to allow control over [Universal Robots](https://www.universal-robots.com/) arms. Currently the following models are supported:

- UR3e, as `viam:universal-robots:ur3e`
- UR5e, as `viam:universal-robots:ur5e`
- UR20, as `viam:universal-robots:ur20`

## Configuration and Usage

This model can be used to control a universal robots arm from a machine running a viam-server. We recommend that the machine running the viam-server is using a wired ethernet connection for the best performance of this module. The following attribute template can be used to configure this model:

```json
{
    "host": <arm ip address string>,
    "speed_degs_per_sec": <float>,
    "acceleration_degs_per_sec2": <float>
}
```

### Attributes

The following attributes are available for `viam:universal-robots` arms:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `host` | string | **Required** | The IP address of the robot arm on your network. Find this when setting up your UR5e. |
| `speed_degs_per_sec` | float | **Required** | Set the maximum desired speed of the arm joints in degrees per second. |
| `acceleration_degs_per_sec2` | float | **Required** | Set the maximum desired acceleration of the arm joints. |
| `reject_move_request_threshold_deg` | float | Not Required | Rejects move requests when the difference between the current position and first waypoint is above threshold |
| `robot_control_freq_hz` | float | Not Required | Sets the processing frequency for communication with the arm in cycles/second. If the machine running this model is using WiFi, we recommend configuring this to a lower frequency, such as 10 Hz. **Default 100 Hz** |

### Example configuration:

#### using a wired ethernet connection on the same network as the arm
```json
{
    "host": "10.1.10.84",
    "speed_degs_per_sec": 120,
    "acceleration_degs_per_sec2": 8
}
```
#### using a WiFi connection on the same network as the arm
```json
{
    "host": "10.1.10.84",
    "speed_degs_per_sec": 120,
    "acceleration_degs_per_sec2": 8,
    "robot_control_freq_hz": 10
}
```

### DoCommand

#### set_acc DoCommand

`set_acc` will update the `acceleration_degs_per_sec2` maximum acceleration for joints, in deg/sec^2. The value will reset back to the configured maximum when the arm is reconfigured.

```json
{
  "set_acc": <float>
}
```

#### set_vel DoCommand

`set_vel` will update the `speed_degs_per_sec` maximum speed for joints, in deg/sec. The value will reset back to the configured maximum when the arm is reconfigured.

```json
{
  "set_vel": <float>
}
```

#### get_tcp_forces_{base,tool} DoCommand

`get_tcp_forces_base` and `get_tcp_forces_tool` will return the current forces and torques on the end effector, if available. The forces are measured in Newtons(N) and torques are measured in Newtonmeters(Nm). Measurements are at the tool flange with the orientation of the arm's base or tool coordinate system, respectively.

```json
{
  "get_tcp_forces_base": ""
}
```

example output:
```json
{
  "TRy_Nm": -0.002283915120630059,
  "Fx_N": 2.860603275894862,
  "Fz_N": -1.7602239771400954,
  "TRz_Nm": 0.030823427258229515,
  "Fy_N": 2.0520459818874928,
  "TRx_Nm": -0.08206897295825417
}
```


#### clear_pstop DoCommand

`clear_pstop` will clear an active protective stop on the arm. If there is no active protective stops or the arm is disconnected, this will throw an error.

```json
{
  "clear_pstop": ""
}
```

#### is_controllable_state DoCommand

`is_controllable_state` returns a boolean that informs a user whether the arm is in a controllable state and ready to receive commands.

```json
{
  "is_controllable_state": ""
}
```

#### get_state_description DoCommand

`get_state_description` returns the description of the arm's current state, such as whether the arm is controllable or whether the arm is in a stopped or disconnected state.

```json
{
  "get_state_description": ""
}
```

### Interacting with the Arm
First ensure that your machine is displaying as **Live** on the Viam App. Then you can interact with your Universal Robots arm in a couple ways:
- To simply view data from and manipulate your arm, use the **CONTROL** tab of the Viam App.
For more information, see [Control Machines](https://docs.viam.com/fleet/control/).
- More advanced control of the arm can be achieved by using one of [Viam's client SDK libraries](https://docs.viam.com/components/arm/#control-your-arm-with-viams-client-sdk-libraries)

## Building and Running Locally
Clone this repository to your machine and from the newly created folder start a new Docker container using the following commands:

```
git submodule update --init
docker pull ghcr.io/viam-modules/universal-robots:amd64
docker run --net=host --volume .:/src -e APPDIR='/src' -it ghcr.io/viam-modules/universal-robots:amd64
```

Once inside the docker container build the binary using:

```
cd src && make appimages
```

Then, follow the instructions to [add a local module](https://docs.viam.com/registry/configure/#add-a-local-module) to add the local instance of the `universal-robots` module to your machine.
Provide an **executable path** pointing toward the appropriate AppImage file which will have been created within the `packaging/appimages/deploy` subdirectory.

> [!NOTE]
> Simply running `make` instead of `make appimages` is a faster way to build but creates an executable that must be run from within the Docker container.  If you are interested in making and testing many quick changes to this module it will likely be faster to ony build this way and then run `viam-server` from within the Docker container, pointing to a config file that has a local instance of this module wih an **executable path** of `src/build/universal/robots`.  To download and run `viam-server` this way run:
> ```
> wget https://storage.googleapis.com/packages.viam.com/apps/viam-server/viam-server-stable-x86_64
> chmod +x ./viam-server-stable-x86_64
> ./viam-server-stable-x86_64 -config {PATH_TO_VIAM_CONFIG}
> ```

## Troubleshooting
If the module fails to start successfully look through the error logs coming from `viam-server`.  Some common issues errors are shown below along with solutions
| Error    | Solution |
| -------- | ------- |
| "Command is not allowed due to safety reasons..." | Use the UR pendant to switch the robot from Local Control to Remote Control |
| "Did not receive answer from dashboard server in time..." | It is possible that the host address provided is not correct, or that the E-stop has been pressed on the UR pendant |

## Future Work
- integration tests
- address sanitizer support
