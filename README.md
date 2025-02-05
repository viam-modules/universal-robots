# [`universal-robots` module](https://app.viam.com/module/viam/universal-robots)

This repo is a [module](https://docs.viam.com/registry/#modular-resources) that implements the [`rdk:component:arm` API](https://docs.viam.com/components/arm/) resource to allow control over [Universal Robots](https://www.universal-robots.com/) arms

## Configuration and Usage

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/build/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in [the Viam app](https://app.viam.com/).
[Add `arm / universal-robots` to your machine](https://docs.viam.com/build/configure/#components).

On the new component panel, copy and paste the following attribute template into your camera’s attributes field, editing the attributes as applicable:

```json
{
    "host": "10.1.10.84",
    "speed_degs_per_sec": 60,
}
```

> [!NOTE]  
> For more information, see [Configure a Machine](https://docs.viam.com/manage/configuration/).

### Attributes

The following attributes are available for `viam:universal-robots` arms:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `host` | string | **Required** | The IP address of the robot arm, specified as a string. |
| `speed_degs_per_sec` | float | **Required** | The maximum speed of the arm joints. |
| `acceleration_degs_per_sec2` | float | **Required** | The maximum acceleration of the arm joints. |

### Example configuration:

```
{
    "components": [
        {
            "name": "myURArm",
            "attributes": {
                "host": "10.1.10.84",
                "speed_degs_per_sec": 120,
                "acceleration_degs_per_sec2": 8
            },
            "namespace": "rdk",
            "type": "arm",
            "model": "viam:arm:universal-robots"
        }
    ]
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
- mac support
- integration tests
- address sanitizer support
