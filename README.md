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
| `speed_degs_per_sec` | float | Optional | The maximum speed of the arm joints specified. A default of TODO will be used if unset. |

### Example configuration:

```
{
    "components": [
        {
        "name": "myURArm",
        "attributes": {
            "host": "10.1.10.84",
            "speed_degs_per_sec": 60,
        },
        "namespace": "rdk",
        "type": "arm",
        "model": "viam:arm:universal-robots"
        }
    ]
}
```

### Test the module

Before testing the module, make sure that your machine is connected to the Viam app, displaying as **Live** in the part status dropdown in the top right corner of the machine's page.

### Interacting with the Arm
Once the module is working you can interact with your Universal Robots arm in a couple ways:
- You can view data from and manipulate your arm live on the **CONTROL** tab of the Viam app.
For more information, see [Control Machines](https://docs.viam.com/fleet/control/).
- More advanced control of the arm can be achieved by using one of [Viam's client SDK libraries](https://docs.viam.com/components/arm/#control-your-arm-with-viams-client-sdk-libraries)

### Locally install the module
TODO

<!-- If you are using a Linux machine, and do not want to use the Viam registry, you can download the module AppImage from our servers directly to your machine:

```
sudo curl -o /usr/local/bin/viam-camera-realsense http://packages.viam.com/apps/camera-servers/viam-camera-realsense-latest-aarch64.AppImage
sudo chmod a+rx /usr/local/bin/viam-camera-realsense
```

If you need the AppImage associated with a specific tag, replace `latest` in the URL with the tag version, i.e. `0.0.X`.

Then, follow the instructions to [add a local module](https://docs.viam.com/registry/configure/#add-a-local-module) to add the local instance of the `realsense` module to your machine.
Provide an **Executable path** of `/usr/local/bin/viam-camera-realsense` when adding the module.

Or, if you aren't using the Viam app to manage your machine's configuration, modify your machine's JSON file as follows to add the `realsense` module to your machine: 

```
"modules": [
    {
        "type": "local",
        "name": "intel",
        "executable_path": "/usr/local/bin/viam-camera-realsense"
    }
],
``` -->

## Building the module
TODO

## Integration Tests
TODO

## Known supported hardware
TODO

## Linux distribution recommendation
TODO

## Troubleshooting
TODO

## Using with a Frame System
TODO

## Remaining TODOs
- configurable logging https://www.boost.org/doc/libs/1_84_0/libs/log/doc/html/log/tutorial/trivial_filtering.html
- configs
- workflows
- integration tests
- address sanitizer?
- README sections