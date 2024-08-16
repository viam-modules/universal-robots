# ur5e-arm-module

## Steps to Run
Start by cloning this repository onto your local machine
```
git clone https://github.com/viamrobotics/ur5e-arm-module.git
```

Make sure to initialize the submodules (Universal Robots Client Library + trajectory generation repos) by running
```
git submodule update --init
```

Make note of the full path of this repository, you will need it later

The module needs `viam-cpp-sdk` installed alongside all of its dependencies. Therefore, to make the development process easier, we use the `viam-camera-realsense` docker image as this has `viam-cpp-sdk`, `ninja` (the build tool we use), and most dependencies. In order to run this docker image, make sure you have docker installed and run

```
sudo docker run --privileged --net=host --volume <PATH_TO_UR5E_ARM_MODULE_REPO>:/host -it ghcr.io/viamrobotics/viam-camera-realsense:amd64
```

Notes:
* This will only work on x86 machines (i.e. Framework laptops). This image is specifically meant for these architectures (hence the `amd64` tag at the end of the docker image name). If you can find a docker image with all of `viam-cpp-sdk` + dependencies preinstalled, I don't see why you couldn't build and run this executable on M-series Macs
* Replace the <PATH_TO_UR5E_ARM_MODULE_REPO> with the absolute path to your cloned repository
* `--privileged` allows something called `modprobe fuse` to run inside the docker which is necessary for running `viam-server`
* `--net=host` allows the docker image to get full access to the wifi network that your host machine has. I tried experimenting with just forwarding a single port but that didn't seem to work

Once inside the docker image, the ur5e-arm-module repo will have been mounted to `/host`. Change into this directory, and you will see all your mounted files

## Building the module inside Docker
The build tool used for this c++ module is `ninja`. It should come preinstalled in the `viam-camera-realsense` docker image. However, this docker image does not have the `Eigen3` dependency preinstalled so first run:

```
sudo apt update
sudo apt install libeigen3-dev
```

Now you should be able to run 

```
ninja all
```

which will give you an executable called `ur5earm-x86`

## Running the arm with viam-server
Follow instructions on app to get viam-server executable inside your docker (standard Linux instructions)

Create a new machine on app. Add an Arm Component as well as the Arm module by providing the absolute path of the `ur5earm-x86` executable inside your docker. 

Grab the json for your machine and inside the docker run
```
sudo ./viam-server -config=<NAME_OF_JSON>
```

This should start up your instance of `viam-server` and now you can run client scripts from inside the docker or even on your host machine to communicate with this server instance

Have fun!

