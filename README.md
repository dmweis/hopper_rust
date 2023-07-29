# Hopper Rust

[![codecov](https://codecov.io/gh/dmweis/hopper_rust/branch/main/graph/badge.svg)](https://codecov.io/gh/dmweis/hopper_rust)
[![Rust](https://github.com/dmweis/hopper_rust/workflows/Rust/badge.svg)](https://github.com/dmweis/hopper_rust/actions)
[![Arm cross compile](https://github.com/dmweis/hopper_rust/actions/workflows/arm-cross-compile.yml/badge.svg)](https://github.com/dmweis/hopper_rust/actions/workflows/arm-cross-compile.yml)
[![Security audit](https://github.com/dmweis/hopper_rust/workflows/Security%20audit/badge.svg)](https://github.com/dmweis/hopper_rust/actions)
[![Private docs](https://github.com/dmweis/hopper_rust/workflows/Deploy%20Docs%20to%20GitHub%20Pages/badge.svg)](https://davidweis.dev/hopper_rust/hopper_rust/index.html)

Rewrite of [Hopper's](https://github.com/dmweis/Hopper_ROS) body controller and kinematics unit in Rust

## [Click here for more images](https://davidweis.dev/robotics/2019/09/21/HopperGallery2019.html)

[![Hopper](https://github.com/dmweis/Hopper_ROS/raw/master/images/ucreate_pretty.JPG)](https://davidweis.dev/robotics/2019/09/21/HopperGallery2019.html)

## [Click here to see live CAD model!](https://davidweis.dev/robotics/2019/06/22/HopperModels.html)

[![Cad model can be viewed here](https://github.com/dmweis/Hopper_ROS/raw/master/images/hopper_cad.jpg)](https://davidweis.dev/robotics/2019/06/22/HopperModels.html)

## [Click here for videos](https://www.youtube.com/playlist?list=PL2rJqSX7Z5cFj5UM5ozf1wcm_McQg75ch)

[![Hopper climbing obstacle](https://img.youtube.com/vi/faWG_BYd5a0/0.jpg)](https://www.youtube.com/playlist?list=PL2rJqSX7Z5cFj5UM5ozf1wcm_McQg75ch)

Hopper (Named after Grace Hopper) is a 3D printed hexapod robot running on ROS.  
It's brain is a Raspberry Pi 3 running ~~Ubunut Xenial with ROS Kinetic~~ Slowly porting into pure Rust.  
The platform is modeled and 3D printed by me and is still a work in progress. It's heavily inspired by [PhantomX Hexapod from Trossen robotics](http://www.trossenrobotics.com/phantomx-ax-hexapod.aspx)

CAD design files can be found [here](https://github.com/dmweis/hopper_design)  
They may be outdated. If you'd like current file you can email me [here](mailto:dweis7@gmail.com)

## Dependencies

In case build fails with alsa-sys build stepa you want to install dev dependencies for `alsa`.  
On debian the package is called `libasound2-dev`.  

You may also need `libssl-dev` depending on which ssl library you are using.  
And `libudev` - `libudev-dev` on Ubuntu

```bash
sudo apt install libssl-dev libasound2-dev libudev-dev -y


# camera
sudo apt install libv4l-dev libclang-dev -y
```

## Audio on raspberry pi

depending on if you are running as user or system you'll want to have the following config

```shell
pcm.!default {
    type hw
    card 1
}
ctl.!default {
    type hw
    card 1
}
```

either in `~/.asoundrc` or `/etc/asound.conf`
