# Installation

## Ubuntu

Prebuilt packages are available from the official Ubuntu archive and the
[ROS repository](http://packages.ros.org/).
If you are using ROS Noetic on Ubuntu 20.04 or later, install **catkin_lint** with
```sh
$ sudo apt install catkin-lint
```

!!! note
    Starting with Ubuntu 20.04, the package has been renamed. If you are using an older
    release, please run `sudo apt install python-catkin-lint` instead.


Alternatively, you can use [Timo's Ubuntu PPA for ROS Packages](https://launchpad.net/~roehling/+archive/ros) on Launchpad,
which will always ship the latest release:
```sh
$ sudo add-apt-repository ppa:roehling/ros
$ sudo apt update
$ sudo apt install catkin-lint
```

## Debian

Prebuilt packages are available from the official Debian archive. Install with
```sh
$ sudo apt install catkin-lint
```

!!! note
    For Debian Buster, the package is named `python-catkin-lint`.

## Download from PyPI

You can download and install **catkin_lint** from the [Python Package Index](https://pypi.python.org/pypi/catkin_lint)
with:
```sh
$ sudo pip install catkin-lint
```

## Install from Source

You can clone **catkin_lint** from [GitHub](https://github.com/fkie/catkin_lint):
```sh
$ git clone https://github.com/fkie/catkin_lint.git
$ cd catkin_lint
$ sudo python setup.py install
```
