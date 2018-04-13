# Packages for ATF Navigation Tests

[![GitHub commit activity the past week, 4 weeks, yea](https://img.shields.io/github/commit-activity/4w/ipa-flg-ma/atf_nav_pkgs.svg)](https://github.com/ipa-flg-ma/atf_nav_pkgs)

[![GitHub repo size in bytes](https://img.shields.io/github/repo-size/ipa-flg-ma/atf_nav_pkgs.svg)](https://github.com/ipa-flg-ma/atf_nav_pkgs)

[![Build Status](https://travis-ci.org/ipa-flg-ma/atf_nav_pkgs.svg?branch=master)](https://travis-ci.org/ipa-flg-ma/atf_nav_pkgs)

This repository contains the packages to perform the automated navigation tests using [ATF](https://github.com/ipa-fmw/atf), including the `launch` files, the `application.py` which sets the navigation goal and the min/max allowed values for the test metrics in the `test1.yaml`.

## Installation
Dependencies:

- catkin

**How To Install**:
1. Create a "Catkin Workspace"
2. Clone repository
3. Use `catkin_make` to build the package

See code below:

```
$ mkdir -p ~/git/nav_tests/src
$ cd ~/git/nav_tests/
$ catkin_make
$ cd ~/git/nav_tests/src
$ git clone https://github.com/ipa-flg-ma/atf_nav_pkgs.git
$ cd ~/git/nav_tests/
$ catkin_make
```



## Usage
Use `catkin_make atf_$PKG_NAME$` to start one of the following packages, where `$PKG_NAME$` is one from the list:

- line_passage
- line_passage_obstacle
- line_passage_person_moving
- line_passage_spawn_obstacle
- narrow_passage_2_cone
- t_passage
- t_passage_obstacle

