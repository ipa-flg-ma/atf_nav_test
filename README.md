# Packages for ATF Navigation Tests
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
Use `catkin_make $PKG_NAME$` to start one of the following packages, where `$PKG_NAME$` is one from the list:

- line_passage
- line_passage_obstacle
- line_passage_person_moving
- line_passage_spawn_obstacle
- narrow_passage_2_cone
- t_passage
- t_passage_obstacle

