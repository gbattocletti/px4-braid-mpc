# Repo setup

To create the ros package skeleton:
```sh
ros2 pkg create px4_braid_mpc --build-type ament_python --license GPL-3.0-or-later --maintainer-name "G. Battocletti" --maintainer-email "g.battocletti@protonmail.com" --dependencies rclpy px4_msgs
```
Add submodules with:
```sh
cd ~/px4_ws/src/px4_braid_mpc
git submodule add https://github.com/gbattocletti/px4-launch.git external/px4-launch
git submodule add https://github.com/gbattocletti/braid-controller.git external/braid-controller
git config submodule.recurse true   # to automatically pull changes in submodules
```
NOTE: To remove a submodule:
```sh
git submodule deinit -f external/<name>
git rm -f external/<name>
rm -rf .git/modules/external/<name>
```	