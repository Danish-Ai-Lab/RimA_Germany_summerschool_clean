# Riss 2025 Devcontainer

This is an example workspace with a devcontainer file for the 2025 RISS. The devcontainer
can be setup in VsCode or over cli using the [devcontainer/cli]{https://github.com/devcontainers/cli}
## Setup
### VSCode
To use this devcontainer, install docker and VsCode. There are a bunch of ways to install docker
incorrectly. Install it using the [official apt repos](https://docs.docker.com/engine/install/ubuntu/).

**Don't install docker using**
- snap
- original ubuntu repos (might be too old)
- DONT INSTALL DOCKER DESKTOP (except on windows/MAC)

After all this setup, make sure your user is in the docker group (see [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)).
Open the workspace in VsCode, summon the command prompt using `Ctrl+Shift+P`. Search for *Reopen in
Container* and press enter. Building the container can take some time, just check the logs. Once
that is done you should have a working environment.

### cli
Install the devcontainer/cli on https://github.com/devcontainers/cli
To ease the use of the cli you can use the following alias and copy them in your preferred shellrc

Create and/or start devcontainer
```
alias devup='devcontainer up --workspace-folder . --dotfiles-repository=https://github.com/fmauch/dotfiles.git --dotfiles-install-command=install.sh'
```
Create devcontainer and override previous instance
```
alias devnew='devcontainer up --workspace-folder . --dotfiles-repository=https://github.com/fmauch/dotfiles.git --dotfiles-install-command=install.sh --remove-existing-container'
```
Execute command in devcontainer
```
alias devrun='devcontainer exec --workspace-folder .'
```
Open shell in devcontainer
```
alias devsh='devcontainer exec --workspace-folder . zsh'
```

### Build workspace

After the devcontainer is setup and running you can build the included ros workspace.
For this first you have to install all necessary dependencies using:
```
cd ~/ws/
rosdep install --from-paths src --ignore-src -r
```
Now click `y` when it asks you to install specific packages. However make sure to press `n` for
`libopenvdb-dev` and `ros-humble-ompl`.
If you installed it by accident don't worry just remove them by:
```
sudo apt remove libopenvdb-dev
sudo apt remove ros-humble-ompl
```

When all dependencies are correctly installed use
```
colcon build --symlink-install
```
in your ws folder to build the whole workspace.


If anything doesn't work, feel free to ask us.

## Quick Fixes

### Navpi Not Building because of OMPL

ROS version of ompl is currently broken, install the one from the apt repos. Might break when install debs via
rosdep.

```
sudo apt remove ros-humble-ompl
sudo apt install libompl-dev
```
# RimA_Germany_summerschool
# RimA_Germany_summerschool
