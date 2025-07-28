# thesis-monorepo
This repo stores all of the code written and used for my undergraduate thesis. That is using an event camera to predict
camera frame velocities in 6-DOF.

**OceanSim Development Environment Setup**
This repo can be directly stored in your isaacsim/extsUser directory after installing isaacsim. The following instructions allow you to separate this git repo from isaacsim making it easier to maintain.

1. Follow installation instructions as shown in https://github.com/umfieldrobotics/OceanSim/blob/main/docs/subsections/installation.md
2. To setup development environment correctly create a symbolic link in isaacsim directory: <pre>cd isaacsim/extsUser
ln -s $THIS_REPO/OceanSim OceanSim</pre>
3. We will use the isaacsim environment: <pre> cd isaacsim
rm requirements.txt
ln -s $THIS_REPO/requirements.txt requirements.txt
./python.sh -m pip install -r requirements.txt
4. In VScode install the Isaac Sim VS Code Edition extension. This will load python configs and the isaacsim development environment
5. To launch the development environment open vscode from the isaacsim folder <pre> code $PATH_TO_ISAACSIM </pre>


