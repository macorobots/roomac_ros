Here you can find example devcontainer setup that I use for development. To apply it, you have to remove `_template` postfix from both files. In this configuration I have whole local `catkin_ws` mounted:
```
	"workspaceMount": "src=${localWorkspaceFolder}/../..,dst=/home/roomac/catkin_ws/,type=bind",
```
So you should have following setup `{some_local_directory}/catkin_ws/src/roomac_ros` for devcontainer to work properly. Mounting whole `catkin_ws` allows to preserve builds, so it doesn't have to be redone once container is removed. Of course you can feel free to modify it to your own preference.