# ArmStone

## Docker
There is a docker file that contains the packages and all the dependencies. 
Where the general workflow to build the image:  

- Clone the repo
- Build the docker image

```bash
cd ArmStone
docker build --pull --rm -f ./.docker/Dockerfile  -t arm_stone:latest .
```

If you are changing the Dockerfile remove the `--rm` tag to keep your intermediate builds. 

### Running

My approach (2.3 from the [ROS guide](http://wiki.ros.org/docker/Tutorials/GUI))

```bash
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --workdir="/catkin_ws/src" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --gpus 'all,"capabilities=graphics,utility"'\
    arm_stone:latest
```

or without GPUs

```bash
docker run -it \
    --user=$(id -u $USER):$(id -g $USER) \
    --group-add sudo \
    --env="DISPLAY" \
    --workdir="/catkin_ws/src" \
    --volume="/home/$USER:/home/$USER" \
    --volume="/etc/group:/etc/group:ro" \
    --volume="/etc/passwd:/etc/passwd:ro" \
    --volume="/etc/shadow:/etc/shadow:ro" \
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    arm_stone:latest
```

This should be logging your host user in the container, mounting your home directory within the image and other things like x server info and sudo access.  
The other perk of this is your ssh keys are hopefully in `~/.ssh` so you can then push your changes. 

if you are going to use this container for a while then give it name with: `--name arm_stone_dev`

Lastly the repo has been added in the docker process and is owned by root so the user id you've added won't be able to use it.
Change ownership to the user with `sudo chown -R $UID /catkin_ws/`
