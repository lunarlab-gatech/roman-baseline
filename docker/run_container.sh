DATA_DIR='<FILL_IN>'
REPO_DIR='<FILL_IN>'

docker run -it \
    --name="roman_baseline" \
    --net="host" \
    --privileged \
    --gpus 'all,"capabilities=compute,utility,graphics,display"' \
    --device /dev/dri:/dev/dri:rw \
    --device /dev/nvidia0:/dev/nvidia0:rw \
    --device /dev/nvidiactl:/dev/nvidiactl:rw \
    --device /dev/nvidia-uvm:/dev/nvidia-uvm:rw \
    --workdir="/home/$USER/roman" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --env="XDG_RUNTIME_DIR=/tmp/runtime-$USER" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="$REPO_DIR:/home/$USER/roman" \
    --volume="$DATA_DIR:/home/$USER/data" \
    --volume="/home/$USER/.bash_aliases:/home/$USER/.bash_aliases" \
    --volume="/home/$USER/.ssh:/home/$USER/.ssh:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume /tmp/runtime-$USER:/tmp/runtime-$USER \
    --volume="$XAUTHORITY:/tmp/.Xauthority:ro" \
    --volume="/usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro" \
    --volume="/etc/vulkan/icd.d:/etc/vulkan/icd.d:ro" \
    --volume="/usr/share/nvidia:/usr/share/nvidia:ro" \
    roman_baseline \
    /bin/bash
