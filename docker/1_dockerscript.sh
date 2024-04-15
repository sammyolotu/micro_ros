xhost +local:root

#docker image build -t origami:1.0 -f docker/Dockerfile docker     
docker pull enunezs/microros:humble 


docker run -it --env="DISPLAY" \
	--device=/dev/video0:/dev/video0 \
	--env DISPLAY=$DISPLAY \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	--net=host \
	--volume /dev/shm:/dev/shm \
	enunezs/microros:humble \
	bash

#	--env "ROS_DOMAIN_ID=7" \
#	--volume $(pwd):/uros2_ws \ # Mount our files


export containerId=$(docker ps -l -q)
