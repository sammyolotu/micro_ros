xhost +local:root

docker image build -t microros:humble -f docker/Dockerfile docker     

docker run -it --env="DISPLAY" \
	--device=/dev/video0:/dev/video0 \
	--env DISPLAY=$DISPLAY \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--privileged \
	--net=host \
	--volume /dev/shm:/dev/shm \
	microros:humble
#	--env "ROS_DOMAIN_ID=7" \
#	--volume $(pwd):/root/ws/origami \

export containerId=$(docker ps -l -q)

