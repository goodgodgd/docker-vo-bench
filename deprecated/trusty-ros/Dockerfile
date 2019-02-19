FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive
COPY scripts/ros_trusty_setup.sh /root
RUN chmod a+x /root/ros_trusty_setup.sh \
	&& /root/ros_trusty_setup.sh

# docker run --name orbslam -it -v "/home/ian/workplace/docker-vo/trusty-ros:/work" trusty-ros:dev bash

