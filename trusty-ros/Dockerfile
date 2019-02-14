FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive
RUN mkdir -p /work
WORKDIR /work
COPY ros_trusty_setup.sh /work
RUN chmod a+x ros_trusty_setup.sh \
	&& /.ros_trusty_setup.sh

