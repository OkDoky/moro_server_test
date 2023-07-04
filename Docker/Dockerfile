FROM ros:melodic-ros-base

RUN apt-get update && apt-get dist-upgrade -y

COPY ./install.sh /
RUN /install.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
    CMD [ "bash" ]