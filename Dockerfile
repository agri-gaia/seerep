FROM ros:noetic

#add non root user
RUN useradd -u 1001 --create-home --shell /bin/bash docker && echo "docker:docker" | chpasswd
USER docker
WORKDIR /home/docker
#workspace to be used for code
RUN mkdir -p /home/docker/workspace/src

#install stuff
USER root
RUN mkdir -p /tmp/pre-commit
COPY .pre-commit-config.yaml /tmp/pre-commit/
COPY installDependencies.sh .
RUN ./installDependencies.sh

USER docker
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/bin/sh", "-c", "while sleep 1000; do :; done" ]
