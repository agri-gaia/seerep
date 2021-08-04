FROM ros:noetic

#add non root user
RUN useradd -u 1001 --create-home --shell /bin/bash docker && echo "docker:docker" | chpasswd && adduser docker sudo
USER docker
WORKDIR /home/docker
#folder for caching of VS Code extensions
RUN mkdir -p /home/docker/.vscode-server
#workspace to be used for code
RUN mkdir -p /home/docker/workspace/src

#install stuff
USER root
COPY installDependencies.sh .
RUN ./installDependencies.sh

USER docker
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.zshrc" >> ~/.zshrc

RUN echo "[[ -e /home/docker/workspace/devel/setup.bash ]] && source /home/docker/workspace/devel/setup.bash" >> ~/.bashrc
RUN echo "[[ -e /home/docker/workspace/devel/setup.zshrc ]] && source /home/docker/workspace/devel/setup.zshrc" >> ~/.zshrc

ENTRYPOINT [ "/bin/sh", "-c", "while sleep 1000; do :; done" ]
