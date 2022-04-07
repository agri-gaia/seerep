# base container with all necessary dependencies
FROM git.ni.dfki.de:5050/agrigaia/seerep/base

USER docker
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc

RUN echo "[[ -e /home/docker/workspace/devel/setup.bash ]] && source /home/docker/workspace/devel/setup.bash" >> ~/.bashrc
RUN echo "[[ -e /home/docker/workspace/devel/setup.zsh ]] && source /home/docker/workspace/devel/setup.zsh" >> ~/.zshrc

RUN echo 'export PYTHONPATH="${PYTHONPATH}:/home/docker/workspace/build/seerep-msgs:/home/docker/workspace/build/seerep-com"' >> ~/.bashrc

ENTRYPOINT [ "/bin/sh", "-c", "while sleep 1000; do :; done" ]
