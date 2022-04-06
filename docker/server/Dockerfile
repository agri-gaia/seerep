# base container with all necessary dependencies
FROM git.ni.dfki.de:5050/agrigaia/seerep/base

WORKDIR /home/docker/seerep_ws
RUN mkdir -p src/seerep

COPY certs certs
COPY seerep-com src/seerep/seerep-com
COPY seerep-hdf5 src/seerep/seerep-hdf5
COPY seerep-msgs src/seerep/seerep-msgs
COPY seerep-ros src/seerep/seerep-ros
COPY seerep-srv src/seerep/seerep-srv
COPY util src/seerep/util
COPY docker/start_server.sh start_server.sh

RUN chown -R docker:docker /home/docker/seerep_ws

USER docker

RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin build"
RUN /bin/bash -c ". /home/docker/seerep_ws/devel/setup.bash"

EXPOSE 9090

ENTRYPOINT ["/bin/bash", "start_server.sh"]
