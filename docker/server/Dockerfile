# base container with all necessary dependencies
FROM git.ni.dfki.de:5050/agrigaia/seerep/base

USER root
WORKDIR /seerep

COPY certs src/certs/
COPY seerep-com src/seerep-com/
COPY seerep-hdf5 src/seerep-hdf5/
COPY seerep-msgs src/seerep-msgs/
COPY seerep-ros src/seerep-ros/
COPY seerep-srv src/seerep-srv/
COPY util src/util/
COPY docker/start_server.sh src/start_server.sh

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build"

EXPOSE 9090

ENTRYPOINT ["/bin/bash", "src/start_server.sh"]
