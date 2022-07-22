# base container with all necessary dependencies
ARG IMAGEBASE
ARG IMAGEBASETAG
FROM $IMAGEBASE:$IMAGEBASETAG

USER root
WORKDIR /seerep

COPY certs src/certs/
COPY seerep-com src/seerep-com/
COPY seerep-hdf5 src/seerep-hdf5/
COPY seerep-msgs src/seerep-msgs/
COPY seerep-ros src/seerep-ros/
COPY seerep-srv src/seerep-srv/
COPY examples src/examples/
COPY docker/start_server.sh src/start_server.sh

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release; catkin build"

EXPOSE 9090

ENTRYPOINT ["/bin/bash", "src/start_server.sh"]
