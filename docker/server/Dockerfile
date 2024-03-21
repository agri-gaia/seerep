ARG IMAGEBASE
ARG IMAGEBASETAG
FROM $IMAGEBASE:$IMAGEBASETAG

USER root

WORKDIR /seerep

COPY certs src/certs/
COPY seerep_com src/seerep_com/
COPY seerep_hdf5 src/seerep_hdf5/
COPY seerep_msgs src/seerep_msgs/
COPY seerep_ros src/seerep_ros/
COPY seerep_srv src/seerep_srv/
COPY examples src/examples/
COPY docker/server/start_server.sh src/start_server.sh

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release; catkin build"

EXPOSE 9090

ENTRYPOINT ["/bin/bash", "src/start_server.sh"]
