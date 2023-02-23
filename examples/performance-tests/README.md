# MCAP vs HDF5 comparison for saving ROS messages

This package contains the benchmark for the comparison between [HDF5](https://www.hdfgroup.org/solutions/hdf5/) and
[MCAP](https://mcap.dev/) for storing ROS messages.

## Usage

Build the code with optimization (-O3 -DNDEBUG):

```shell
catkin build seerep_performance_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Just to be sure, source the catkin workspace (assuming it's located under `~/catkin_ws/`):

```shell
source ~/catkin_ws/devel/setup.bash
```

### Run the benchmark with

```shell
rosrun seerep_performance_tests run.py
```

## Settings

The benchmark currently support the following parameters:

| Name | Description  |
|---|---|
| Output Dir | Dir to write the `.mcap` and `.hdf5` files to |
| Message Payload | File to use as bytes for the messages data field |
| Message Sizes | Array of messages sizes (in bytes) to be tested |
| Total Sizes | Array of which total bytes should be tested |
| Num Runs | Number of times each config should be run |

Each of these paramters are located at the top of the [run.py](./run.py) file.

## Results

The resulting executation times are stored in `.csv` files with a label describing the config. Further a plot will be
generated to visualize the results. Both are saved in the `Output Dir`.
