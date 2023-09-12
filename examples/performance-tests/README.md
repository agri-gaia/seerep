# MCAP vs HDF5 comparison for saving ROS messages

This package contains the benchmark for the comparison between [HDF5](https://www.hdfgroup.org/solutions/hdf5/) and
[MCAP](https://mcap.dev/) for storing ROS messages.

## Usage

Run the following commands for the setup:

```shell
pip install conan==1.60.2 pandas
PATH=$PATH:/home/docker/.local/bin
sudo apt update
sudo apt install texlive-latex-extra texlive-fonts-recommended cm-super dvipng  -y
```

Build the code with optimization (-O3 -DNDEBUG):

```shell
catkin build seerep_performance_tests --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Source the catkin workspace (assuming it's located under `~/catkin_ws/`):

```shell
source ~/catkin_ws/devel/setup.bash
```

If nedded, change the I/O paths at the top of `run.py`:

```python
...
OUTPUT_DIR = Path("/seerep/seerep-data/benchmarks")
MESSAGE_PAYLOAD = Path("/seerep/seerep-data/lero/...")
...
```

Run the benchmark with:

```shell
rosrun seerep_performance_tests run.py
```

If you only want to plot existing `.csv` data use:

```shell
rosrun seerep_performance_tests run.py --only-plot
```

## Settings

The benchmark currently support the following parameters:

| Name | Description  |
|---|---|
| Output Dir | Dir to write the `.mcap` and `.hdf5` files to |
| Message Payload | File to use as bytes for the messages data field |
| Message Sizes | Array of messages sizes (in bytes) to be tested |
| Total Sizes | Array of total data to write in bytes|
| Num Runs | Number of times each config should be run |

Each of these paramters are located at the top of the [run.py](./run.py) file in the `config` dictonary
