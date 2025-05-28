### Instructions

This directory allows us to graph per joint the inputs passed into `MoveThroughJointPositions` and how they compare to the inputs given through trajectory generation.
In order to run we first need to create a virtual environment.
We recommend using [uv](https://docs.astral.sh/uv/getting-started/installation/)
Once you have  `uv` installed you need to do
```
uv venv -p python3.10 graph
source graph/bin/activate
uv pip install matplotlib numpy pandas
```

here is an example of how to run the file
```
python3 graph.py /home/nick/.viam/module-data/[part-id]/universal-robots-executable/1748360427793_trajectory.csv /home/nick/.viam/module-data/[part-id]/universal-robots-executable/1748360427793_waypoints.csv

```