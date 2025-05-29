### Instructions

In order to run we first need to create a virtual environment.
We recommend using [uv](https://docs.astral.sh/uv/getting-started/installation/)
Once you have  `uv` installed you need to do
```
uv venv -p python3.10 graph
source graph/bin/activate
uv pip install matplotlib numpy pandas
```

Next create a json file named `config.json` within this directory
```json
{
  "viam": {
    "host": "",
    "entity_id": "",
    "api_key": ""
  },
  "arm_name": "ur5e-modular",
  "trajectory_csv": "~/.viam/module-data/[part-id]/universal-robots-executable/1748360427793_trajectory.csv",
  "waypoints_csv": "~/.viam/module-data/[part-id]/universal-robots-executable/1748360427793_waypoints.csv",
  "output_trajectory_poses": "~/trajectory_poses.csv",
  "output_waypoint_poses": "~/waypoint_poses.csv"
}
```

Plotting Joint Positions
`python3 graph_joint_positions.py`

Plotting XYZ Positions
`go run graph.go`
`python3 graph_poses.py`