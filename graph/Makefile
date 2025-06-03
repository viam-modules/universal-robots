install-uv:
	curl -LsSf https://astral.sh/uv/install.sh | sh

create-venv:
	uv venv -p python3.10
	uv pip install -r requirements.txt

graphs:
	go run graph.go
	.venv/bin/python3 graph.py

remove-graphs:
	rm *.png

create-config:
	echo '{' > config.json && \
	echo '  "trajectory_csv": "~/.viam/module-data/[part-id]/universal-robots-executable/[timestamp]_trajectory.csv",' >> config.json && \
	echo '  "waypoints_csv": "~/.viam/module-data/[part-id]/universal-robots-executable/[timestamp]_waypoints.csv",' >> config.json && \
	echo '  "output_trajectory_poses": "~/trajectory_poses.csv",' >> config.json && \
	echo '  "output_waypoint_poses": "~/waypoint_poses.csv"' >> config.json && \
	echo '}' >> config.json
