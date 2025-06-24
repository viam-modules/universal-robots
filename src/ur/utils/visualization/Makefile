graphs:
	go run graph.go

remove-graphs:
	rm *.png

create-config:
	echo '{' > config.json && \
	echo '  "trajectory_csv": "~/.viam/module-data/[part-id]/viam_universal-robots/[timestamp]_trajectory.csv",' >> config.json && \
	echo '  "waypoints_csv": "~/.viam/module-data/[part-id]/viam_universal-robots/[timestamp]_waypoints.csv",' >> config.json && \
	echo '  "arm_kinematics_path": "../../../kinematics/ur5e.json"' >> config.json && \
	echo '}' >> config.json
