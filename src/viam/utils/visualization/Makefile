graphs:
	go run graph.go

remove-graphs:
	rm *.png

create-config:
	echo '{' > config.json && \
	echo '  "trajectory_csv": "~/.viam/module-data/[part-id]/universal-robots-executable/[timestamp]_trajectory.csv",' >> config.json && \
	echo '  "waypoints_csv": "~/.viam/module-data/[part-id]/universal-robots-executable/[timestamp]_waypoints.csv",' >> config.json && \
	echo '}' >> config.json
