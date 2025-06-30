### Instructions

On the machine where the module has stored its data - run `make create-example-config`.
This will create a `config.json` file which a user will need to update.
The `trajectory_csv` field should be the absolute path to a given trajectory csv.
Similarly, `waypoints_csv` should be the absolute path to a given waypoints csv.
The `arm_kinematics_path` field will also need to be updated with the relative path from this location to either `ur3e.json` or `ur5e.json` or `ur20.json`.

Now we are ready to create graphs!
To do so just run `make graphs` or `go run graph.go`.
We note that a user does not need to remove `.png` files if running visualization multiple times, the `.png` files will be overwritten.
If you prefer to see them appear each time a user may run `make remove-graphs` or `rm *.png`.