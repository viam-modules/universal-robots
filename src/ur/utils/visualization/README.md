### Instructions

On the machine where the module has stored its data - run `make create-example-config`.
This will create a `config.json` file which the user will need to update.
If you ran the universal-robots module locally you only need to update the part-id of the machine as well as the unix timestamp of the data one wishes to graph.
This path being editable so, if data lives elsewhere and this intial population is just for convinience and can be changed entirely.
We also note that part of the config is the relative path to the arm's kinemtatics, which defaults to the ur5e.
To plot against a different model that will also need to be updated.
Please note, if you are using an executable you will need to change `viam_universal-robots` to `universal-robots-executable`.
This can be determined by entering the `.viam` directory.

Now we are ready to create graphs! To do so just run `make graphs` or `go run graph.go`
We note that a user does not need to remove `.png` files if running visualization multiple times, the `.png` files will be overwritten.
If you prefer to see them appear each time a user may run `make remove-graphs` or `rm *.png`