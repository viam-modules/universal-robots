### Instructions

Run `make create-config`. This will create a `config.json` file which the user will need to update with two pieces of information, the part-id of the machine as well as the unix timestamp of the data they wish to graph.
We also note that part of the config is the relative path to the arm's kinemtatics, which defaults to the ur5e.
To plot against a different model that will also need to be updated.
Please note, if you are using an executable you will need to change `viam_universal-robots` to `universal-robots-executable`

Now we are ready to create graphs! To do so just run `make graphs` or `go run graph.go`
We note that a user does not need to remove `.png` files if running visualization multiple times, the `.png` files will be overwritten.