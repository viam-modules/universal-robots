### Instructions

In order to run we first need to create a virtual environment.
We will be using [uv](https://docs.astral.sh/uv/getting-started/installation/) for virtual environment management.
To install please run `make install-uv`
To setup the virtual environment please run `make create-venv`
Next, run `make create-config`. This will create a `config.json` file which the user will need to update with two pieces of information, the part-id of the machine as well as the unix timestamp of the data they wish to graph.

Now we are ready to create graphs! To do so just run `make graphs`