A python wrapper around ICG tracker, __WIP__

Install dependencies:  
`conda create -n pyicg`  
`mamba update --file environment.yaml`

Just `pip install .`

TODO
----
* Make sure compilation is done with RELEASE flag
* `pip install -e .` not working
* Check https://github.com/scikit-build/scikit-build/issues/479#issuecomment-1502585979

TROUBLESHOOTING
---------------
* linker issues in condaenv: update your env variable  
`export LD_LIBRARY_PATH=/home/mfourmy/miniconda3/envs/pyicg/lib/`