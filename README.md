A python wrapper around ICG tracker, __WIP__

Building
----

`git clone git@github.com:MedericFourmy/pyicg.git --recursive`

Install dependencies:  
`conda create -n pyicg`  
`conda activate pyicg`  
`mamba update --file environment.yaml`

Then  
`pip install .`

Running
----
```python3 run_image_per_image_color_depth.py -b banana -m /home/mfourmy/sandbox/3DObjectTracking/ICG/examples/generator_example -i /home/mfourmy/Documents/ciirc_research/data/banana_video/bananas -s```
```python3 run_image_per_image_color.py -b banana -m /home/mfourmy/sandbox/3DObjectTracking/ICG/examples/generator_example -i /home/mfourmy/Documents/ciirc_research/data/banana_video/bananas -s```



TODO
----
* Make sure compilation is done with RELEASE flag
* `pip install -e .` not working
* Check https://github.com/scikit-build/scikit-build/issues/479#issuecomment-1502585979
