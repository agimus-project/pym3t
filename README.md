A python wrapper around ICG tracker from https://github.com/DLR-RM/3DObjectTracking/tree/master, __WIP__

# Installation

`git clone https://github.com/MedericFourmy/pyicg.git --recursive`

Install dependencies:  
`conda create -n pyicg`  
`conda activate pyicg`  
`mamba update --file environment.yaml`

Then  
`pip install .`

# Example scripts
As example of usage of the library, scripts are provided: `run_image_per_image_color.py` and `run_image_per_image_color_depth.py` run single object tracking.

## Setup a tracking sequence 

To run the examples as is, you should have:
* A directory `<obj_dir>` containing the `<object_name>.obj` object file model
* In `<obj_dir>`, a `<object_name>.yaml` file similar to the `banana.yaml' example in config directory. The "object_name" need to match with .obj file.
* A sequence of color (and optionally depth) images stored in the `<images_dir>` directory with following conventions (using default opencv formats):
  * Color images should be 8-bit BGR, file name starting with `bgr`
  * Depth images should be 16-bit grayscale, file name starting with `depth`
* A `static_detector.yaml` in the config directory (default is `./config/`) following. This sets the initial pose from camera to object. 

## Running the examples
----

Color only:   
```
python3 run_image_per_image_color.py -b <object_name> -m <obj_dir> -i <images_dir>
```

Color + depth:   
```
python3 run_image_per_image_color_depth.py -b <object_name> -m <obj_dir> -i <images_dir>
```

With a realsense camera plugged:
```
python3 run_on_camera_sequence_realsense.py -b <object_name> -m <obj_dir> 
```

Check the options with `-h` argument.

TODO
----
* Make sure compilation is done with RELEASE flag
* `pip install -e .` not working
* Check https://github.com/scikit-build/scikit-build/issues/479#issuecomment-1502585979
