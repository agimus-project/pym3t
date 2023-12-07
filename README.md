A python wrapper around M3T tracker from https://github.com/DLR-RM/3DObjectTracking/tree/master

# Installation

`git clone git@github.com:MedericFourmy/pym3t.git --recursive`

Install dependencies with conda:  
`conda env create --name pym3t --file environment.yaml`  

Then  
`pip install .`

# Example scripts
As example of usage of the library, scripts are provided: 
* `run_image_dir_example.py`: single object tracking using color and depth images from filesystem
* `run_webcam_example.py`: single object tracking with first camera device detected by the system (webcam or other usb camera usually)
* `run_realsense_example.py`: single object tracking with realsense camera

:question:: check available options with `python <script>.py -h`

For all examples, you need a object mesh in the wavefront .obj format with name <object_id>.obj. Upon first execution, a set of sparse template views are generated which can take some time.

## Running image per image  
----
For this example you need a set of of recorded sequential color (and potentially depth) images stored in a directory.
The color images `color*.png` and `depth*.png` need have names with lexicographic order (e.g. color_000000.png, color_000001.png, color_000002.png...)
Calibrated camera intrinsics in the formate described in config/cam_d435_640.yaml also need to be provided.

Color only:   
```
python run_image_dir_example.py --use_region -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml --stop
```

Color + depth:   
```
python run_image_dir_example.py --use_region --use_depth -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml --stop
```

Keyboard commands:
- `q`: exit
- `any other key`: When running with --stop/-s argument, continue to next image

## Running with webcam
To bypass camera calibration, a reasonable horizontal fov (~50-70 degrees) can be assumed to get camera intrinsics
```
python run_webcam_example.py --use_region -b obj_000014 -m <path/to/obj/dir>
```

Keyboard commands:
- `q`: exit
- `d`: reset object pose to initial guess
- `x`: start/restart tracking

## Running with realsense camera
----
Color only:   
```
python run_realsense_example.py --use_region -b obj_000014 -m <path/to/obj/dir>
```

----
Color + depth:   
```
python run_realsense_example.py --use_region --use_depth -b obj_000014 -m <path/to/obj/dir>
```

Keyboard commands:
- `q`: exit
- `d`: initialize object pose
- `x`: start/restart tracking