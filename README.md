A python wrapper around M3T tracker from https://github.com/DLR-RM/3DObjectTracking/tree/master

# Installation

`git clone git@github.com:MedericFourmy/pym3t.git --recursive`

Install dependencies with conda:  
`conda env create --name pym3t --file environment.yaml`  

Then  
`pip install .`

# Example scripts
As example of usage of the library, scripts are provided: 
* `run_image_per_image.py`: single object tracking using color and depth images from filesystem
* `run_on_camera_sequence_realsense.py`: single object tracking with realsense camera
* `run_webcam.py`: single object tracking with first camera device detected by the system (webcam or other usb camera usually)

## Running image per image
----
Color only:   
```
python run_image_per_image.py --use_region --use_depth --use_texture -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml -n 10 -s
```

Color + depth:   
```
python run_image_per_image.py --use_region --use_depth -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml -n 10 -s
```

Keyboard commands:
- `q`: exit
- `any other key`: When running with --stop=-s argument, continue to next image

## Running with webcam
To bypass camera calibration, a reasonable horizontal fov (~50-70 degrees) can be assumed to get camera intrinsics
```
python run_webcam.py --use_region -b obj_000014 -m <path/to/obj/dir>
```

Keyboard commands:
- `q`: exit
- `d`: initialize object pose
- `x`: start tracking

## Running with realsense camera
----
Color only:   
```
python run_on_camera_sequence_realsense.py --use_region -b obj_000014 -m <path/to/obj/dir>
```

----
Color + depth:   
```
python run_on_camera_sequence_realsense.py --use_region --use_depth -b obj_000014 -m <path/to/obj/dir>
```

Keyboard commands:
- `q`: exit
- `d`: initialize object pose
- `x`: start tracking