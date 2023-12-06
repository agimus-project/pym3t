A python wrapper around M3T tracker from https://github.com/DLR-RM/3DObjectTracking/tree/master

# Installation

`git clone git@github.com:MedericFourmy/pym3t.git --recursive`

Install dependencies with conda:  
`conda create -n pym3t`  
`conda activate pym3t`  
`mamba env update --file environment.yaml`

Then  
`pip install .`

# Example scripts
As example of usage of the library, scripts are provided: 
* `run_image_per_image.py`: reads color and depth imges from filesystem
* `run_on_camera_sequence_realsense.py`: run single object tracking with realsense camera


## Running image per image example script
----
Color only:   
```
python run_image_per_image.py --use_region --use_depth --use_texture -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml -n 10 -s
```

Color + depth:   
```
python run_image_per_image.py --use_region --use_depth -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml -n 10 -s
```

## Running with webcam
To bypass camera calibration, a reasonable horizontal fov (~50-70 degrees) can be assumed to get camera intrinsics
```
python run_webcam.py --use_region -b obj_000014 -m <path/to/obj/dir>
```

## Running realsense camera example script
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