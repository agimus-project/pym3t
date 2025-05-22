# PYM3T

A python wrapper around M3T tracker from [DLR-RM/3DObjectTracking](https://github.com/DLR-RM/3DObjectTracking/tree/master).

## Installation

To install pym3t, you can use pip or poetry.

We strongly suggest to install it in either a
[venv](https://docs.python.org/fr/3/library/venv.html) or a
[conda environment](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html).

### Example with conda

```
git clone https://github.com/agimus-project/pym3t
cd pym3t
conda env create -f environment.yml
conda activate pym3t
pip install .
```

### Example with venv

> [!NOTE]
> M3T relies on [GLFW](https://www.glfw.org/). Before building ensure it is installed.
> For Ubuntu run `apt-get install libglfw3 libglfw3-dev`

```
git clone https://github.com/agimus-project/pym3t
cd pym3t
python -m venv .venv
source .venv/bin/activate
pip install .
# or if you want to run examples
pip install .[examples]
```

### Building with realsense support
First ensure that [librealsense](https://github.com/IntelRealSense/librealsense) is installed. It should be the case if you followed the conda-based installation. Then build with:

`pip install . --config-settings="cmake.define.USE_REALSENSE=ON"`

# Example scripts
As example usage of the library, we provide several scripts: 
* `run_image_dir_example.py`: single object tracking using color and depth images from filesystem;
* `run_webcam_example.py`: single object tracking with first camera device detected by the system (webcam or other usb camera usually);
* `run_realsense_example.py`: single object tracking with realsense camera.

> [!IMPORTANT]
> For all examples, you need a object mesh in the Wavefront **.obj** format with name **<object_id>.obj**. Upon first execution, a set of sparse template views are generated which can take some time.

> [!TIP]
> Check available options with `python <script name>.py -h`

## Running image per image  
----
To run this example you need a set of of recorded sequential color (and potentially depth) images stored in a directory.
The color images **color\*.png** and **depth\*.png** need to have names with lexicographic order (e.g. *color_000000.png*, *color_000001.png*, *color_000002.png*, ...)
Calibrated camera intrinsics in the formate described in config/cam_d435_640.yaml also need to be provided.

Color only:   
``` bash
python examples/run_image_dir_example.py --use_region -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml --stop
```

Color + depth:   
``` bash
python examples/run_image_dir_example.py --use_region --use_depth -b obj_000014 -m <path/to/obj/dir> -i <path/to/image/dir> -c config/cam_d435_640.yaml --stop
```

Keyboard commands:
- `q`: exit;
- `any other key`: When running with **--stop** or **-s** argument, continue to next image.

## Running with webcam
To bypass camera calibration, a reasonable horizontal fov (50 - 70 degrees) can be assumed to get camera intrinsics
``` bash
python examples/run_webcam_example.py --use_region -b obj_000014 -m <path/to/obj/dir>
```

Keyboard commands:
- `q`: exit;
- `d`: reset object pose to initial guess;
- `x`: start/restart tracking.

## Running with realsense camera
----
Color only:   
```bash
python examples/run_realsense_example.py --use_region -b obj_000014 -m <path/to/obj/dir>
```

----

Color + depth:   
```bash
python examples/run_realsense_example.py --use_region --use_depth -b obj_000014 -m <path/to/obj/dir>
```

Keyboard commands:
- `q`: exit;
- `d`: initialize object pose;
- `x`: start/restart tracking.
