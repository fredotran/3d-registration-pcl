# 3D Registration for large scale point clouds.

## Requirements.

* [Ubuntu (20.04 or higher)](https://ubuntu.com/download/desktop)
* [CMake (3.6 or higher)](https://cmake.org/runningcmake/)
* [Point Cloud Library (1.10.0 or higher)](https://pointclouds.org/downloads/)
* [CUDA (11.0 or higher)](https://developer.nvidia.com/cuda-toolkit-archive) (if you want to use GPU version of PCL)

## Building and compiling the project on Linux (Ubuntu) using CMake

* Downloading the project files using this in your command console :  
`git clone https://github.com/fredotran/3d-registration-pcl.git`
* Create a `build` folder, go to the folder by using : `cd build`.
* Use : `cmake ..` cmd line to build and generate the dependencies of the project into the `build` folder.
* Use : `make` to compile the project each time you did modifications (be sure to be in the `build` folder).

*More infos about how to use PCL in your C++ project here : [tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/using_pcl_pcl_config.html#using-pcl-pcl-config)*

## Launching the pipeline and indicates which input files to use

*What to do ?*
* You will be able to store your data and the parameters' file to use them in the pipelines, in the `data` folder. 
* To use a particular point cloud file, you'll need to specify the location address at : `surface_model_data_file` inside the text file.
* To launch one of the three pipelines : `./[PIPELINE_NAME]`  
with PIPELINE_NAME = {harris_pipeline, sift_pipeline, main_registration}.

## Tools
You will find the source code of the tools used for this pipeline in the `include` folder.
* Visualization tools : `visualization_tools.hpp`
* Registration tools : `registration.hpp`
* Parameters : `parameters.hpp`
* General tools : `tools.hpp`
