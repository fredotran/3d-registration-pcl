# 3D Registration for large scale point clouds.

## Requirements.

* [Ubuntu (20.04 or higher)](https://ubuntu.com/download/desktop)
* [CMake (3.6 or higher)](https://cmake.org/runningcmake/)
* [Point Cloud Library (1.10.0 or higher)](https://pointclouds.org/downloads/)
* [CUDA (11.0 or higher)](https://developer.nvidia.com/cuda-toolkit-archive) (if you want to use GPU version of PCL)

## Building and compiling the project on Linux (Ubuntu) using CMake

* Clone the repo
* In the `3D-registration` folder, run `cmake -B build` to build and generate the dependencies of the project into the `build` folder.
* Enter build folder and run `make` to compile the project each time you do modifications.

*More infos about how to use PCL in your C++ project here : [tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/using_pcl_pcl_config.html#using-pcl-pcl-config)*

## Data 
The framework cannot read .las or .laz point cloud files directly, you need first to convert the data into **.pcd** files.
You can convert any types of data (.laz, .las, .xyz) using : [CloudCompare](https://www.danielgm.net/cc/).

Steps :
* Load the (.laz, .las, .xyz etc...) file in CloudCompare
* Click on the saving button (to save the cloud)
* Choose "point cloud library format" a.k.a **.pcd**
* Press enter or ok, it's done !

## Launching the pipeline and indicates which input files to use

*What to do ?*
* You will be able to store your data and the parameters' file to use them in the pipelines, in the `data` folder. 
* To use a particular point cloud file, you'll need to specify the location address at : `surface_model_data_file` inside the text file.
* To launch the pipeline (if you are in the `build` folder): `./[PIPELINE_NAME] [number_of_iterations]`  
with PIPELINE_NAME = {experiment1, experiment2, etc...}.

## Tools
You will find the source code of the tools used for this pipeline in the `include` folder.
* Visualization tools : `visualization_tools.hpp`
* Registration tools : `registration.hpp`
* Parameters : `parameters.hpp`
* General tools : `tools.hpp`
