<!---
Author: Jan Brejcha <janbrejcha>
Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
Project: ImmersiveTripReports 2017-2018, LandscapeAR 2018 - 2020
AdobePatentID="P7840-US"
-->

ITR - Immersive Trip Reports
=============================
This repository contains the official implementation of the following paper:
```
BREJCHA Jan, LUKÁČ Michal, CHEN Zhili, DIVERDI Stephen a ČADÍK Martin. Immersive Trip Reports. In: Proceedings of the 31st ACM User Interface Software and Technology Symposium. Berlín: Association for Computing Machinery, 2018, s. 389-401. ISBN 978-1-4503-5948-1.
```
[Project webpage](http://cphoto.fit.vutbr.cz/immersive-trip-reports/), [Paper (pdf)](http://cphoto.fit.vutbr.cz/immersive-trip-reports/data/immersive_trip_reports.pdf), [Power Point presentation](http://cphoto.fit.vutbr.cz/immersive-trip-reports/data/itr_uist_2018.ppsx).

Building
--------
To build `itr`, you will need to install following dependencies. Some dependencies need to be patched (respective files should be overwritten) by the files at the `patches` subdirectory. The latest version has been tested on OSX and Fedora GNU/Linux, but should probably work on other Linux systems as well. If something goes wrong, feel free to submit an issue. Please, follow the versions of dependencies specified in the following guideline, as we cannot guarantee that the system works with different versions. For easy installation, we also prepared an install script, which will download, compile and install all needed dependencies (see below).

-   ###### OpenSceneGraph 3.7.0 (<https://github.com/openscenegraph/OpenSceneGraph.git>)
    -   Needs to be compiled on OS X
    -   Latest version (master) in GIT is probably OK, but tested is version 3.7.0.
    -   Dependencies: Jasper
    -   Configuration:
		-   Overwrite OpenSceneGraph/src/osgViewer/GraphicsWindowCocoa.mm with the patch
			from patches/OpenSceneGraph/osgViewer/GraphicsWindowCocoa.mm
        -   Set ```OPENGL_PROFILE GL3```
        -   ``OSG_GL3_AVAILABLE TRUE``
        -   Make sure to DISABLE ALL other old OpenGL features, like display lists, fixed function, etc.

-   ###### GDAL
    -   OSX: ```brew install gdal```

-   ###### OSGEARTH 2.9 (<https://github.com/gwaldron/osgearth.git>)
    -   Needs to be compiled on OS X
    -   Latest version (master) in GIT is probably OK, but tested version is 2.9.
    -   Configuration:
		-   Overwrite osgearth/src/osgEarth/Capabilities by patches/osgearth/Capabilities
        -   Add -std=c++11 to CMAKE_CXX_FLAGS

-   ###### Ceres
    -   This project needs Ceres as well as OpenMVG needs it. Install Ceres prior to OpenMVG.
    -   OS X: ```brew install ceres-solver```

-   ###### Eigen (http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2)
    -   Make sure to use Eigen (3.3.7) with OpenMVG, Libnabo and Libpointmatcher as they don’t work properly with newer versions - memory alignment error in Eigen’s malloc.

-   ###### OpenMVG (<https://github.com/openMVG/openMVG.git>)
	-   use commit ```35da8f72c68``` - so that it is compatible with `itr` code AND with **Eigen 3.3.7**.
    -   Configuration:
		- 	overwrite openMVG/src/software/SfM/export/CMakeLists.txt by patches/openMVG/CMakeLists.txt
		- 	if you are on MAC OS, overwrite openMVG/src/software/SfM/export/InterfaceMVS.h by patches/openMVG/InterfaceMVS.h

        -   Set ```EIGEN_INCLUDE_DIR_HINTS``` to directory where the Eigen 3.3.7 is installed (usually ```/usr/local/include/eigen3```)
        -   Set ```EIGEN_DIR``` to the same directory as ```EIGEN_INCLUDE_DIR_HINTS```

-   ###### Libpointmatcher 1.3.1 (<https://github.com/ethz-asl/libpointmatcher.git>)
    -   Latest version (master) in GIT is probably OK.
    -   Needs to be compiled for OS X
    -   Configuration:
        -   Provide Eigen 3.3.7 in CMake configuration
    -   Dependencies:
        -   **Boost 1.67.0** (<https://dl.bintray.com/boostorg/release/1.67.0/source/boost_1_67_0.tar.bz2>)
        -   **Libnabo 1.0.6** (<https://github.com/ethz-asl/libnabo>)
            -   Latest version (master) in GIT is probably OK.
            -   needs to be compiled for OS X
            -   Configuration:
            -   Provide Eigen 3.3.7 in CMake configuration


-   ###### Glog
    -   OSX: ```brew install glog```
-   ###### LibXML2

    ###### nvm_to_openmv
        - git clone https://github.com/brejchajan/nvm_to_openmvg


Build script
------------
To make the downloading, compilation, and installation of third party dependencies easier, we prepared a build script for GNU/Linux - Fedora, and MacOSX 10.15. You can run it as follows:
```./scripts/build-<system>.sh <build dir> <install dir> <build type (Release|Debug)> [num processes]```
The script will download third party dependencies, apply all needed patches, compile and install them into the <install dir> together with itr. We recommend to use a local install dir which can be added to the PATH manually.

Usage
-----
```itr --directory <components> <earthFile.earth>```

###### mandatory parameters:
```--directory <components>```: The directory containing SfM
reconstructed components. The components directory is supposed to contain numbered subdirectories with individual SfM components. To reconstruct the scene, you may use OpenMVG, or COLMAP. Each subdirectory is supposed to contain all component images, as well as the file containing scene data. In case of OpenMVG reconstruction, this file will be sfm_data.bin, in case of COLMAP use sfm_data.nvm.

```<earthFile.earth>``` The definition file for OSGEarth to define data layers to be loaded (terrain, textures, etc.). To use default earth file, use `example.earth` located in the root of this repository. This argument is supposed to be *ALWAYS* the last argument.

###### optional parameters:
```--all-photoviews <txt_file>```: A file containing list of photographs which
will be visible in the viewer. From this set of photographs the user can pick the photographs for the presentation. Typically the list
will contain all user-taken photographs. The list must contain the path
relative to the image, e.g., if the program is run with ```--directory mydir```,
and mydir contains subdirectories ```0/``` and ```1```, then the file list
would contain:
```0/img1.jpg```
```0/img2.jpg```
...
```1/imgN.jpg```

```--picked-photoviews <txt_file>```: A file containing list of photoviews at which the fly-through will stop. This is especially usable when you create a version of a fly-through which you want to resume later.

```--sort-time```: If this flag is set in combination with `--all-photoviews`,
the photoviews will be sorted according to the time of creation found in exif.
The sort WILL NOT occur, if the date of creation cannot be find for all photographs specified by the `--all-photoviews` flag.

```--render-photoviews <image_width>```: Set this flag to render the dataset
of synthetic and corresponding real images in batch mode.

```--egl```: When combined with `--render-photoviews` and when compiled on Linux machine with support of EGL (NVidia only), this will enable offscreen
rendering (mainly for usage on cluster). The support for EGL is detected
automatically by CMake - if this flag is not available, it means that CMake
did not detect the libraries needed for EGL.

Step-by-step trip creation
--------------------------
1. Create a list file with all user-taken photographs, see ```--all-photoviews``` option above. Run
```itr --directory <components> --all-photoviews <photoviews_file.txt> <earthFile.earth>```.
Please note, that all photographs specified in this file will be used
as control points for the animation spline. The animation will then stop on
photographs, which the user picked, but the shape on the spline does not
depend on the selected photographs (it depends on control points specified
in this list). So if the generated spline seems to be overly complicated,
please, remove the unnecessary photographs from this list to achieve better
result. Also please note, that the fly-through will be generated with the
same order of photographs as defined in this list.

2. Make sure the photographs are above the ground with correct orientation.
First press ```o```, the photos will be moved up to the ground if they are too low, and then ```i```, which will correct their orientation in case they were moved. It might take
some time to process all the images, especially when the terrain is not
cached yet.

3. To be able to locate the photographs on the earth, press ```t```, which shows the red markers of photograph positions. By pressing the same button you can switch between red markers (current photo positions), green markers (EXIF GPS photo positions), and blue markers (pointing in eye direction from the center of the image plane). Locate the photographs on the earth by double clicking near the position of photographs several times, or by
holding right mouse button while pulling the mouse downwards. If you click directly on some photograph, you will be taken to visit mode, and you will see a photograph over the entire screen. To leave it, press ```c```.

4. Show only user-taken photographs specified with ```--all-photoviews```,
press ```h```. By pressing the same key you may switch between viewing all
photographs in the reconstruction, all user-taken photographs, and all user-picked photographs to see only the photographs, which will be presented
(at which the camera will stop during the flythrough).

5. With user-taken photographs selected, you may now pick the photographs
to stop at. Navigate in the 3D world by dragging the mouse. To pick the
photograph, press `ctrl` and click at the photograph. The picked photographs
will be fully opaque in contrast to the not selected ones which will be halfway
transparent. If you want to deselect the photograph, click on it again with
`ctrl` key pressed.

6. When you are satisfied with your selection, press `f`. This will generate
the fly-through based on all user photographs specified by `--all-photoviews`
option. It might take a while to complete, especially if the terrain is not
fully cached yet. After it finishes, the fly-through spline will be visualized
on screen. By pressing `h` (maybe several times) show only the picked
images from previous step. Verify, that the spline goes through the picked images. Make sure that the images at the beginning and at the end are not ommitted. If they are ommitted, please, deselect them - they can't be added into the final fly-through because of the spline restrictions.

7. (optional) To export the scene, press `j`. This will export:
    - selected photographs to the `picked_photoviews.txt` file,
    - KML tour for Google Earth to the `scene.kml` file,
    - Scene for VR viewer in Unity to the `scene.json` file.
All exported files are located in the `<components>` directory.

8. If you want to view the generated fly-through directly in the viewer, press `p`. The fly-through will start and it will be paused at each picked image (from the step 5). When the
fly-through is paused, you may look around by dragging the mouse. To continue,
press `p` again. Make sure not to press `p` before the animation stops
(this will break the animation, it is a known, not yet solved bug).

9. After the fly-through ends, you are again free to move around in the 3D world.
To view the fly-through again, go to step 6. You may also consider adjusting the picked photoviews to get better results before regenerating the fly-through.

Creating the list of all photoviews for the `--all-photoviews` option
-----------------------------------------------------------------
Creating a good list of control points to generate the fly-through may be
tricky. The suggested scenario when the number of photographs is big is following:

A. Create inital list by searching for names of images coming from your camera
(filter out Flickr images.) For example, with Nepal dataset it may be done like this: `cd <components>; for i in $(ls | grep "^[0-9]$"); do ls $i | grep _MG; done > all_photoviews.txt`.

B. Open the `itr` and use this initial list. Complete steps 1. - 7. and pick the photographs you want to use as control points.

C. Close the application and rename the generated list of picked photographs (from step 7.): `mv picked_photoviews.txt all_photoviews_refined.txt`.

D. Go to step 1., but use the refined file `all_photoviews_refined.txt` from step C, instead of the original file `all_photoviews.txt`. Now you can
pick the photoviews you want the fly-through to stop at, and generate the
final fly-through.

To make experimenting with different selections easier, you may also use the
optional flag `--picked-photoviews <picked_photoviews.txt>`, and provide a
list of picked photoviews directly without the need to pick them in the viewer.
