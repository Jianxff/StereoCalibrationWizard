# Stereo Calibration Wizard #

![avatar](https://badgen.net/badge/Language/C++17/green)
![stars](https://badgen.net/badge/Env./windows/blue)
![stars](https://badgen.net/badge/Lab./opencv4/orange)

## INTRO ##
  **DUAL-cameras** clibration wizard by estimating the **best next position** of the chessboard, which can decrease the reprojection error on calibtion and gain higher rate of convergence for camera parameters.

## FEATURE ##
- **number of pictures**
  The calibration result by taking 3 free-capture images and several guiance images (4 or more) is as good as (even better) that by taking 30 or more free-capture images.

- **calibration time**
  Calibration with guidance will cost fewer time than take losts of free-capture images. 

- **fault-tolerance**
  Calibration with guidance has higher rate of fault-tolerace. You may take bad images without consious when free capturing, but it won't happen when using guidance, which support better next position and is also easy to roll-back. 

- **evaluation**
  Calibration result will be evaluated by reprojection errors and measurement result (measuring items in real world). 

## UPDATE ##
**ON UPDATING ..**

**2023/2/11**
  - **C++**
    - add *`magnify`* while selecting points in function `measurement`

**2023/2/10**
  - **C++**
    - remove *`AD-Census`* stereo-match algorithm, which got bad match result and had low speed
    - add function *`inputCalibrate()`* for calibration input images from `data/images`, which is controlled by `data/images/count.txt`
    - add compress for video capture with choices of *`MJPG`* and *`YUY2`* (warning: *`MJPG`* will cause stuck while exiting the program on *`opencv 4`*)
  - **Matlab**
    - add `fast-mode` for get guidance of next position with fewer time (effective on few input images)
  - **Python**
    - remove *`AD-Census`* radio button
  

**2023/2/7**
  - **C++**
    - sync-frame camera supported
    - add argument parser
  - **Matlab** 
    - `cost_function` modify with same-position weight on output
    - pso `lower_bound` and `upper_bound` adjust to improve calc speed

**2023/2/4**
  - **C++**
    - calibrate roll-back supported by putting number of free-capture images at the second row on `data/images/count.txt`

**OLDER**
  - **C++**
    - coding modularization with *`config`*,*`capture`*,*`calibrate`*,*`measurement`*,*`utils`*
    - single/double camera(s) supported
    - read config infomation from *`config.xml`*
    - recorded history calibrate data by *`data/calib_data.xml`*
    - measurement function with stereo-match algorithm *`ELAS`*,*`SGBM`*,*`AD-Census`*
    - measurement function with dynamic toolbar
  - **Matlab**
    - single/double camera(s) supported
    - pso/sa optimization algorithm
    - using jacobian and matrix operation to estimate and evalute the position 
    - autocorrelation matrix to estimate chessboard corners
  - **Python**
    - plot calibration data results/records by `matplotlib` 
    - uniformly call `C++ program` and `matlab engine`
    - simple gui based on `PyQt5`
    - log info with `logging`

<!-- # Calibration Wizard
[**Paper**](http://openaccess.thecvf.com/content_ICCV_2019/papers/Peng_Calibration_Wizard_A_Guidance_System_for_Camera_Calibration_Based_on_ICCV_2019_paper.pdf) | [**Video**](https://www.youtube.com/watch?v=my3jocjpD0U&feature=youtu.be&t=398) <br>
<img src="https://pengsongyou.github.io/media/wizard0.jpg" width="270"/> <img src="https://pengsongyou.github.io/media/wizard1.jpg" width="270"/> <img src="https://pengsongyou.github.io/media/wizard2.jpg" width="270"/>

This repository contains the implementation of the paper:

Calibration Wizard: A Guidance System for Camera Calibration Based on Modelling Geometric and Corner Uncertainty  
[Songyou Peng](http://pengsongyou.github.io/) and [Peter Sturm](https://team.inria.fr/steep/people/peter-sturm/)  
**ICCV 2019** (Oral)  

If you find our code or paper useful, please consider citing
```
@inproceedings{peng2019iccv,
 author =  {Songyou Peng and Peter Sturm},
 title = {Calibration Wizard: A Guidance System for Camera Calibration Based on Modelling Geometric and Corner Uncertainty},
 booktitle = {IEEE International Conference on Computer Vision (ICCV)},
 year = {2019},
}
```
## Installation
### Requirements
* CMake
* OpenCV
* MATLAB

### Build
```sh
git clone https://github.com/pengsongyou/CalibrationWizard
cd CalibrationWizard
mkdir build
cd build
cmake ..
make
```
Tested working on Mac OSX, OpenCV 2.4.11 and MATLAB R2015b.

## Usage

* **First step**: run binary `./bin/CalibrationWizard` and choose `mode=0` to capture images freely for initial calibration.
Press `space` to capture one image. After capturing, press `ESC` and the calibration is automatically done.  
* **Second step**: run `src_matlab/main_estimate.m` to estimate the next best pose.  
* **Third step**: run binary `./bin/CalibrationWizard` again and choose `mode=1`. The estimated next pose should be displayed. You can try to overlay the checkerboard with the new pose, and press `space` to capture the image.

Loop over the second and third step until you are satisfied, or tired :)

#### Command Line Options
* `mode=0` captures images for initial camera calibration
* `mode=1` shows the next best pose for capturing

In addition, we provide `mode=2` if you only want to perform calibration on all the captured images listed in `out/images/image_list.xml`.

#### TODO
* [ ] Release codes of the unified system in C++ which integrates the next best pose estimation.

#### Extreme Poses
Sometimes the next pose is extreme so the checkerboard cannot be detected. Please consider: 
* Check if the values in the uncertainty map are low. If yes, then the calibration has more or less converged.
* Consider autocorrelation matrix, simply set `autoCorr_flag=1` in `src_matlab/main_estimate.m`.
* Re-run `src_matlab/main_estimate.m`. -->
