# Stereolabs ZED -  OpenCV Native Capture in Python

![alt text](https://raw.githubusercontent.com/apennisi/ZedCameraGrabber/master/images/zed.jpg)

This sample shows how to capture rectified images and compute the scene disparity/depth with the StereoLabs ZED (or ZED-M) Stereo Camera and OpenCV, **without the ZED SDK**, using only Python. It is inspired by the C++version of the same available from [stereolabs](https://github.com/stereolabs/zed-opencv-native). As the images supplied from the ZED stereo camera are already rectified (from factory calibration), we do not always need to use the calibration to rectify the images further* before performing the disparity calculation but the intrinsics and extrinsics (camera matrix _K_, focal length _f_, and baseline, _B_) are required for subsequent depth (distance) recovery.

[*] _but we have noticed the calibration to change over time, requring recalibration using this [separate python script tool](https://github.com/tobybreckon/python-examples-cv/blob/master/stereo_sgbm.py)._

Alternatively, if you want to use OpenCV with the ZED SDK features, check out the StereoLabs sample [here](https://github.com/stereolabs/zed-opencv).

In addition to manufacturer supplied calibration files, the ```-xml``` option alternatively facilitates the use of ```calibration.xml``` files generated via a 5-stage chessboard target based, manual calibration process performed using the [stereo_sgbm.py](https://github.com/tobybreckon/python-examples-cv/blob/master/stereo_sgbm.py) example tool from our [python-examples-cv](https://github.com/tobybreckon/python-examples-cv/blob/master/stereo_sgbm.py) teaching repository.

Developed to support teaching within the undergraduate Computer Science programme at [Durham University](http://www.durham.ac.uk) (UK) by [Prof. Toby Breckon](https://breckon.org/toby/). All tested with [OpenCV](http://www.opencv.org) 4.x and Python 3.x.

---

### How to download and run:

_Firstly_, you ideally need to know the serial number of the ZED stereo camera you are trying to use in order to access the manufacturer's factory calibration. To do so, you can use ZED Explorer tools (ZED SDK tools) and check the serial number on the top right of ZED Explorer window or alternatively each camera has a small label on the end of the USB lead with the serial number on it.


Clone the repository and run as follows, with your camera serial number as ```SERIAL```:

```
git clone https://github.com/tobybreckon/zed-opencv-native-python.git
cd zed-opencv-native-python
python3 ./zed-stereo.py --serial SERIAL --camera_to_use 1
```

Example will retrieve camera calibration from manufacturer's on-line calibration site and write to file as ``` zed-cam-sn-SERIAL.conf```

In general, this example can be used as follows:

```
usage: zed-stereo.py [-h] [-c CAMERA_TO_USE] [-s SERIAL] [-cf CONFIG_FILE]
                     [-cm] [-fix] [-fill] [-fs] [-t] [-hs] [-vs] [-no]
                     [-xml CONFIG_FILE_XML] [-lrl] [--showcontrols]

Native live stereo from a StereoLabs ZED camera using factory calibration.

optional arguments:
  -h, --help            show this help message and exit
  -c CAMERA_TO_USE, --camera_to_use CAMERA_TO_USE
                        specify camera to use
  -s SERIAL, --serial SERIAL
                        camera serial number
  -cf CONFIG_FILE, --config_file CONFIG_FILE
                        ZED camera calibration configuration file
  -cm, --colourmap      apply disparity false colour display
  -fix, --correct_focal_length
                        correct for error in VGA factory supplied focal
                        lengths for earlier production ZED cameras
  -fill, --fill_missing_disparity
                        in-fill missing disparity values via basic
                        interpolation
  -fs, --fullscreen     run disparity full screen mode
  -t, --showcentredepth
                        display cross-hairs target and depth from centre of
                        image
  -hs, --sidebysideh    display left image and disparity side by side
                        horizontally (stacked)
  -vs, --sidebysidev    display left image and disparity top to bottom
                        vertically (stacked)
  -no, --nooriginal     do not display original live image from camera
  -xml CONFIG_FILE_XML, --config_file_xml CONFIG_FILE_XML
                        manual camera calibration XML configuration file
  -lrl, --leftrightleft
                        perform left to right + right to left matching and
                        weighted least square filtering
  --showcontrols        display track bar disparity tuning controls
```

Key commands can be used as follows:
```
Keyboard Controls:
space    - change camera mode
f        - toggle disparity full-screen mode
c        - toggle disparity false colour mapping
t        - toggle display centre target cross-hairs and depth
h        - toggle horizontal side by side [left image | disparity]
v        - toggle vertical side by side [left image | disparity]
o        - toggle original live image display
w        - toggle left->right + right->left weighted least squares filtering
i        - toggle disparity in-filling via interpolation
x        - exit
```

Left click mouse in disparity image to print depth value at that point (beware that depth values in VGA mode maybe wrong unless ```-fix``` option is used). Changing between the resolutions worked when tested although it appears it does cause the camera image to freeze sometimes.

---

### Re-usable Exemplar Components:

For teaching and learning, this codebase contains several re-usable exemplar elements that offer more general insight:

- ```zed_calibration.py``` - an example of how to setup and perform camera calibration in OpenCV using pre-existing manufacturer supplied calibration data (as opposed to performing manual calibration with a calibration target object such as a chessboard as available in this example - [stereo_sgbm.py](https://github.com/tobybreckon/python-examples-cv/blob/master/stereo_sgbm.py))

- ```camera_stream.py``` - a re-usable threaded camera class, that is call compatible with the existing OpenCV VideoCapture class, designed to always deliver the latest frame from a single camera without buffering delays. _This code is not specific to stereo cameras or the ZED stereo camera_.

- ```utils.py``` - functions ```h_concatenate()``` and ```v_concatenate()``` perform horizontal/vertical OpenCV image stacking for then displaying several images in a single common window.

- ```zed-cam-sn-1010.conf``` - an example of how parameters and settings can be stored in this simple INI file format (originating from MS Windows INI files) and read/parsed using functionality built into the Python standard library (see ```zed-stereo.py``` / ```zed_calibration.py```).

---

### References:

If using this example in your own work (e.g _"... based on the implementation of REF..."_), please reference our related research work from which the default parameters for the Semi-Global Block Matching [Hirschmuller, 2007] approach were derived:

- [Generalized Dynamic Object Removal for Dense Stereo Vision Based Scene Mapping using Synthesised Optical Flow](https://breckon.org/toby/publications/papers/hamilton16removal.pdf) (O.K. Hamilton, T.P. Breckon), In Proc. International Conference on Image Processing, IEEE, pp. 3439-3443, 2016. [[pdf]](https://breckon.org/toby/publications/papers/hamilton16removal.pdf)

- [A Foreground Object based Quantitative Assessment of Dense Stereo Approaches for use in Automotive Environments](https://breckon.org/toby/publications/papers/hamilton13stereo.pdf) (O.K. Hamilton, T.P. Breckon, X. Bai, S. Kamata), In Proc. International Conference on Image Processing, IEEE, pp. 418-422, 2013. [[pdf]](https://breckon.org/toby/publications/papers/hamilton13stereo.pdf)

---

If you find any bugs raise an issue (or much better still submit a git pull request with a fix) - toby.breckon@durham.ac.uk

_"may the source be with you"_ - anon.
