# Stereolabs ZED -  OpenCV Native Capture in Python

![alt text](https://raw.githubusercontent.com/apennisi/ZedCameraGrabber/master/images/zed.jpg)

This sample shows how to capture rectified images with the StereoLabs ZED (or ZED-M) Stereo Camera and OpenCV, **without the ZED SDK**, using only Python. It is inspired by the C++version of the same available from [stereolabs](https://github.com/stereolabs/zed-opencv-native). As the images supplied from the ZED stereo camera are already rectified, we do not need to use the calibration to rectify the images further before performing the disparity calculation but the intrinsics and extrinsics (camera matrix _K_, focal length _f_, and baseline, _B_) are required for subsequent depth (distance) recovery.

Alternatively, if you want to use OpenCV with the ZED SDK features, check our sample [here](https://github.com/stereolabs/zed-opencv).

Developed to support teaching within the undergraduate Computer Science programme at [Durham University](http://www.durham.ac.uk) (UK) by [Prof. Toby Breckon](http://community.dur.ac.uk/toby.breckon/). All tested with [OpenCV](http://www.opencv.org) 3.x and Python 3.x.

---

### How to download and run:

_Firstly_, you need to know the serial number of the ZED stereo camera you are trying to use. To do so, you can use ZED Explorer tools (ZED SDK tools) and check the serial number on the top right of ZED Explorer window or alternatively each camera has a small label on the end of the USB lead with the serial number on it.


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

Native live stereo from a StereoLabs ZED camera using factory calibration.

optional arguments:
  -h, --help            show this help message and exit
  -c CAMERA_TO_USE, --camera_to_use CAMERA_TO_USE
                        specify camera to use
  -s SERIAL, --serial SERIAL
                        camera serial number
  -cf CONFIG_FILE, --config_file CONFIG_FILE
                        camera calibration configuration file
  -fs, --fullscreen     run disparity full screen mode
  -cm, --colourmap      apply disparity false colour display
```

Press the _"f"_ key to run disparity fullscreen, press  _"c"_ key to add colour map and _space_ to change camera resolution mode (press _"x"_ to exit). Changing between the resolutions does work when tested although sometimes does cause the camera image to freeze.

---

### Re-usable Exemplar Components:

From teaching and learning, this codebase contains several re-usable exemplar elements that offer more general insight:

- ```zed_calibration.py``` - an example of how to setup and perform camera calibration in OpenCV using pre-existing manufacturer supplied calibration data (as opposed to performing manual calibration with a calibration target object such as a chessboard as available in this example - [stereo_sgbm.py](https://github.com/tobybreckon/python-examples-cv/blob/master/stereo_sgbm.py))

- ```camera_stream.py``` - a re-usable threaded camera class, that is call compatible with the existing OpenCV VideoCapture class, designed to always deliver the latest frame from a single camera without buffering delays. _This code is not specific to stereo cameras or the ZED stereo camera_.

- ```zed-cam-sn-1010.conf``` - an example of how parameters and settings can be stored in this simple INI file format (originating from MS Windows INI files) and read/parsed using functionality built into the Python standard library (see ```zed-stereo.py``` / ```zed_calibration.py```).

---

### References:

If using this example in your own work (e.g _"... based on the implementation of REF..."_), please reference our related research work:

- [Generalized Dynamic Object Removal for Dense Stereo Vision Based Scene Mapping using Synthesised Optical Flow](http://community.dur.ac.uk/toby.breckon/publications/papers/hamilton16removal.pdf) (O.K. Hamilton, T.P. Breckon), In Proc. International Conference on Image Processing, IEEE, pp. 3439-3443, 2016. [[pdf]](http://community.dur.ac.uk/toby.breckon/publications/papers/hamilton16removal.pdf)

- [A Foreground Object based Quantitative Assessment of Dense Stereo Approaches for use in Automotive Environments](http://community.dur.ac.uk/toby.breckon/publications/papers/hamilton13stereo.pdf) (O.K. Hamilton, T.P. Breckon, X. Bai, S. Kamata), In Proc. International Conference on Image Processing, IEEE, pp. 418-422, 2013. [[pdf]](http://community.dur.ac.uk/toby.breckon/publications/papers/hamilton13stereo.pdf)

---

If you find any bugs raise an issue (or much better still submit a git pull request with a fix) - toby.breckon@durham.ac.uk

_"may the source be with you"_ - anon.
