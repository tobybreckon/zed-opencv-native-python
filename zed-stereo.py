################################################################################

# native stereo capture using the StereoLabs ZED camera in Python

# Copyright (c) 2018 Toby Breckon, Durham University, UK

# License: MIT License (MIT)

################################################################################

import cv2
import argparse
import sys
import math

import requests
import configparser

from camera_stream import *
from zed_calibration import *

################################################################################

# parse command line arguments for camera ID and config

parser = argparse.ArgumentParser(description='Native live stereo from a StereoLabs ZED camera using factory calibration.');
parser.add_argument("-c", "--camera_to_use", type=int, help="specify camera to use", default=0);
parser.add_argument("-s", "--serial", type=int, help="camera serial number", default=0);
parser.add_argument("-cf", "--config_file", type=str, help="camera calibration configuration file", default='');

args = parser.parse_args()

################################################################################

# process agruments to get camera config

if (args.serial > 0):

    url = 'http://calib.stereolabs.com/?SN=';

    # we have a serial number - go get the config file from the config url

    r = requests.get(url+str(args.serial));

    if (r.status_code == requests.codes.ok):

        with open("zed-cam-sn-"+str(args.serial)+".conf", "w") as config_file:
            config_file.write(r.text[1:]); # write to file skipping first blank character

        path_to_config_file = "zed-cam-sn-"+str(args.serial)+".conf";

    else:
        print("Error - failed to retrieve camera config from: " + url);
        print();
        parser.print_help();
        exit(1);

elif (len(args.config_file) > 0):

    path_to_config_file = args.config_file;

else:
    print("Error - no serial number or config file specified.");
    print();
    exit(1);
################################################################################

# parse camera configuration as an INI format file

cam_calibration = configparser.ConfigParser();
cam_calibration.read(path_to_config_file);

################################################################################

# define video capture object as a threaded video stream

zed_cam = CameraVideoStream(src=args.camera_to_use).open()

if (zed_cam.isOpened()):
    ret, frame = zed_cam.read();

height,width, channels = frame.shape;

################################################################################

# select config profiles based on image dimensions

# MODE  FPS     Width x Height  Config File Option
# 2.2K 	15 	    4416 x 1242     2K
# 1080p 30 	    3840 x 1080     FHD
# 720p 	60 	    2560 x 720      HD
# WVGA 	100 	1344 x 376      VGA

config_options_width = {4416: "2K", 3840: "FHD", 2560: "HD",calibration.py 1344: "VGA"};
config_options_height = {1242: "2K", 1080: "FHD", 720: "HD", 376: "VGA"};

try:
    camera_mode = config_options_width[width];
except KeyError:
    print("Error - selected camera #", args.camera_to_use,
    " : resolution does not match a known ZED configuration profile.");
    print();
    exit(1);

print();
print("ZED left/right resolution: ", int(width/2), " x ",  int(height));
print("ZED mode: ", camera_mode);
print("Press <space> to change camera mode");
print();

################################################################################

# process config to get camera calibration from calibration file

mapL1, mapL2, mapR1, mapR2 = zed_camera_calibration(cam_calibration, camera_mode, width, height);

################################################################################

# define display window names

windowName = "Live Camera Input"; # window name
windowNameD = "Stereo Disparity"; # window name

################################################################################

# set up defaults for stereo disparity calculation

max_disparity = 128;
window_size = 21;

stereoProcessor = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities = max_disparity, # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=window_size,
        #P1=8 * window_size ** 2,       # 8*number_of_image_channels*SADWindowSize*SADWindowSize
        #P2=32 * window_size ** 2,      # 32*number_of_image_channels*SADWindowSize*SADWindowSize
        #disp12MaxDiff=1,
        #uniquenessRatio=15,
        #speckleWindowSize=0,
        #speckleRange=2,
        #preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_HH
)

################################################################################

# if camera is successfully connected

if (zed_cam.isOpened()) :

    # create window by name (as resizable)

    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL);
    cv2.resizeWindow(windowName, width, height);

    cv2.namedWindow(windowNameD, cv2.WINDOW_NORMAL);
    cv2.resizeWindow(windowNameD, int(width/2), height);

    # loop control flag

    keep_processing = True;
    apply_colourmap = False;

    while (keep_processing):

        # start a timer (to see how long processing and display takes)

        start_t = cv2.getTickCount();

        # if video file successfully open then read frame

        if (zed_cam.isOpened()):
            ret, frame = zed_cam.read();

            # when we reach the end of the video (file) exit cleanly

            if (ret == 0):
                keep_processing = False;
                continue;

        # split single ZED frame into left an right

        frameL= frame[:,0:int(width/2),:]
        frameR = frame[:,int(width/2):width,:]

        # remember to convert to grayscale (as the disparity matching works on grayscale)

        grayL = cv2.cvtColor(frameL,cv2.COLOR_BGR2GRAY);
        grayR = cv2.cvtColor(frameR,cv2.COLOR_BGR2GRAY);

        # undistort and rectify based on the mappings (could improve interpolation and image border settings here)
        # N.B. mapping works independant of number of image channels

        undistorted_rectifiedL = cv2.remap(grayL, mapL1, mapL2, cv2.INTER_LINEAR);
        undistorted_rectifiedR = cv2.remap(grayR, mapR1, mapR2, cv2.INTER_LINEAR);

        # compute disparity image from undistorted and rectified versions
        # (which for reasons best known to the OpenCV developers is returned scaled by 16)

        disparity = stereoProcessor.compute(undistorted_rectifiedL,undistorted_rectifiedR);
        cv2.filterSpeckles(disparity, 0, 4000, max_disparity);

        # scale the disparity to 8-bit for viewing
        # divide by 16 and convert to 8-bit image (then range of values should
        # be 0 -> max_disparity) but in fact is (-1 -> max_disparity - 1)
        # so we fix this also using a initial threshold between 0 and max_disparity
        # as disparity=-1 means no disparity available

        _, disparity = cv2.threshold(disparity,0, max_disparity * 16, cv2.THRESH_TOZERO);
        disparity_scaled = (disparity / 16.).astype(np.uint8);

        # display disparity - which ** for display purposes only ** we re-scale to 0 ->255

        if (apply_colourmap):

            disparity_colour_mapped = cv2.applyColorMap((disparity_scaled * (256. / max_disparity)).astype(np.uint8), cv2.COLORMAP_HOT);
            cv2.imshow(windowNameD, disparity_colour_mapped);
        else:
            cv2.imshow(windowNameD, (disparity_scaled * (256. / max_disparity)).astype(np.uint8));

        # display input image

        cv2.imshow(windowName,frame);

        # stop the timer and convert to ms. (to see how long processing and display takes)

        stop_t = ((cv2.getTickCount() - start_t)/cv2.getTickFrequency()) * 1000;

        # start the event loop - essential
        # wait 40ms or less depending on processing time taken (i.e. 1000ms / 25 fps = 40 ms)

        key = cv2.waitKey(max(2, 40 - int(math.ceil(stop_t)))) & 0xFF;

        # e.g. if user presses "x" then exit  / press "f" for fullscreen display

        if (key == ord('x')):
            keep_processing = False;
        elif (key == ord('c')):
            apply_colourmap = not(apply_colourmap);
        elif (key == ord('f')):
            cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN);
        elif (key == ord(' ')):

            # cycle camera resolutions to get the next one on the list

            pos = 0;
            list_widths = list(config_options_width.keys())
            list_heights = list(config_options_height.keys())

            for (width_resolution, config_name) in config_options_width.items():

                    if (list_widths[pos % len(list_widths)] == width):

                        camera_mode = config_options_width[list_widths[(pos+1) % len(list_widths)]]

                        # get new camera resolution

                        width = next(key for key, value in config_options_width.items() if value == camera_mode)
                        height = next(key for key, value in config_options_height.items() if value == camera_mode)

                        print ("Changing camera config to use: ", camera_mode, " @ ", width, " x ", height);
                        break;

                    pos+=1

            # reset to new camera resolution

            zed_cam.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
            zed_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))

            width = int(zed_cam.get(cv2.CAP_PROP_FRAME_WIDTH))
            height =  int(zed_cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

            print ("Camera config confirmed back from camera as: ", width , " x ", height);
            print();
            print("ZED left/right resolution: ", int(width/2), " x ",  int(height));
            print("ZED mode: ", camera_mode);
            print();

            # reset window sizes

            cv2.resizeWindow(windowName, width, height);
            cv2.resizeWindow(windowNameD, int(width/2), height);

            # get calibration for new camera resolution

            mapL1, mapL2, mapR1, mapR2 = zed_camera_calibration(cam_calibration, camera_mode, width, height);

    # close all windows and release camera

    cv2.destroyAllWindows()

    zed_cam.release();

else:
    print("Error - no camera connected.");

################################################################################
