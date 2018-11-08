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

cam_config = configparser.ConfigParser();
cam_config.read(path_to_config_file);
print(cam_config.sections())

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

config_options_width = {4416: "2K", 3840: "FHD", 2560: "HD", 1344: "VGA"};
config_options_height = {1242: "2K", 1080: "FHD", 720: "HD", 376: "VGA"};

try:
    camera_config = config_options_width[width];
except KeyError:
    print("Error - selected camera #", args.camera_to_use,
    " : resolution does not match a known ZED configuration profile.");
    print();
    exit(1);

print();
print("ZED left/right resolution: ", int(width/2), " x ",  int(height));
print("ZED mode: ", camera_config);
print("Press <space> to change camera mode);
print();

################################################################################

# process config to get camera calibration from calibration file

# TODO

################################################################################

# define display window names

windowName = "Live Camera Input"; # window name
windowNameD = "Stereo Disparity"; # window name

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

        # perform stereo disparity computation - TODO

        # scale, filter and colour map for display - TODO

        # display image

        cv2.imshow(windowName,frame);

        # stop the timer and convert to ms. (to see how long processing and display takes)

        stop_t = ((cv2.getTickCount() - start_t)/cv2.getTickFrequency()) * 1000;

        # start the event loop - essential
        # wait 40ms or less depending on processing time taken (i.e. 1000ms / 25 fps = 40 ms)

        key = cv2.waitKey(max(2, 40 - int(math.ceil(stop_t)))) & 0xFF;

        # e.g. if user presses "x" then exit  / press "f" for fullscreen display

        if (key == ord('x')):
            keep_processing = False;
        elif (key == ord('f')):
            cv2.setWindowProperty(windowName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN);
        elif (key == ord(' ')):

            # cycle camera resolutions to get the next one on the list

            pos = 0;
            list_widths = list(config_options_width.keys())
            list_heights = list(config_options_height.keys())

            for (width_resolution, config_name) in config_options_width.items():

                    if (list_widths[pos % len(list_widths)] == width):

                        camera_config = config_options_width[list_widths[(pos+1) % len(list_widths)]]

                        # get new camera resolution

                        width = next(key for key, value in config_options_width.items() if value == camera_config)
                        height = next(key for key, value in config_options_height.items() if value == camera_config)

                        print ("Changing camera config to use: ", camera_config, " @ ", width, " x ", height);
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
            print("ZED mode: ", camera_config);
            print();

            cv2.resizeWindow(windowName, width, height);
            cv2.resizeWindow(windowNameD, int(width/2), height);


    # close all windows

    cv2.destroyAllWindows()

    zed_cam.release();

else:
    print("Error - no camera connected.");

################################################################################
