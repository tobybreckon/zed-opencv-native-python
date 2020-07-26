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

from zed_calibration import *
from utils import *

################################################################################

# mouse call back routine to display depth value at selected point

def on_mouse_display_depth_value(event, x, y, flags, params):

    # when the left mouse button has been clicked

    if ((event == cv2.EVENT_LBUTTONDOWN) and not((args.sidebysidev) or (args.sidebysideh))):

        # unpack the set of parameters

        f, B = params

        # safely calculate the depth and display it in the terminal

        if (disparity_scaled[y,x] > 0):
            depth = f * (B / disparity_scaled[y,x])
        else:
            depth = 0

        # as the calibration for the ZED camera is in millimetres, divide
        # by 1000 to get it in metres

        print("depth @ (" + str(x) + "," + str(y) + "): " + '{0:.3f}'.format(depth / 1000) + "m")

################################################################################

# track bar call back routine to set stereo parameters if controls are enabled

def on_trackbar_set_disparities(value):
    stereoProcessor.setNumDisparities(max(16, value * 16))

def on_trackbar_set_blocksize(value):
    if not(value % 2):
        value = value + 1
    stereoProcessor.setBlockSize(max(3, value))

def on_trackbar_set_speckle_range(value):
    stereoProcessor.setSpeckleRange(value)

def on_trackbar_set_speckle_window(value):
    stereoProcessor.setSpeckleWindowSize(value)

def on_trackbar_set_setDisp12MaxDiff(value):
    stereoProcessor.setDisp12MaxDiff(value)

def on_trackbar_set_setP1(value):
    stereoProcessor.setP1(value)

def on_trackbar_set_setP2(value):
    stereoProcessor.setP2(value)

def on_trackbar_set_setPreFilterCap(value):
    stereoProcessor.setPreFilterCap(value)

def on_trackbar_set_setUniquenessRatio(value):
    stereoProcessor.setUniquenessRatio(value)

def on_trackbar_set_wlsLmbda(value):
    wls_filter.setLambda(value)

def on_trackbar_set_wlsSigmaColor(value):
    wls_filter.setSigmaColor(value * 0.1)

def on_trackbar_null(value):
    return

################################################################################

# parse command line arguments for camera ID and config

parser = argparse.ArgumentParser(description='Native live stereo from a StereoLabs ZED camera using factory calibration.')
parser.add_argument("-c", "--camera_to_use", type=int, help="specify camera to use", default=0)
parser.add_argument("-s", "--serial", type=int, help="camera serial number", default=0)
parser.add_argument("-cf", "--config_file", type=str, help="ZED camera calibration configuration file", default='')
parser.add_argument("-cm", "--colourmap", action='store_true', help="apply disparity false colour display")
parser.add_argument("-fix", "--correct_focal_length", action='store_true', help="correct for error in VGA factory supplied focal lengths for earlier production ZED cameras")
parser.add_argument("-fill", "--fill_missing_disparity", action='store_true', help="in-fill missing disparity values via basic interpolation")
parser.add_argument("-fs", "--fullscreen", action='store_true', help="run disparity full screen mode")
parser.add_argument("-t",  "--showcentredepth", action='store_true', help="display cross-hairs target and depth from centre of image")
parser.add_argument("-hs", "--sidebysideh", action='store_true', help="display left image and disparity side by side horizontally (stacked)")
parser.add_argument("-vs", "--sidebysidev", action='store_true', help="display left image and disparity top to bottom vertically (stacked)")
parser.add_argument("-no", "--nooriginal", action='store_true', help="do not display original live image from camera")
parser.add_argument("-xml", "--config_file_xml", type=str, help="manual camera calibration XML configuration file", default='')
parser.add_argument("-lrl", "--leftrightleft", action='store_true', help="perform left to right + right to left matching and least square filtering")
parser.add_argument("--showcontrols", action='store_true', help="display track bar disparity tuning controls")

args = parser.parse_args()

################################################################################

# process agruments to get camera calibration

camera_calibration_available = False
manual_camera_calibration_available = False

if (args.serial > 0):

    url = 'http://calib.stereolabs.com/?SN='

    # we have a serial number - go get the config file from the config url

    r = requests.get(url+str(args.serial))

    if ((r.status_code == requests.codes.ok) and not("ERROR" in r.text)):

        with open("zed-cam-sn-"+str(args.serial)+".conf", "w") as config_file:
            config_file.write(r.text[0:])

        path_to_config_file = "zed-cam-sn-"+str(args.serial)+".conf"

    else:
        print("Error - failed to retrieve camera config from: " + url)
        print()
        parser.print_help()
        exit(1)

    camera_calibration_available = True

elif (len(args.config_file) > 0):

    path_to_config_file = args.config_file
    camera_calibration_available = True

elif (len(args.config_file_xml) > 0):

    path_to_config_file = args.config_file_xml
    manual_camera_calibration_available = True

else:
    print("Warning - no serial number or config file specified.")
    print()

################################################################################

# define video capture object as a threaded video stream

try:
    # to use a non-buffered camera stream (via a separate thread)

    import camera_stream
    zed_cam = camera_stream.CameraVideoStream()

except BaseException:
    # if not then just use OpenCV default

    print("INFO: camera_stream class not found - camera input may be buffered")
    zed_cam = cv2.VideoCapture()

zed_cam.open(args.camera_to_use)

print("INFO: camera backend is:", zed_cam.getBackendName())
print()

# check open and read video frame

if (zed_cam.isOpened()):
    ret, frame = zed_cam.read()
else:
    print("Error - selected camera #", args.camera_to_use, " : not found.")
    exit(1)

height,width, channels = frame.shape

################################################################################

# select config profiles based on image dimensions

# MODE  FPS     Width x Height  Config File Option
# 2.2K 	15 	    4416 x 1242     2K
# 1080p 30 	    3840 x 1080     FHD
# 720p 	60 	    2560 x 720      HD
# WVGA 	100 	1344 x 376      VGA

config_options_width = {4416: "2K", 3840: "FHD", 2560: "HD", 1344: "VGA"}
config_options_height = {1242: "2K", 1080: "FHD", 720: "HD", 376: "VGA"}

try:
    camera_mode = config_options_width[width]
except KeyError:
    print("Error - selected camera #", args.camera_to_use,
    " : resolution does not match a known ZED configuration profile.")
    print()
    exit(1)

print()
print("ZED left/right resolution: ", int(width/2), " x ",  int(height))
print("ZED mode: ", camera_mode)
print()
print("Keyboard Controls:")
print("space \t - change camera mode")
print("f \t - toggle disparity full-screen mode")
print("c \t - toggle disparity false colour mapping")
print("t \t - toggle display centre target cross-hairs and depth")
print("h \t - toggle horizontal side by side [left image | disparity]")
print("v \t - toggle vertical side by side [left image | disparity]")
print("o \t - toggle original live image display")
print("w \t - toggle left->right + right->left weighted least squares filtering")
print("i \t - toggle disparity in-filling via interpolation")
print("x \t - exit")
print()

################################################################################

# process config to get camera calibration from calibration file
# by parsing camera configuration as an INI format file

if (camera_calibration_available):
    cam_calibration = configparser.ConfigParser()
    cam_calibration.read(path_to_config_file)
    fx, fy, B, Kl, Kr, R, T, Q = zed_camera_calibration(cam_calibration, camera_mode, width, height)

    # correct factory supplied values if specified

    if ((args.correct_focal_length) and (camera_mode == "VGA")):
        fx = fx / 2.0
        fy = fy / 2.0
        Q[0][3] =  -1 * (width / 4)
        Q[1][3] = -1 * (height / 2)
        Q[2][3] = fx
        Q[3][3] = 0 # as Lcx == Rcx
elif (manual_camera_calibration_available):
    fx, fy, B, Kl, Kr, R, T, Q = read_manual_calibration(path_to_config_file)
    # no correction needed here

################################################################################

# define display window names

windowName = "Live Camera Input" # window name
windowNameD = "Stereo Disparity" # window name

################################################################################

# set up defaults for stereo disparity calculation

max_disparity = 128
window_size = 21

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

# set up left to right + right to left left->right + right->left matching +
# weighted least squares filtering (not used by default)

left_matcher = stereoProcessor
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

wls_lmbda = 800
wls_sigma = 1.2

wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(wls_lmbda)
wls_filter.setSigmaColor(wls_sigma)

################################################################################

# if camera is successfully connected

if (zed_cam.isOpened()) :

    # create window by name (as resizable)

    if (not(args.nooriginal)):
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, width, height)

    cv2.namedWindow(windowNameD, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowNameD, int(width/2), height)

    # if calibration is available then set call back to allow for depth display
    # on left mouse button click

    if (camera_calibration_available):
        cv2.setMouseCallback(windowNameD,on_mouse_display_depth_value, (fx, B))

    # if specified add trackbars
    if (args.showcontrols):
        cv2.createTrackbar("Max Disparity(x 16): ", windowNameD, int(max_disparity/16), 16, on_trackbar_set_disparities)
        cv2.createTrackbar("Window Size: ", windowNameD, window_size, 50, on_trackbar_set_blocksize)
        cv2.createTrackbar("Speckle Window: ", windowNameD, 0, 200, on_trackbar_set_speckle_window)
        cv2.createTrackbar("LR Disparity Check Diff:", windowNameD, 0, 25, on_trackbar_set_setDisp12MaxDiff)
        cv2.createTrackbar("Disparity Smoothness P1: ", windowNameD, 0, 4000, on_trackbar_set_setP1)
        cv2.createTrackbar("Disparity Smoothness P2: ", windowNameD, 0, 16000, on_trackbar_set_setP2)
        cv2.createTrackbar("Pre-filter Sobel-x- cap: ", windowNameD, 0, 5, on_trackbar_set_setPreFilterCap)
        cv2.createTrackbar("Winning Match Cost Margin %: ", windowNameD, 0, 20, on_trackbar_set_setUniquenessRatio)
        cv2.createTrackbar("Speckle Size: ", windowNameD, math.floor((width * height) * 0.0005), 10000, on_trackbar_null)
        cv2.createTrackbar("Max Speckle Diff: ", windowNameD, 16, 2048, on_trackbar_null)
        if (args.leftrightleft):
            cv2.createTrackbar("WLS Filter Lambda: ", windowNameD, wls_lmbda, 10000, on_trackbar_set_wlsLmbda)
            cv2.createTrackbar("WLS Filter Sigma Color (x 0.1): ", windowNameD, math.ceil(wls_sigma / 0.1), 50, on_trackbar_set_wlsSigmaColor)

    # loop control flags

    keep_processing = True

    while (keep_processing):

        # start a timer (to see how long processing and display takes)

        start_t = cv2.getTickCount()

        # if video file successfully open then read frame

        if (zed_cam.isOpened()):
            ret, frame = zed_cam.read()

            # when we reach the end of the video (file) exit cleanly

            if (ret == 0):
                keep_processing = False
                continue

        # split single ZED frame into left an right

        frameL= frame[:,0:int(width/2),:]
        frameR = frame[:,int(width/2):width,:]

        # remember to convert to grayscale (as the disparity matching works on grayscale)

        grayL = cv2.cvtColor(frameL,cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(frameR,cv2.COLOR_BGR2GRAY)

        # perform preprocessing - raise to the power, as this subjectively appears
        # to improve subsequent disparity calculation

        grayL = np.power(grayL, 0.75).astype('uint8')
        grayR = np.power(grayR, 0.75).astype('uint8')

        # compute disparity image from undistorted and rectified versions
        # (which for reasons best known to the OpenCV developers is returned scaled by 16)

        if (args.leftrightleft):
            displ = left_matcher.compute(cv2.UMat(grayL),cv2.UMat(grayR))  # .astype(np.float32)/16
            dispr = right_matcher.compute(cv2.UMat(grayR),cv2.UMat(grayL))  # .astype(np.float32)/16
            displ = np.int16(cv2.UMat.get(displ))
            dispr = np.int16(cv2.UMat.get(dispr))
            disparity = wls_filter.filter(displ, grayL, None, dispr)
        else:
            disparity_UMat = stereoProcessor.compute(cv2.UMat(grayL),cv2.UMat(grayR))
            disparity = cv2.UMat.get(disparity_UMat)

        if (args.showcontrols):
            speckleSize = cv2.getTrackbarPos("Speckle Size: ", windowNameD)
            maxSpeckleDiff = cv2.getTrackbarPos("Max Speckle Diff: ", windowNameD)
        else:
            speckleSize = math.floor((width * height) * 0.0005)
            maxSpeckleDiff = (8 * 16) # 128

        cv2.filterSpeckles(disparity, 0, speckleSize, maxSpeckleDiff)

        # scale the disparity to 8-bit for viewing
        # divide by 16 and convert to 8-bit image (then range of values should
        # be 0 -> max_disparity) but in fact is (-1 -> max_disparity - 1)
        # so we fix this also using a initial threshold between 0 and max_disparity
        # as disparity=-1 means no disparity available

        _, disparity = cv2.threshold(disparity,0, max_disparity * 16, cv2.THRESH_TOZERO)
        disparity_scaled = (disparity / 16.).astype(np.uint8)

        # fill disparity if requested

        if (args.fill_missing_disparity):
            _, mask = cv2.threshold(disparity_scaled,0, 1, cv2.THRESH_BINARY_INV)
            mask[:,0:120] = 0
            disparity_scaled = cv2.inpaint(disparity_scaled, mask, 2, cv2.INPAINT_NS)

        # display disparity - which ** for display purposes only ** we re-scale to 0 ->255

        if (args.colourmap):
            disparity_to_display = cv2.applyColorMap((disparity_scaled * (256. / max_disparity)).astype(np.uint8), cv2.COLORMAP_HOT)
        else:
            disparity_to_display = (disparity_scaled * (256. / max_disparity)).astype(np.uint8)

        # if requested draw target and display depth from centre of image

        if (args.showcentredepth):
            cv2.line(disparity_to_display, (int(width / 4) - 20, int(height / 2)),
                (int(width / 4) + 20, int(height / 2)), (255, 255, 255), 2)
            cv2.line(disparity_to_display, (int(width / 4), int(height / 2) - 20),
                (int(width / 4), int(height / 2) + 20), (255, 255, 255), 2)
            cv2.line(frameL, (int(width / 4) - 20, int(height / 2)),
                (int(width / 4) + 20, int(height / 2)), (255, 255, 255), 2)
            cv2.line(frameL, (int(width / 4), int(height / 2) - 20),
                (int(width / 4), int(height / 2) + 20), (255, 255, 255), 2)
            if (disparity_scaled[int(height / 2), int(width / 4)]):
                depth = fx * (B / disparity_scaled[int(height / 2), int(width / 4)])
                label = '{0:.3f}'.format(depth / 1000) + 'm'
                cv2.putText(disparity_to_display, label,(30,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # display side-by-side with left image as required

        if (args.sidebysideh):
            disparity_to_display = h_concatenate(frameL, disparity_to_display)
        elif (args.sidebysidev):
            disparity_to_display = v_concatenate(frameL, disparity_to_display)

        cv2.imshow(windowNameD, disparity_to_display)

        # switch between fullscreen and small - as required

        cv2.setWindowProperty(windowNameD, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN & args.fullscreen)

        # display input image (combined left and right)

        if (not(args.nooriginal)):
            cv2.imshow(windowName,frame)

        # stop the timer and convert to ms. (to see how long processing and display takes)

        stop_t = ((cv2.getTickCount() - start_t)/cv2.getTickFrequency()) * 1000

        # start the event loop - essential
        # wait 40ms or less depending on processing time taken (i.e. 1000ms / 25 fps = 40 ms)

        key = cv2.waitKey(max(2, 40 - int(math.ceil(stop_t)))) & 0xFF

        # user keyboard controls

        if (key == ord('x')):
            keep_processing = False
        elif (key == ord('c')):
            args.colourmap = not(args.colourmap)
        elif (key == ord('f')):
            args.fullscreen = not(args.fullscreen)
        elif (key == ord('i')):
            args.fill_missing_disparity = not(args.fill_missing_disparity)
        elif (key == ord('t')):
            args.showcentredepth = not(args.showcentredepth)
        elif (key == ord('h')):
            args.sidebysideh = not(args.sidebysideh)
        elif (key == ord('v')):
            args.sidebysidev = not(args.sidebysidev)
        elif (key == ord('o')):
            args.nooriginal = not(args.nooriginal)
        elif (key == ord('w')):
            args.leftrightleft = not(args.leftrightleft)
        elif (key == ord(' ')):

            # cycle camera resolutions to get the next one on the list

            pos = 0
            list_widths = list(config_options_width.keys())
            list_heights = list(config_options_height.keys())

            for (width_resolution, config_name) in config_options_width.items():

                    if (list_widths[pos % len(list_widths)] == width):

                        camera_mode = config_options_width[list_widths[(pos+1) % len(list_widths)]]

                        # get new camera resolution

                        width_new = next(key for key, value in config_options_width.items() if value == camera_mode)
                        height_new = next(key for key, value in config_options_height.items() if value == camera_mode)

                        print ("Changing camera config to use: ", camera_mode, " @ ", width, " x ", height)
                        break

                    pos+=1

            # reset to new camera resolution

            zed_cam.set(cv2.CAP_PROP_FRAME_WIDTH, width_new)
            zed_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height_new)

            zed_cam.read()

            width = int(zed_cam.get(cv2.CAP_PROP_FRAME_WIDTH))
            height =  int(zed_cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

            print("Camera config confirmed back from camera as: ", width , " x ", height)
            print()
            print("ZED left/right resolution: ", int(width/2), " x ",  int(height))
            print("ZED mode: ", camera_mode)
            print()

            # reset window sizes

            cv2.resizeWindow(windowName, width, height)
            cv2.resizeWindow(windowNameD, int(width/2), height)

            # get calibration for new camera resolution

            if (camera_calibration_available):
                fx, fy, B, Kl, Kr, R, T, Q = zed_camera_calibration(cam_calibration, camera_mode, width, height)

    # release camera

    zed_cam.release()

else:
    print("Error - no camera connected.")

################################################################################
