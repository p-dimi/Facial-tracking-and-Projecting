# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 13:47:12 2017

@author: Dima
"""
import time
import numpy as np
import math
import cv2
import sys

''' Currently works with webcams only '''

cam_index = int(input('What index is your webcam? If it is the built in webcam of the laptop, it is index 0.\nIf it is an externally added webcam, it is index 1.\nIf you do not have any webcams connected, except the built in webcam, simply type 0: '))

projector_resolution_width = int(input('What is the width resolution of the projector (in pixels)? : '))
projector_resolution_height = int(input('What is the height resolution of the projector? (in pixels): '))

print('The Projection Plane is the ideal position of the subject.')
print('For instance: If there is a wall on which there is a picture of a face, then that wall is the Projection Plane.')

camera_distance = int(input('What is the distance of the camera (front) from the Projection Plane? (in cm): '))
projector_distance = int(input('What is the distance of the projector (front) from the Projection Plane? (in cm): '))

print("The Vertical Field Of View (FOV) corresponds to a lens focal length.\nFor example - a standard 50mm full frame lens would have a vertical FOV of 27.0 degrees.\nThere are three types of FOV - Horizontal, Vertical, Diagonal. This code uses only the Vertical FOV.\nIf you don't know the Vertical FOV of your camera or projector, you can search for their specs, or calculate it using Trigonometry.")
print('This code comes with a rudimentary calculator for the Vertical FOV of your camera or projector.')
know_fov = str(input('Do you already know the Vertical FOV of your Camera and Projector? (y / n): ')).upper()

camera_vertical_fov = None
projector_vertical_fov = None

if know_fov == 'N':
    need_help = str(input('Do you need help with calculating the Vertical FOV of your Camera or Projector? (y / n): ')).upper()
    if need_help == 'Y':
        cam_or_proj = str(input('Do you need to know the Vertical FOV of your Camera or your Projector? (camera / projector/ both): ')).upper()
        if cam_or_proj == 'CAMERA':   
            print('Follow the next instructions:\n1) Put your camera with its front facing a wall straight on\n.2) Measure the distance between the middle of the lens lenghtwise, and the wall the lens is facing')
            b = int(input('What is the distance you measured (in cm)?: '))
            print('3) Put something you know the exact height of on the wall, such as a 5cm long strip of tape. Take a picture of the wall, and measure how many pixels tall is the 5cm strip of tape (or any other object you know the size of).\n')
            obj_cm = int(input('What is the height of the object you took a picture of (in cm)?: '))
            obj_px = int(input('How many pixels tall is the object (in pixels)?: '))
            cam_res = int(input('What is your vertical camera resolution? For example - a camera of 640 x 480 pixels has a vertical resolution of 480 pixels. Input the vertical resolution: '))
            ratio = obj_px / cam_res
            picture_height_cm = obj_cm / ratio
            print('The total height of the picture, in cm, is approximately {} (in case you are curious)'.format(int(picture_height_cm)))
            a = picture_height_cm / 2
            camera_vertical_fov = math.degrees(math.atan(a/b))
            print('The Vertical FOV of your camera is approximately {}'.format(camera_vertical_fov))
        elif cam_or_proj == 'PROJECTOR':
            print('Follow the next instructions:\n1) Put your projector with its front facing a wall straight on\n.2) Measure the distance between the middle of the lens lenghtwise, and the wall the lens is facing')
            b = int(input('What is the distance you measured (in cm)?: '))
            print('3) Project something onto the wall. Measure the distance between the top and bottom of the projection (in cm)')
            proj_height_cm = int(input('What was the height that you measured between the top and bottom of the projection (in cm)? '))
            a = proj_height_cm / 2
            projector_vertical_fov = math.degrees(math.atan(a/b))
            print('The Vertical FOV of your projector is approximately {}'.format(projector_vertical_fov))
        else:
            print('CAMERA VERTICAL FOV GETTING:')
            print('Follow the next instructions:\n1) Put your camera with its front facing a wall straight on\n.2) Measure the distance between the middle of the lens lenghtwise, and the wall the lens is facing')
            b = int(input('What is the distance you measured (in cm)?: '))
            print('3) Put something you know the exact height of on the wall, such as a 5cm long strip of tape. Take a picture of the wall, and measure how many pixels tall is the 5cm strip of tape (or any other object you know the size of).\n')
            obj_cm = int(input('What is the height of the object you took a picture of (in cm)?: '))
            obj_px = int(input('How many pixels tall is the object (in pixels)?: '))
            cam_res = int(input('What is your vertical camera resolution? For example - a camera of 640 x 480 pixels has a vertical resolution of 480 pixels. Input the vertical resolution: '))
            ratio = obj_px / cam_res
            picture_height_cm = obj_cm / ratio
            print('The total height of the picture, in cm, is approximately {} (in case you are curious)'.format(int(picture_height_cm)))
            a = picture_height_cm / 2
            camera_vertical_fov = math.degrees(math.atan(a/b))
            print('The Vertical FOV of your camera is approximately {}\n'.format(camera_vertical_fov))
            print('Camera Vertical FOV getting done. Now getting Projector Vertical FOV.\n\n')
            print('PROJECTOR VERTICAL FOV GETTING:')
            print('Follow the next instructions:\n1) Put your projector with its front facing a wall straight on\n.2) Measure the distance between the middle of the lens lenghtwise, and the wall the lens is facing')
            b = int(input('What is the distance you measured (in cm)?: '))
            print('3) Project something onto the wall. Measure the distance between the top and bottom of the projection (in cm)')
            proj_height_cm = int(input('What was the height that you measured between the top and bottom of the projection (in cm)? '))
            a = proj_height_cm / 2
            projector_vertical_fov = math.degrees(math.atan(a/b))
            print('The Vertical FOV of your projector is approximately {}'.format(projector_vertical_fov))
            print('Projector Vertical FOV getting done. Proceeding.\n')
    else:
        print('You need to have the Vertical FOV of your camera and your projector before continuing. Without these values the code will not work.')
        sys.exit()
else:
    c_v_fov = int(input("What is your camera's Vertical FOV (in degrees)?: "))
    camera_vertical_fov = c_v_fov / 2
    p_v_fov = int(input("What is your projector's Vertical FOV (in degrees)?: ")) 
    projector_vertical_fov = p_v_fov / 2

print("\nPROJECTOR OFFSET\nYour projector is likely not at the exact same location as your camera, as two objects cannot occupy the same exact space. So - let's get that offset:")
proj_h_dir = str(input('Looking at the camera from the front - is the projector to the left or to the right of it? (left / right): ')).upper()
if proj_h_dir == 'left':
    proj_h_dir = -1
elif proj_h_dir == 'right':
    proj_h_dir = 1

proj_h_dist = int(input("What is the horizontal distance from the middle of the projector's lens to the middle of the camera's lens? (in cm): "))
horizontal_offset = proj_h_dist * proj_h_dir

proj_v_dir = str(input('Looking at the camera from the front - is the projector above the camera or below it? (above / below): ')).upper()
if proj_v_dir == 'below':
    proj_v_dir = -1
elif proj_v_dir == 'above':
    proj_v_dir = 1

proj_v_dist = int(input("What is the vertical distance from the middle of the projector's lens to the middle of the camera's lens? (in cm): "))
vertical_offset = proj_v_dist * proj_v_dir

#Import face haar cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# CAP THE VIDEO FROM CAM & get it's resolution
cap=cv2.VideoCapture(cam_index)
time.sleep(2)
ret, frame_init = cap.read()

#print(ret)

frame_init=cv2.cvtColor(frame_init,cv2.COLOR_BGR2GRAY)

color1 = (0,255,255)
color2 = (255,255,0)

# the beta angle in the spread triangle
beta_cam = 180 -90 -camera_vertical_fov
beta_proj = 180 -90 -projector_vertical_fov

img_resolution_cam = (frame_init.shape[1], frame_init.shape[0])

projection_matrix_size_multiplier = 1

img_resolution_proj = (int(projector_resolution_width*projection_matrix_size_multiplier), int(projector_resolution_height*projection_matrix_size_multiplier))

def get_aspect_ratio(integer):
    divisors = []
    for i in range(2, integer+1):
        if (integer % i == 0):
            divisors.append(i)
    divisors.sort()
    return divisors[0]

aspect_ratio_cam = (get_aspect_ratio(img_resolution_cam[0]), get_aspect_ratio(img_resolution_cam[1]))
aspect_ratio_proj = (get_aspect_ratio(img_resolution_proj[0]), get_aspect_ratio(img_resolution_proj[1]))

# VARIABLES TO PLANT "get rect side scales" RESULTS
cam_scales = (0,0)
proj_scales = (0,0)


# CONVERT ALL THE DEGREES INTO RADIANS
camera_vertical_fov = np.deg2rad(camera_vertical_fov)
projector_vertical_fov = np.deg2rad(projector_vertical_fov)
beta_cam = np.deg2rad(beta_cam)
beta_proj = np.deg2rad(beta_proj)

# The projection matrix
projection_matrix = np.ones( (int(480*projection_matrix_size_multiplier),int(640*projection_matrix_size_multiplier),3) , dtype=np.uint8)

'''
# function that draws white circle in the projection matrix
def project_white(a_detect, b_detect, c_detect, d_detect):
    global projection_matrix
    x1 = a_detect[0]
    x2 = a_detect[0]
    
    y1 = c_detect[1]
    y2 = c_detect[1]
    #the part of the calibrated image in which the white porjection's gotta go
    projection_matrix[x1: x2, y1: y2] = 255
    
    return
'''

# for the full equation - m and the whole thing
# USED in "get rect side scales"
def math_m(alpha,beta):
    m = np.tan(0.5*(alpha-beta)) / np.tan(0.5*(alpha+beta))
    return m

def math_scale_a(d,m):
    a = d*((1+m) / (1-m))
    #keep in mind this is HALF a scale. This is also for Vertical measuring
    return a

# Retrieve the height and width of a rectangle
# USE THIS IN A FUNCTION
def get_rect_side_scales(aspect_ratio, distance, alpha, beta):
    global math_m
    global math_scale_a
    V, H = aspect_ratio
    vertical_scale_angle_parameter = math_m(alpha, beta)
    vertical_scale = 2 * math_scale_a(distance, vertical_scale_angle_parameter)
    horizontal_scale = (vertical_scale / V) * H
    return (vertical_scale, horizontal_scale)

"""
# Checks whether we have an overlapped rectangle
def check_overlap(h_offset, v_offset, rect_cam, rect_proj):
    h_c, v_c = rect_cam
    h_p, v_p = rect_proj
    scalar_h = (h_c / 2) + (h_p / 2)
    scalar_v = (v_c / 2) + (v_p / 2)
    if np.abs(h_offset) >= scalar_h or np.abs(v_offset) >= scalar_v:
        return False
    else: return True
"""
    
# This function will pull the raw pixel coordinates from the img with the face detection rectangle
# just the actual location of the pixels, along with the size in pixels of the image, and will convert to ratios
def detection_rect_get_ratio(a,b,c,d,resolution):
    m = a[0] / resolution[0]
    k = a[1] / resolution[1]
    face_width = np.abs(a[0] - b[0])
    face_height = np.abs(a[1] - d[1])
    n = (a[0] + face_width) / resolution[0]
    l = (a[1] + face_height) / resolution[1]
    return m, k, n, l

# this function is intended to use the above given ratios in order to convert the coordinates into our plane
# it will need to be fed the cam rectangle dimensions as they are in the plane
def detection_rect_in_plane(m, k, n, l, cam_rect_width, cam_rect_height):
    ad = (m * cam_rect_width, k * cam_rect_height)
    bd = (n * cam_rect_width, k * cam_rect_height)
    cd = (n * cam_rect_width, l * cam_rect_height)
    dd = (m * cam_rect_width, l * cam_rect_height)
    rect_width = np.abs(ad[0] - bd[0])
    rect_height = np.abs(ad[1] - dd[1])
    
    return rect_width, rect_height, ad, bd, cd, dd

#This is our logic for finding the overlap coordinates
# y,y means completely outside, n,n means completely inside
# REVISION - THIS FUNCTION SHOULD ONLY BE USED TO DETERMINE THE OVERLAP OF THE DETECTION RECTANGLE OVER THE
# PROJECTION RECTANGLE. FEED IN THIS ORDER: AP, AD, BP, BD, CP, CD, DP, DD
def get_overlap_rect(ac, ap, bc, bp, cc, cp, dc, dp):
    a = [0,0]
    b = [0,0]
    c = [0,0]
    d = [0,0]
    
    overlap_a = [0,0]
    overlap_b = [0,0]
    overlap_c = [0,0]
    overlap_d = [0,0]
    
    if ap[0] < ac[0]:
        a[0] = True
    else: a[0] = False
    if ap[1] < ac[1]:
        a[1] = True
    else: a[1] = False
    
    if a == [True,True]:
        overlap_a = ac
    elif a== [False,False]:
        overlap_a = ap
    elif a==[True,False]:
        overlap_a = (ac[0], ap[1])
    elif a==[False,True]:
        overlap_a = (ap[0], ac[0])
    
    if bp[0] > bc[0]:
        b[0] = True
    else: b[1] = False
    if bp[1] < bc[1]:
        b[1] = True
    else: b[1] = False
    
    if b==[True,True]:
        overlap_b=bc
    elif b==[False,False]:
        overlap_b=bp
    elif b==[True,False]:
        overlap_b=(bc[0],bp[1])
    elif b==[False,True]:
        overlap_b = (bp[0],bc[1])
    
    if cp[0] > cc[0]:
        c[0] = True
    else: c[0] = False
    if cp[1] > cc[1]:
        c[1] = True
    else: c[1] = False
    
    if c==[True,True]:
        overlap_c=cc
    elif c==[False,False]:
        overlap_c=cp
    elif c==[True,False]:
        overlap_c=(cc[0],cp[1])
    elif c==[False,True]:
        overlap_c=(cp[0],cc[1])
    
    if dp[0] < dc[0]:
        d[0] = True
    else: d[0] = False
    if dp[1] > dc[1]:
        d[1] = True
    else: d[1] = False
        
    if d==[True,True]:
        overlap_d = dc
    elif d==[False,False]:
        overlap_d = dp
    elif d==[True,False]:
        overlap_d = (dc[0], dp[1])
    elif d==[False,True]:
        overlap_d = (dp[0], dc[1])

    #print(overlap_a, overlap_b, overlap_c, overlap_d)

    scale_width = np.abs(overlap_d[0] - overlap_c[0])
    scale_height = np.abs(overlap_a[1] - overlap_d[1])
    
    overlap_a = tuple(overlap_a)
    overlap_b = tuple(overlap_b)
    overlap_c = tuple(overlap_c)
    overlap_d = tuple(overlap_d)
    
    return scale_width, scale_height, overlap_a, overlap_b, overlap_c, overlap_d

# generates NEW coordinates of overlapping rectangle, based on a new axis in which ap is (0,0)
# feed the "a" point of the projection rectangle, feed all points of the overlapping rectangle & dimensions
def get_overlap_coords_based_on_projector_rect(ap, overlap_a, overlap_width, overlap_height):
    new_overlap_a = (overlap_a[0] - ap[0], overlap_a[1] - ap[1])
    new_overlap_b = (new_overlap_a[0] + overlap_width, new_overlap_a[1])
    new_overlap_c = (new_overlap_b[0], new_overlap_b[1] + overlap_height)
    new_overlap_d = (new_overlap_a[0], new_overlap_c[1])
    return new_overlap_a, new_overlap_b, new_overlap_c, new_overlap_d
    
# Feed into this the overlap coordinates based on the "get_overlap_coords_base_on_projector_rect" function
# also feed the width and height of the projector rectangle
def get_trimmed_detect_ratio_by_projection(a, b, c, d, proj_width, proj_height):

    trimmed_overlap_width = np.abs(a[0] - b[0])
    trimmed_overlap_height = np.abs(a[1] - d[1])
    m = a[0] / proj_width
    k = a[1] / proj_height
    n = (a[0] + trimmed_overlap_width) / proj_width
    l = (a[1] + trimmed_overlap_height) / proj_height
    return m, k, n, l

# feed ratios receive from "get_trimmed_detect_ratio_by_projection" and the resolution of the projected image 
# This function will give us the final coordinates IN THE PROJECTED IMAGE in which there is a face :)
def get_detected_rectangle_in_projected_image(m, k, n, l, resolution_proj):
    a_detect = (np.int(m * resolution_proj[0]), np.int(k * resolution_proj[1]))
    b_detect = (np.int(n * resolution_proj[0]), np.int(k * resolution_proj[1]))
    c_detect = (np.int(n * resolution_proj[0]), np.int(l * resolution_proj[1]))
    d_detect = (np.int(m * resolution_proj[0]), np.int(l * resolution_proj[1]))
    detect_width = np.abs(a_detect[1] - b_detect[1])
    detect_height = np.abs(a_detect[0] - d_detect[0])
    return a_detect, b_detect, c_detect, d_detect, detect_width, detect_height


# BEFORE WE GO INTO THE LOOP - WE SET UP ALL THE PERMANENCE SCALES
# just the sizes of each rectangle's sides based on the projection spread angle and distance
cam_scales = get_rect_side_scales(aspect_ratio_cam, camera_distance, camera_vertical_fov, beta_cam)
proj_scales = get_rect_side_scales(aspect_ratio_proj, projector_distance, projector_vertical_fov, beta_proj)

# DEFINE BOTH RECTANGLES IN AXIS
# Width, height and center of camera rectangle
rect_cam = get_rect_side_scales(aspect_ratio_cam, camera_distance, camera_vertical_fov, beta_cam)
width_cam, height_cam = rect_cam
center_cam = (width_cam / 2, height_cam / 2)

# Width, height and center of projector rectangle in relation to Cam
rect_proj = get_rect_side_scales(aspect_ratio_proj, projector_distance, projector_vertical_fov, beta_proj)
width_proj, height_proj = rect_proj
center_proj = ((width_cam / 2) + horizontal_offset, (height_cam / 2) + vertical_offset)
ProjC_x, ProjC_y = center_proj

# points of cam rectangle, and of projector rectangle. THE ARE ALL DEFINED FOR RECTANGLES ONLY. No angle vs the wall.
# CAM POINTS
ac = (0,0)
bc = (width_cam, 0)
cc = (width_cam, height_cam)
dc = (0, height_cam)

# PROJ POINTS
ap = (ProjC_x - (width_proj / 2), ProjC_y - (height_proj / 2))
bp = (ProjC_x + (width_proj / 2), ProjC_y - (height_proj / 2))
cp = (ProjC_x + (width_proj / 2), ProjC_y + (height_proj / 2))
dp = (ProjC_x - (width_proj / 2), ProjC_y + (height_proj / 2))



cv2.namedWindow("projection", cv2.WND_PROP_FULLSCREEN)          
cv2.setWindowProperty("projection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# THE WHILE LOOP WHERE EVERYTHING HAPPENS
    
while True:
    _, frame = cap.read()
    cam_feed=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    # I want to get the two Cam and Proj rectangles now in the RL axis I defined by Cam
    
    
    # detect faces & select a random one
    faces = face_cascade.detectMultiScale(cam_feed, 1.1, 12)
    if len(faces) > 0:
        color=color1
        if len(faces) > 1:
            color=color2
        
        x, y, w, h = faces[np.random.randint(0, len(faces), dtype=np.uint8)]
        a1 = (x,y)
        b1 = (x+w,y)
        c1 = (x+w, y+h)
        d1 = (x, y+h)
        #print(a1,b1,c1,d1)
        # m k n and l ratios-by-cam of the detected face, converted into ratios-by-cam for use in our global axis
        m,k,n,l = detection_rect_get_ratio(a1,b1,c1,d1, img_resolution_cam)
        # get all the infos about the detected face in our global plain
        d_width, d_height, ad,bd,cd,dd = detection_rect_in_plane(m, k, n, l, cam_scales[0], cam_scales[1])
        '''this part checked and is okay'''
        
        
        
        """ WE NEED TO CHECK FOR OVERLAP AT ***THIS*** STAGE """
        
        # ADJUSTMENT ONLY FOR OVERLAP OVER PROJECTION RECTANGLE
        # PROJECTION RECTANGLE. FEED IN THIS ORDER: AP, AD, BP, BD, CP, CD, DP, DD
        d_width, d_height, ad,bd,cd,dd = get_overlap_rect(ap, ad, bp, bd, cp, cd, dp, dd)
        #print(ad,bd,cd,dd)
        '''at this point we're getting points that are all 0'''
        # convert into axis which is defined by projector rect
        
        ad, bd, cd, dd = get_overlap_coords_based_on_projector_rect(ap, ad, d_width, d_height)
        """ BROKEN OVERLAP COORDS - NEED FIX """
        #print(ad,bd,cd,dd)
    
        md, kd, nd, ld = get_trimmed_detect_ratio_by_projection(ad, bd, cd, dd, proj_scales[0], proj_scales[1])
        # get the final set of coordinates or scales in the projected image of where the face is
        a_detect, b_detect, c_detect, d_detect, detect_width, detect_height = get_detected_rectangle_in_projected_image(md, kd, nd, ld, img_resolution_proj)
        #print(a_detect,b_detect,c_detect,d_detect)
        # Finally - we get the picture the projector should project.
        #project_white(a_detect, b_detect, c_detect, d_detect)
        cv2.rectangle(projection_matrix, a_detect, c_detect, color, -1)
        #projection_matrix = cv2.flip(projection_matrix, 1)
        
    # WE WANNA SHOW THE PROJECTION MATRIX HERE, NOT THE IMAGE
    cv2.imshow('projection', projection_matrix)
    projection_matrix = projection_matrix * 0
    #if press q, close window. if press s, save frame for test purposes        
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
                
#release all resources and close all windows
cap.release()
cv2.destroyAllWindows()
