# -*- coding: utf-8 -*-
"""
Created on Thu May  7 14:02:24 2020

@author: ACER
"""

import numpy as np
import cv2
from collections import deque
from PIL import ImageGrab


#Global Variables
# Upper and lower boundaries
lowerColor = np.array([100, 60, 60])
upperColor = np.array([140, 255, 255])

# Kernel for erosion and dilation
kernel = np.ones((5, 5), np.uint8)





# Toggles
eraserMode = False
disabledMode = False
recordMode = False

# Rectangle dimensions

# Draw buttons like colored rectangles on the white image
rect_top = 450
rect_height = 50
rect_bottom = rect_top + rect_height






#Paint-related
""" 
    Stores pixels of different colors 
    in different array in deques
"""
points = [deque(maxlen=512)]
# Index variable for each color 
index = 0




def prepare_canvas(canvas):

    #canvas = cv2.rectangle(canvas, (160,rect_top), (255,rect_bottom), colors[0], -1)
    #canvas = cv2.rectangle(canvas, (275,rect_top), (370,rect_bottom), colors[1], -1)
    #canvas = cv2.rectangle(canvas, (390,rect_top), (485,rect_bottom), colors[2], -1)
    #canvas = cv2.rectangle(canvas, (505,rect_top), (600,rect_bottom), colors[3], -1)
    
    if recordMode:
        center_coord = (450, 50)
        record_color = (0, 153, 255)
        radius = 30
        cv2.circle(canvas, center_coord, radius, record_color, -1)
        cv2.putText(canvas, "REC", (center_coord[0] + radius + 10, center_coord[1] + radius - 10), cv2.FONT_HERSHEY_SIMPLEX, 2, record_color,  1, cv2.LINE_AA)
    
    # Label the rectanglular boxes drawn on the image
    if not disabledMode:
        cv2.putText(canvas, "[D] DISABLE   [E] ERASE    [R] RECORD", (int(canvas.shape[1] / 5), rect_top), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    #cv2.putText(canvas, "BLUE", (185, rect_top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    #cv2.putText(canvas, "GREEN", (298, rect_top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    #cv2.putText(canvas, "RED", (420, rect_top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    #cv2.putText(canvas, "YELLOW", (520, rect_top), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150,150,150), 2, cv2.LINE_AA)#


def create_mask(hsv_frame):
    mask = cv2.inRange(hsv_frame, lowerColor, upperColor)
    mask = cv2.erode(mask, kernel, iterations = 2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.dilate(mask, kernel, iterations = 1)


def put_point(canvas_frame, center):
    
    global points
    global index
  
    points[index].appendleft(center)
 
    
"""
    Persistence = number of lines before line disappears (-1 means forever)
"""
def redraw_points(frame, preclear = True, persistence = -1):
    if preclear:
        frame.fill(255)
    for i in range(len(points)):
        for j in range(1, len(points[i])):
            if points[i][j - 1] is None or points[i][j] is None:
                continue
            color = (255, 0 ,0)
            if persistence == -1:
                cv2.line(frame, points[i][j - 1], points[i][j], color, 2)
            elif len(points) - 1 == i and abs(j - len(points)) < persistence:
                cv2.line(frame, points[i][j - 1], points[i][j], color, 2)

                
    

"""
    returns the current frame on-screen
"""

def capture_screen():
    screen_capture =  ImageGrab.grab()
    screen_capture = np.array(screen_capture,dtype='uint8')
    #.reshape((screen_capture.size[1], screen_capture.size[0], 3))
    #screen_capture = np.reshape(screen_capture, (1080, 1920, 3))
    screen_capture =  cv2.resize(screen_capture, (1920, 1080))
    screen_capture = cv2.cvtColor(screen_capture, cv2.COLOR_BGR2RGB)
    return screen_capture


def erase_point(center, radius):
    global index
    for i in range(len(points)):
        for j in range(1, len(points[i])):
            poi = points[i][j]
            if poi is None:
                continue
            if abs(poi[0] - center[0]) <= radius and abs(poi[1] - center[1] <= radius):
                points[i][j] = None

#.reshape((screen_capture.size[1],screen_capture.size[0],3)) 

def get_fresh_writer(capture_name):
    (height, width, _) = capture_screen().shape
    print(width)
    return cv2.VideoWriter(capture_name, cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))


if __name__ == "__main__":
    # Drawing canvas for hand
    # Create a window to display the above image (later)
    cv2.namedWindow('Paint', cv2.WINDOW_AUTOSIZE)    
    paintWindow = np.zeros((471, 636, 3)) + 255
    video = cv2.VideoCapture(0) # connection to camera 0
    record_writer = get_fresh_writer("recording.avi")
    # Processing
    while True:
        (status, frame) = video.read()
        # Flip the frame
        frame = cv2.flip(frame, 1) 
        # Converts RGB pixels to HSV counterpart
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        prepare_canvas(frame)
        if not status:
            break
        if recordMode:
            recording_frame = capture_screen()
                # cv2.imshow("Recording", recording_frame)
            record_writer.write(recording_frame)  
        if not disabledMode:
            # Find target object (hand)
            mask = create_mask(hsv_frame)
            
            # Find contours
            (cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(cnts) > 0:
                cnt = sorted(cnts, key = cv2.contourArea, reverse = True)[0] # Largest contour
                # Get the radius of the enclosing circle around the found contour
                ((x, y), radius) = cv2.minEnclosingCircle(cnt)
                
                # Draw the circle around the contour
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 255), 2)
                
                M = cv2.moments(cnt)
                
                center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00'] ))
                if not eraserMode:
                    put_point(paintWindow, center)
                else:
                    erase_point(center, radius)
                redraw_points(frame, preclear = False, persistence = 512)
                redraw_points(paintWindow)
            
            else:
                points.append(deque(maxlen=512))
                index += 1
        
        # Update frame
        cv2.imshow("Frame", frame)
        cv2.imshow("Paint", paintWindow)
        pressedKey = cv2.waitKey(1)
        if pressedKey & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            record_writer.release()
            video.release()
            break
        elif pressedKey & 0xFF == ord("e"):
            eraserMode = not eraserMode #toggle
        elif pressedKey & 0xFF == ord("d"):
            disabledMode = not disabledMode
            put_point(paintWindow, None)
        elif pressedKey & 0xFF == ord("r"):
            recordMode = not recordMode
            
                
            
            
                    
                
        
    
        
    