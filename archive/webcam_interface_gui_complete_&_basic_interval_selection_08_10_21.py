import cv2 as cv
global ret, frame, flip_frame, boundingBox

# 0 for built in webcam, 2 for external webcam
cap = cv.VideoCapture(0, cv.CAP_V4L2) # This added VideoCapture API allows camera fps to be changed from initial 5fps
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280) # 640 x 480 is 480p (30fps on this cam), 1280 x 720 is 720p (7.5 fps on this cam)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

tracker = cv.legacy.TrackerCSRT_create()
ret, frame = cap.read()
flip_frame = cv.flip(frame, 1) # Mirrors live video for better user experience
boundingBox = cv.selectROI('MIDI Controller', flip_frame, True)
tracker.init(flip_frame, boundingBox)

def setup_gui():
    global width, height
    width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
    thickness = 2
    color = (0,255,0)
    font = cv.FONT_HERSHEY_DUPLEX
    font_size = 4
    font_color = (0,255,0)
    font_thickness = 4
    line_type = cv.LINE_AA
    # Rectangles for Note Interval Selection
    top_left = cv.rectangle(flip_frame, (0,0), (int(width/3),0), color, thickness) 
    top_center = cv.rectangle(flip_frame, (int(width/3),0), (int(2*width/3),int(height/3)), color, thickness)
    top_right = cv.rectangle(flip_frame, (int(2*width/3),0), (int(width),int(height/3)), color, thickness)
    center_left = cv.rectangle(flip_frame, (0,int(height/3)), (int(width/3),int(2*height/3)), color, thickness)
    center_right = cv.rectangle(flip_frame, (int(2*width/3),int(height/3)), (int(width),int(2*height/3)), color, thickness)
    bottom_left = cv.rectangle(flip_frame, (0,int(2*height/3)), (int(width/3),int(height)), color, thickness) 
    bottom_right = cv.rectangle(flip_frame, (int(2*width/3),int(2*height/3)), (int(width),int(height)), color, thickness)
    # Text Placement for Note Intervals
    one = cv.putText(flip_frame,'1',(int(width/8),int(9*height/10)), font, font_size,font_color,font_thickness,line_type)
    two = cv.putText(flip_frame,'2',(int(width/8),int(5.5*height/10)), font, font_size,font_color,font_thickness,line_type)
    three = cv.putText(flip_frame,'3',(int(width/8),int(2.25*height/10)), font, font_size,font_color,font_thickness,line_type)
    four = cv.putText(flip_frame,'4',(int(3.75*width/8),int(2.25*height/10)), font, font_size,font_color,font_thickness,line_type)
    five = cv.putText(flip_frame,'5',(int(6.45*width/8),int(2.25*height/10)), font, font_size,font_color,font_thickness,line_type)
    six = cv.putText(flip_frame,'6',(int(6.45*width/8),int(5.5*height/10)), font, font_size,font_color,font_thickness,line_type)
    seven = cv.putText(flip_frame,'7',(int(6.45*width/8),int(9*height/10)), font, font_size,font_color,font_thickness,line_type)

def showROI(flip_frame, boundingBox):
    global center_x, center_y
    x, y, w, h = int(boundingBox[0]), int(boundingBox[1]), int(boundingBox[2]), int(boundingBox[3])
    center_x = int(x+(w/2))
    center_y = int(y+(h/2))
    cv.rectangle(flip_frame, (x, y), (x+w, y+h), (255, 0, 0), 5)
    dot = cv.circle(flip_frame, (center_x,center_y), 5, (0,0,255), -1)

def interval_selection(): # Uses the coordinates of the dot center to select intervals
    if center_x >= 0 and center_x <= int(width/3):
        if center_y >= int(2*height/3) and center_y <= int(height):
            print('Selected Interval 1')
        if center_y >= int(height/3) and center_y <= int(2*height/3):
            print('Selected Interval 2')
        if center_y >= 0 and center_y <= int(height/3):
            print('Selected Interval 3')
    elif center_x >= int(width/3) and center_x <= int(2*width/3):
        if center_y >= 0 and center_y <= int(height/3):
            print('Selected Interval 4')
    elif center_x >= int(2*width/3) and center_x <= int(width):
        if center_y >= 0 and center_y <= int(height/3):
            print('Selected Interval 5')
        if center_y >= int(height/3) and center_y <= int(2*height/3):
            print('Selected Interval 6')
        if center_y >= int(2*height/3) and center_y <= int(height):
            print('Selected Interval 7')
    
if not cap.isOpened():
    print('Could not open camera')
    exit()

while True:
    ret, frame = cap.read()
    flip_frame = cv.flip(frame, 1) 
    if ret == True:
        setup_gui()
        showROI(flip_frame, boundingBox)
        interval_selection()
    else:
        print('Could not return frame')
    cv.imshow('MIDI Controller', flip_frame)
    ret, boundingBox = tracker.update(flip_frame)
    if cv.waitKey(1) == 27: # Uses the escape key to exit the program (on Ubuntu)
        break
    elif cv.waitKey(1) == ord('r'): # Allows user to resets ROI when the 'r' key is held
        tracker = cv.legacy.TrackerCSRT_create()
        ret, frame = cap.read()
        flip_frame_updated = cv.flip(frame, 1)
        boundingBox_updated = cv.selectROI('MIDI Controller', flip_frame, True)
        tracker.init(flip_frame_updated, boundingBox_updated)
        showROI(flip_frame_updated, boundingBox_updated)
        ret, boundingBox = tracker.update(flip_frame)
        print('ROI Reset')

cap.release()
cv.destroyAllWindows()
