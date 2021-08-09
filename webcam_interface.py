import cv2 as cv
global ret, frame, flip_frame, boundingBox

# 0 for built in webcam, 2 for external webcam
cap = cv.VideoCapture(0, cv.CAP_V4L2) # this added VideoCapture API allows camera fps to be changed from initial 5fps
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640) # 640 x 480 is 480p (30fps on this cam), 1280 x 720 is 720p (7.5 fps on this cam)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

tracker = cv.legacy.TrackerCSRT_create()
ret, frame = cap.read()
flip_frame = cv.flip(frame, 1) # flips video for better user experience
boundingBox = cv.selectROI('MIDI Controller', flip_frame, True)
tracker.init(flip_frame, boundingBox)

def showROI(flip_frame, boundingBox):
    x, y, w, h = int(boundingBox[0]), int(boundingBox[1]), int(boundingBox[2]), int(boundingBox[3])
    center_x = int(x+(w/2))
    center_y = int(y+(h/2))
    cv.rectangle(flip_frame, (x, y), (x+w, y+h), (255, 0, 0), 5)
    dot = cv.circle(flip_frame, (center_x,center_y), 5, (0,0,255), -1)
    # if dot is in drawn rectangular region play note
    
if not cap.isOpened():
    print('Could not open camera')
    exit()

while True:
    ret, frame = cap.read()
    flip_frame = cv.flip(frame, 1) 
    if ret == True:
        showROI(flip_frame, boundingBox)
    else:
        print('Could not return frame')
    cv.imshow('MIDI Controller', flip_frame)
    ret, boundingBox = tracker.update(flip_frame)
    if cv.waitKey(1) == 27: # uses the escape key to exit the program (on Ubuntu)
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
