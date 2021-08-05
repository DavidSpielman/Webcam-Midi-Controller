import cv2 as cv
# 0 for built in webcam, 2 for external webcam
cap = cv.VideoCapture(2, cv.CAP_V4L2) # this added VideoCapture API allows camera fps to be changed from initial 5fps
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640) # 640 x 480 is 480p (30fps on this cam), 1280 x 720 is 720p (7.5 fps on this cam)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

tracker = cv.legacy.TrackerCSRT_create()
ret, frame = cap.read()
flip_frame = cv.flip(frame, 1)
boundingBox = cv.selectROI('MIDI Controller', flip_frame, True)
tracker.init(flip_frame, boundingBox)

def showROI(flip_frame, boundingBox):
    x, y, w, h = int(boundingBox[0]), int(boundingBox[1]), int(boundingBox[2]), int(boundingBox[3])
    center_x = int(x+(w/2))
    center_y = int(y+(h/2))
    cv.rectangle(flip_frame, (x, y), (x+w, y+h), (255, 0, 0), 5)
    dot = cv.circle(flip_frame, (center_x,center_y), 5, (0,0,255), -1)


if not cap.isOpened():
    print('Could not open camera')
    exit()

while True:
    ret, frame = cap.read()
    flip_frame = cv.flip(frame, 1) # flips video for better user experience
    if ret == True:
        showROI(flip_frame, boundingBox)
    else:
        print('Could not return frame')
    cv.imshow('MIDI Controller', flip_frame)
    ret, boundingBox = tracker.update(flip_frame)
    if cv.waitKey(1) == 27: # uses the escape key to exit the program (on Ubuntu)
        break

cap.release()
cv.destroyAllWindows()
