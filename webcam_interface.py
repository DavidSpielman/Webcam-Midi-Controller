import cv2 as cv

cap = cv.VideoCapture(2, cv.CAP_V4L2) # this added VideoCapture API allows camera fps to be changed from initial 5fps
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640) # 640 x 480 is 480p (30fps on this cam), 1280 x 720 is 720p (7.5 fps on this cam)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print('Could not open camera')
    exit()

while True:
    ret, frame = cap.read()
    flip_frame = cv.flip(frame, 1) # flips video for better user experience
    cv.imshow('MIDI Controller', flip_frame)
    if cv.waitKey(1) == 27: # uses the escape key to exit the program (on Ubuntu)
        break

cap.release()
cv.destroyAllWindows()
