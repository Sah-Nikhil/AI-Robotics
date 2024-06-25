import cv2 as cv
import numpy as np


def ballDetection(frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lowerShade = np.array([20, 100, 100])
    upperShade = np.array([30, 255, 255])
    mask = cv.inRange(hsv, lowerShade, upperShade)

    #  # Remove noise with morphological operations
    # kernel = np.ones((5, 5), np.uint8)
    # mask = cv.erode(mask, kernel, iterations=2)
    # mask = cv.dilate(mask, kernel, iterations=2)
    
    # gaussianBlur
    mask = cv.GaussianBlur(mask, (15, 15), 0)

    #houghCircles
    circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT, dp = 1.2, minDist=30, 
                              param1=50, param2=30, minRadius=10, maxRadius=400)
    # drawCircles
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x,y,r) in circles:
            cv.circle(frame, (x,y), r, (0,255,0), 4)
            cv.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

    return(frame)



def main():
        
    videoCapture = cv.VideoCapture(0)

    while True:
        ret, frame = videoCapture.read()
        if not ret:
            break
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
        output = ballDetection(frame)
        
        cv.imshow('frame', output)
    
    
    # hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # lower = np.array([0, 0, 0])
    # upper =
    
    videoCapture.release()
    cv.destroyAllWindows()
    
if __name__ == "__main__":
    main()
