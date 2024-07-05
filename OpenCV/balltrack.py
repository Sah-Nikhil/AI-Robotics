import cv2 as cv
import numpy as np

def ballDetection(frame):
    # Convert the frame to different color spaces
    gauss = cv.GaussianBlur(frame, (9, 9), 10)
    median = cv.medianBlur(gauss, 5)
    framehsv = cv.cvtColor(median, cv.COLOR_BGR2HSV)
    framelab = cv.cvtColor(median, cv.COLOR_BGR2LAB)

    # Define the color ranges for masking
    lower_hsv = np.array([20, 70, 80])
    upper_hsv = np.array([38, 255, 255])
    lower_lab = np.array([50, 0, 146])
    upper_lab = np.array([255, 134, 191])

    # Create masks
    maskinglab = cv.inRange(framelab, lower_lab, upper_lab)
    maskinghsv = cv.inRange(framehsv, lower_hsv, upper_hsv)
    mask = cv.bitwise_or(maskinglab, maskinghsv)

    # Apply morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask_erode = cv.erode(mask, kernel, iterations=2)
    mask_dilate = cv.dilate(mask_erode, kernel, iterations=2)
    mask = cv.GaussianBlur(mask_dilate, (15, 15), 0)

    # Detect circles using HoughCircles
    circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT, dp=1.2, minDist=50, 
                              param1=50, param2=40, minRadius=10, maxRadius=400)

    # Draw the circles
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
    
    return frame

def main():
    videoCapture = cv.VideoCapture(0)

    while True:
        ret, frame = videoCapture.read()
        if not ret:
            break
        
        output = ballDetection(frame)
        
        cv.imshow('frame', output)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    
    videoCapture.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
