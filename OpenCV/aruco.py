import cv2
import numpy as np

def main():
    # Define the dictionary and parameters for ArUco marker detection
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()

    # Set up the webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video stream from webcam.")
        return

    # Load the camera calibration parameters
    camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture image.")
            break

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            print(f"Detected markers: {ids.flatten()}")

            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose of each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)

            for rvec, tvec in zip(rvecs, tvecs):
                # Draw axis for the marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # Calculate the distance from the camera to the marker
                distance = np.linalg.norm(tvec)
                distance_text = f"Distance: {distance:.2f} m"

                # Draw a line from the camera center to the marker center
                camera_center = (int(camera_matrix[0, 2]), int(camera_matrix[1, 2]))
                marker_center = (int(corners[0][0][0][0]), int(corners[0][0][0][1]))
                cv2.line(frame, camera_center, marker_center, (0, 255, 0), 2)

                # Put the distance text on the frame
                cv2.putText(frame, distance_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                # Print the rotation and translation vectors (pose information)
                print(f"Rotation Vector: {rvec}")
                print(f"Translation Vector: {tvec}")
        else:
            print("No markers detected.")

        # Display the resulting frame
        cv2.imshow('ArUco Marker Detection', frame)

        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
