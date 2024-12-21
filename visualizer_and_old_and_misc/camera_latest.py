import cv2
import numpy as np
import pupil_apriltags as apriltag
import multiprocessing
import time
from pupil_apriltags import Detector
from controller_latest import ControllerInterface


if __name__ == "__main__":
    motor = ControllerInterface(rod_len=100)
    """Camera process to read frames, calculate theta, and log locally."""
    def deblur_image(image):
        kernel = np.array([[0, -1, 0],
                        [-1, 5, -1],
                        [0, -1, 0]])  # Sharpening kernel
        deblurred = cv2.filter2D(image, -1, kernel)
        return deblurred

    def is_blurry(image, threshold=100):
        variance = cv2.Laplacian(image, cv2.CV_64F).var()
        return variance < threshold

    # Initialize AprilTag detector
    detector = Detector(
        families="tag36h11",
    )


    # Open camera
    cap = cv2.VideoCapture(1)
    # "WhatsApp.mp4"
    # cap = cv2.VideoCapture("vid2.mp4")

    if not cap.isOpened():
        print("Error: Could not open the camera.")
        exit()

    while True:

        ret, frame = cap.read()
        if not ret:
            
            break
        
        timestamp = time.time()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #gray = deblur_image(gray)


        # Create masks for dark and bright regions
        dark_mask = cv2.inRange(gray, 0, 100)  # Dark regions (intensity 0 to 50)
        bright_mask = cv2.inRange(gray, 200, 255)  # Bright regions (intensity 180 to 255)

        # Combine the masks
        combined_mask = cv2.bitwise_or(dark_mask, bright_mask)

        # Create a gray background (mid-gray intensity 128)
        neutral_gray = np.full_like(gray, 255)

        # Apply the mask to keep only the desired regions
        filtered_frame = cv2.bitwise_and(gray, gray, mask=combined_mask)

        # Set everything outside the mask to neutral gray
        gray = np.where(combined_mask > 0, filtered_frame, neutral_gray)

        # Detect AprilTags
        results = detector.detect(gray)

        pts_base = None
        pts_weight = None

        for detection in results:
            if detection.tag_id == 0:
                pts_base = np.array(detection.corners, dtype=np.int32)
            elif detection.tag_id == 1:
                pts_weight = np.array(detection.corners, dtype=np.int32)
        base_positions = []
        weight_positions = []
        
        




        if pts_base is not None and pts_weight is not None:
            # Calculate positions and angle
            base_pos = np.mean(pts_base, axis=0)
            weight_pos = np.mean(pts_weight, axis=0)

            delta_x = weight_pos[0] - base_pos[0]
            delta_y = weight_pos[1] - base_pos[1]
            angle = 90 + np.arctan2(delta_y, delta_x) * (180 / np.pi)  # in degrees
            

            motor.updateAngle(-angle)
            # Draw line and annotate frame
            cv2.line(frame, tuple(base_pos.astype(int)), tuple(weight_pos.astype(int)), (0, 255, 0), 3)
            cv2.putText(frame, f"Angle: {angle:.2f} degrees", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f"Timestamp: {timestamp:.2f}s", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        
        print("ghgh")
        cv2.imshow("Camera Stream",frame)
        cv2.waitKey(1)
       
        


    cap.release()
    cv2.destroyAllWindows()

