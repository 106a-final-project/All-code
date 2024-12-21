import csv
import time
import cv2
import numpy as np
from controller_latest import ControllerInterface
from collections import deque
import just_gripper

ROD_LEN = 280

if __name__ == '__main__':
    gripper_controller = just_gripper.GripperControl()
    gripper_controller.open_gripper()

    prev_angle, prev_time = None, None
    angle_offset = 0.0
    stable_cnt = 0
    velocity_buf = deque(maxlen=20)
    stability_threshold = 2
    frame_req = 20
    cnt = 0
    init_angle = 0

    # Open camera
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    #cap = cv2.VideoCapture("vid2.mp4")
    motor = ControllerInterface(rod_len=280)
    time.sleep(0.1)

    if not cap.isOpened():
        print("Error: Could not open the camera.")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        timestamp = time.time()

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for green
        lower_color_1 = np.array([25, 139, 100])
        upper_color_1 = np.array([68, 255, 255])


        # Define HSV range for yellow
        lower_color_2 = np.array([147, 218, 74])
        upper_color_2 = np.array([179, 255, 186])




        # Create a mask for green regions
        color_1_mask = cv2.inRange(hsv, lower_color_1, upper_color_1)
        color_2_mask = cv2.inRange(hsv, lower_color_2, upper_color_2)


        y_idxs_1, x_idxs_1 = np.where(color_2_mask > 0)
        if len(x_idxs_1) > 0 and len(y_idxs_1) > 0:
            base_pos = np.array([np.mean(x_idxs_1), np.mean(y_idxs_1)])
        else:
            base_pos = np.array([0, 0])    # No detected region

        # Calculate the centroid for color_2_mask
        y_idxs_2, x_idxs_2 = np.where(color_1_mask > 0)
        if len(x_idxs_2) > 0 and len(y_idxs_2) > 0:
            weight_pos = np.array([np.mean(x_idxs_2), np.mean(y_idxs_2)])
        else:
            weight_pos = np.array([0, 0])  # No detected region

        if base_pos is not None and weight_pos is not None:
            # Calculate positions and angle
            delta_x = weight_pos[0] - base_pos[0]
            delta_y = weight_pos[1] - base_pos[1]

            angle = -np.arctan2(delta_x, delta_y) * (180 / np.pi) + 180  # in degrees
            if angle > 180:
                angle = angle - 360

            # Calculate time difference (dt)
            if prev_time is None:
                prev_time = time.time()
            current_time = time.time()
            dt = current_time - prev_time + 0.00001
            prev_time = current_time
            # Calculate angular velocity
            if prev_angle is None:
                prev_angle = 0

            angle_velocity = (angle - prev_angle) / dt
            prev_angle = angle

            velocity_buf.append(angle_velocity)
            cnt += 1
            if len(velocity_buf) == velocity_buf.maxlen:
                vel_array = np.array(velocity_buf)
                if np.std(vel_array) < stability_threshold:
                    stable_count += 1
                    if stable_count >= frame_req:
                        angle_offset = angle
                else:
                    stable_count = 0

            # Calculate the angle
            angle = angle

            # Define the current state
            current = np.array([0, angle, angle_velocity, 0])  # Assuming placeholders for other values

            cv2.line(frame, tuple(base_pos.astype(int)), tuple(weight_pos.astype(int)), (0, 255, 0), 3)
            cv2.putText(frame, f"Angle: {angle:.2f} degrees", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f"Timestamp: {timestamp:.2f}s", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            # if angle > 4 or angle < -4:


            #print(abs(angle))
            if abs(angle) > 15 and cnt > 5:
                print("ANGLE DETECTED")
                gripper_controller.close_gripper()
                motor.stop()
                motor == None

            # if angle > 0 and angle < 70 and cnt > 5:
            #     print("ANGLE DETECTED")
            #     gripper_controller.close_gripper()
            #     motor.stop()
            #     motor == None

            radians = -angle *  np.pi/180
            com = np.sin(radians) * ROD_LEN * 0.5
            # overshoot = 30 * np.sign(-angle_velocity)
            # motor.updateAngle(com * 1.25 + overshoot)
            # overshoot = 25 * np.sign(-angle_velocity)
            # motor.updateAngle(com * 5 + overshoot)

        # Display the frame
        # cv2.imshow("Filtered Green Regions", color_1_mask + color_2_mask)
        cv2.imshow("Filtered Green Regions", frame)


        cnt += 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
