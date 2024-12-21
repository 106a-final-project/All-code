import cv2
import time
import numpy as np
import multiprocessing
import csv     
#from controller_latest import ControllerInterface
from collections import deque
import os

def camera_process(pipe, stop_event):
    # Open camera

    #cap = cv2.VideoCapture(0)
    # windows
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Error: Could not open the camera.")
        exit()

    base_pos = np.array([0, 0])  # No detected region
    weight_pos = np.array([0, 0])  # No detected region

    while not stop_event.is_set():  # Check if stop signal is set
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

        # Create a masks
        color_1_mask = cv2.inRange(hsv, lower_color_1, upper_color_1)
        color_2_mask = cv2.inRange(hsv, lower_color_2, upper_color_2)
        
        y_idxs_1, x_idxs_1 = np.where(color_2_mask > 0)
        if len(x_idxs_1) > 0 and len(y_idxs_1) > 0:
            base_pos = np.array([np.median(x_idxs_1), np.median(y_idxs_1)])
        else:
            base_pos = np.array([0, 0])    # No detected region

        # Calculate the centroid for color_2_mask
        y_idxs_2, x_idxs_2 = np.where(color_1_mask > 0)
        if len(x_idxs_2) > 0 and len(y_idxs_2) > 0:
            weight_pos = np.array([np.median(x_idxs_2), np.median(y_idxs_2)])
        else:
            weight_pos = np.array([0, 0])  # No detected region

        if base_pos is not None and weight_pos is not None:
            # Calculate positions and angle
            delta_x = weight_pos[0] - base_pos[0]
            delta_y = weight_pos[1] - base_pos[1]

            angle = -np.arctan2(delta_x, delta_y) * (180 / np.pi) + 180  # in degrees
            theta = angle

            timestamp = time.time()

            pipe.send((theta, timestamp))  # Send data to the motor process

            cv2.line(frame, tuple(base_pos.astype(int)), tuple(weight_pos.astype(int)), (0, 255, 0), 3)
            cv2.putText(frame, f"Angle: {theta:.2f} degrees", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f"Timestamp: {timestamp:.2f}s", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # Display the
        #cv2.imshow("input", color_1_mask + color_2_mask)
        cv2.imshow("input", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()
    cap.release()
    cv2.destroyAllWindows()


def touch_sensor_process(pipe, stop_event):
    time.sleep(0.1) 
    while not stop_event.is_set():
        # Simulate touch sensor logic
        pass


def motor_process(camera_pipe, touch_sensor_pipe, stop_event):
    prev_angle, prev_time = None, None
    prev_time = time.time()

    data_buffer = []

    data_count = 0

    prev_angle, prev_time = None, None
    angle_offset = 0.0
    stable_cnt = 0
    velocity_buf = deque(maxlen=20)
    stability_threshold = 2
    frame_req = 20
    cnt = 0
    init_angle = 0
    init_position = 0


    #motor = ControllerInterface(rod_len=100)
    time.sleep(0.1)

    while not stop_event.is_set():  # Check if stop signal is set
        if camera_pipe.poll():
            theta, timestamp = camera_pipe.recv()

            if prev_time is not None:
                dt = timestamp - prev_time
                angle_velocity = (theta - prev_angle) / dt if prev_angle is not None else 0
                
                #TODO this should recive motor inputs########
                motor_x, motor_velocity = 0, 0
                ##############################

                state = np.array([motor_x, theta, motor_velocity, angle_velocity])

                control, error = pd_controller(state)
                target_velocity = motor_velocity + control * dt

                
                #target velocity:
                #motor.updateAngle(target_velocity)
                ##target acceleration:
                #motor.updateAngle(control)

                print(f"Control Output: {target_velocity}, Timestamp: {timestamp}")
                if data_count < 2000:
                    data_buffer.append([timestamp, theta, motor_x, motor_velocity, angle_velocity, target_velocity, error])


            prev_time = timestamp
            prev_angle = theta
        time.sleep(0.1)  # delay

    with open('motor_data.csv', 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Timestamp", "Theta", "Motor_x", "Motor_velocity", "Angle_velocity", "Control_Output"])

        csv_writer.writerows(data_buffer)
        print("Data has been written to the CSV file.")

def pd_controller(current, target=np.array([0, 0, 0, 0])):
    # Define controller gains
          
    #[  14.92236214 2447.68628701   27.83794322   60.197189  ]
    Kp = np.array([[14.92236214, 2447.68628701, 27.83794322, 60.197189 ]])
    
    Kd = np.array([[0, 0, 0, 0]])
    
    Ki = np.array([[0, 0, 0, 0]])
    
    # Calculate the error and its derivative
    error = target - current
    
    # Ensure prev_error and integral are initialized
    if not hasattr(pd_controller, "prev_error"):
        pd_controller.prev_error = np.zeros_like(error)
    if not hasattr(pd_controller, "integral"):
        pd_controller.integral = np.zeros_like(error)
    if not hasattr(pd_controller, "dt"):
        pd_controller.dt = 0.1  # Default time step if not set
    
    derivative = (error - pd_controller.prev_error) / pd_controller.dt  # Derivative term
    
    # Update the previous error and integral
    pd_controller.prev_error = error
    pd_controller.integral += error * pd_controller.dt  # Integral term
    
    # Calculate the output
    output = np.dot(Kp, error) + np.dot(Kd, derivative) + np.dot(Ki, pd_controller.integral)
    return output, error


def run_onlySense_script():
    print("running onlysense")
    os.system("python onlySense.py")


if __name__ == "__main__":
    # Create the stop event
    stop_event = multiprocessing.Event()

    # Create pipes for communication
    camera_parent_pipe, camera_child_pipe = multiprocessing.Pipe()
    touch_sensor_parent_pipe, touch_sensor_child_pipe = multiprocessing.Pipe()

    # Create and start the processes
    p1 = multiprocessing.Process(target=camera_process, args=(camera_child_pipe, stop_event))
    p2 = multiprocessing.Process(target=touch_sensor_process, args=(touch_sensor_child_pipe, stop_event))
    p3 = multiprocessing.Process(target=motor_process, args=(camera_parent_pipe, touch_sensor_parent_pipe, stop_event))


    process = multiprocessing.Process(target=run_onlySense_script)
    process.start()

 

    p1.start()
    p2.start()
    p3.start()


    start_time = time.time()
    print("Press 'q' to stop or wait for 10 seconds: ")
    while True:
        user_input = None #input("")
        if user_input == 'q' or time.time() - start_time >= 20:
            stop_event.set()  
            print("Stop event triggered!")
            break 
        time.sleep(0.1) 
    
    print("1")
    exit()

 

