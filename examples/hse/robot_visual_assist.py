"""
Robot Control Script for WebRTC and Marker Detection
Based on: https://github.com/legion1581/go2_webrtc_connect

Author: Marco Dittmann  
Date: 4.12.24  

Description:  
This script uses the Dog class to control the movement of a robotic dog and utilizes its camera feed to search for and detect ArUco markers.
The robot's behavior dynamically adapts based on marker detection, allowing it to perform various actions, such as moving, sitting, and standing.  
The script integrates OpenCV for video processing, asyncio for asynchronous communication, and WebRTC for establishing a connection with the robot.
It processes video frames in real-time to detect markers and triggers appropriate actions based on the detection state.  

Features:  
- Real-time video streaming and marker detection with OpenCV and ArUco.  
- Robot movement control (e.g., move, sit, stand) using the Dog class.  
- Asynchronous communication with the robot via WebRTC.  
- Integration with threading for smooth operation.  

Note:  
- Ensure the robot's IP address is correctly set in the IP_ADDRESS constant.  
- Install all dependencies, including OpenCV, asyncio, and aiortc.  
"""

import asyncio
import pyrealsense2 as rs
import cv2
import numpy as np
from dog import Dog, ControlMode

# Create an OpenCV window and display a blank image
height, width = 720, 1280  # Adjust the size as needed
img = np.zeros((height, width, 3), dtype=np.uint8)
cv2.imshow('Video', img)
cv2.waitKey(1)  # Ensure the window is created

import logging
import threading
import time
from os import path
from queue import Queue
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import VUI_COLOR
from aiortc import MediaStreamTrack

# Constants 
IP_ADDRESS = "192.168.4.202"

# Enable logging for debugging
logging.basicConfig(level=logging.WARN)


dog = Dog(IP_ADDRESS)

def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

def main():
    global dog
    frame_queue = Queue()

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Configure accl and gyro streams
    pipeline_accl_gyro = rs.pipeline()
    config_acc_gyro = rs.config()

    # Get device product line of intel realsense camera for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    #Check for right hardware
    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    #Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  #depth
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) #rgb

    # Start streaming
    pipeline.start(config)

    # Choose a connection method
    dog.conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=dog.ip_address)

    #Set dog to manual mode
    dog.set_mode("MODE_MANUAL")

    def output_img():
        capture_realsense_frames()


    def capture_realsense_frames():
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()

            # Align depth frame to color frame
            align_to = rs.stream.color
            align = rs.align(align_to)
            aligned_frames = align.process(frames)

            # Extract aligned depth and color frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_AUTUMN)

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            # Generate pointcloud
            mapped_frame, color_source = color_frame, color_image
            pc = rs.pointcloud()
            pc.map_to(mapped_frame)
            points = pc.calculate(depth_frame)
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

            # 3D-obstacle mask
            x_values = verts[:, 0]  # x = width
            z_values = verts[:, 2]  # z = depth
            y_values = verts[:, 1]  # y = height refering to axis through cameralens
            depth_threshold = 2.0  # sets distance in meters to detect obstacle

            obstacle_mask_3d = (z_values > 0) & (z_values < depth_threshold) & (y_values < 0.3) & (y_values > -0.2) & (x_values > -0.3) & (x_values < 0.3)
            
            # Resize mask to match colored image
            obstacle_mask_img = obstacle_mask_3d.reshape(depth_image.shape)
            color_mask = np.zeros_like(color_image)
            color_mask[obstacle_mask_img] = depth_colormap[obstacle_mask_img]
            overlay = cv2.addWeighted(color_image, 0.7, color_mask, 0.3, 0)

            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # Define your original fixed 3D lines (unchanged)
            line_3d_left = [[-0.3, 0.3, 0.1], [-0.3, 0.3, 2.0]]
            line_3d_right = [[0.3, 0.3, 0.1], [0.3, 0.3, 2.0]]

            # Gradient colors (BGR)
            start_color = (0, 0, 255)    # Red
            end_color = (0, 255, 255)    # Yellow

            # Number of segments in the gradient
            num_segments = 100

            def lerp_color(c1, c2, t):
                return tuple(int(c1[i] + (c2[i] - c1[i]) * t) for i in range(3))

            def draw_gradient_line(line_3d):
                for i in range(num_segments):
                    z0 = 0.1 + (i / num_segments) * (2.0 - 0.1)
                    z1 = 0.1 + ((i + 1) / num_segments) * (2.0 - 0.1)

                    pt0_3d = [line_3d[0][0], line_3d[0][1], z0]
                    pt1_3d = [line_3d[0][0], line_3d[0][1], z1]

                    pt0_2d = rs.rs2_project_point_to_pixel(color_intrinsics, pt0_3d)
                    pt1_2d = rs.rs2_project_point_to_pixel(color_intrinsics, pt1_3d)

                    t = i / num_segments
                    color = lerp_color(start_color, end_color, t)

                    if all(0 <= pt0_2d[j] < overlay.shape[1 - j] for j in [0, 1]) and \
                    all(0 <= pt1_2d[j] < overlay.shape[1 - j] for j in [0, 1]):
                        pt0 = tuple(map(int, pt0_2d))
                        pt1 = tuple(map(int, pt1_2d))
                        cv2.line(overlay, pt0, pt1, color, 2, cv2.LINE_AA)

            # Draw gradient lines for both sides
            draw_gradient_line(line_3d_left)
            draw_gradient_line(line_3d_right)

            # Add distance markers at 1m and 2m
            for z_mark in [1.0, 2.0]:
                pt_left = rs.rs2_project_point_to_pixel(color_intrinsics, [-0.3, 0.3, z_mark])
                pt_right = rs.rs2_project_point_to_pixel(color_intrinsics, [0.3, 0.3, z_mark])
                pt_left = tuple(map(int, pt_left))
                pt_right = tuple(map(int, pt_right))

                if all(0 <= pt_left[j] < overlay.shape[1 - j] for j in [0, 1]) and \
                all(0 <= pt_right[j] < overlay.shape[1 - j] for j in [0, 1]):
                    # Draw horizontal marker line
                    cv2.line(overlay, pt_left, pt_right, (255, 255, 255), 1, cv2.LINE_AA)
                    # Draw label just above the left point
                    label = f"{int(z_mark)}m"
                    text_pos = (pt_left[0] - 5, pt_left[1] - 10)  # Slightly to the left and above
                    cv2.putText(overlay, label, text_pos, cv2.FONT_HERSHEY_PLAIN, 1.2, (255, 255, 255), 1, cv2.LINE_AA)

            # Write status in stream
            overlay = cv2.putText(overlay, 'state:', (10, 40), cv2.FONT_HERSHEY_DUPLEX, 1, (200, 200, 200), 1, cv2.LINE_AA)
            overlay = cv2.putText(overlay, 'ONLINE', (120, 40), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                

            # If depth and color resolutions are different, resize color image to match depth image for display
            if depth_colormap_dim != color_colormap_dim:
                resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                img = np.hstack((resized_color_image, depth_colormap, overlay))
                frame_queue.put(img)
            else:
                img = np.hstack((color_image, depth_colormap, overlay))
                frame_queue.put(img)


    def run_asyncio_loop(loop):
        asyncio.set_event_loop(loop)
        async def setup():
            try:
                # Connect to the device
                await dog.conn.connect()

                logging.info("Performing 'StandUp' movement...")
                dog.balance_stand()
            except Exception as e:
                logging.error(f"Error in WebRTC connection: {e}")

        # Run the setup coroutine and then start the event loop
        loop.run_until_complete(setup())
        loop.run_forever()

    # Create a new event loop for the asyncio code
    loop = asyncio.new_event_loop()

    # Start the asyncio event loop in a separate thread
    asyncio_thread = threading.Thread(target=run_asyncio_loop, args=(loop,))
    asyncio_thread.start()

    #Start the realsense stream in a seperate thread
    realsense_thread = threading.Thread(target=output_img)
    realsense_thread.start()

    try:
        while True:
            if not frame_queue.empty():
                img = frame_queue.get()

                # Display the frame 
                cv2.imshow('Video', img)
                key_input = cv2.waitKey(1)

                if dog.mode is ControlMode.MODE_MANUAL.value:
                    dog.process_key(key_input, loop)

            else:
                # Sleep briefly to prevent high CPU usage
                time.sleep(0.01)
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        # Stop the asyncio event loop
        loop.call_soon_threadsafe(loop.stop)
        asyncio_thread.join()

if __name__ == "__main__":
    main()

