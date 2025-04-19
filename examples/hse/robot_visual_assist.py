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

import argparse
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
import os
import json
import ssl
from queue import Queue
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import VUI_COLOR
from aiortc import VideoStreamTrack, RTCPeerConnection, RTCRtpSender, RTCSessionDescription
from aiohttp import web
from av import VideoFrame

class RealSenseVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        frame_queue = Queue()
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        # Get device product line of intel realsense camera for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        #Check for right hardware
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The programm requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # Start streaming
        self.pipeline.start(config)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        # Align depth frame to color frame
        align_to = rs.stream.color
        align = rs.align(align_to)
        aligned_frames = align.process(frames)

        # Extract aligned depth and color frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

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
        

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            img = np.hstack((resized_color_image, depth_colormap, overlay))
            self.frame_queue.put(img)
        else:
            img = np.hstack((color_image, depth_colormap, overlay))
            self.frame_queue.put(img)

        # Frame konvertieren
        color_image = np.asanyarray(color_frame.get_data())
        frame = VideoFrame.from_ndarray(color_image, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        return frame

def create_local_tracks(play_from, decode):
    return None, RealSenseVideoTrack()

def force_codec(pc, sender, forced_codec):
    kind = forced_codec.split("/")[0]
    codecs = RTCRtpSender.getCapabilities(kind).codecs
    transceiver = next(t for t in pc.getTransceivers() if t.sender == sender)
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType == forced_codec]
    )


async def index(request):
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)


async def javascript(request):
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # open media source
    video = RealSenseVideoTrack()
    audio = None

    if video:
        video_sender = pc.addTrack(video)
        if args.video_codec:
            force_codec(pc, video_sender, args.video_codec)
        elif args.play_without_decoding:
            raise Exception("You must specify the video codec using --video-codec")

    await pc.setRemoteDescription(offer)

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),)


pcs = set()


async def on_shutdown(app):
    # close peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

def run_web_server(app):
    web.run_app(app, host=args.host, port=args.port)

# Constants 
IP_ADDRESS = "192.168.4.202"
ROOT = os.path.dirname(__file__)

# Enable logging for debugging
logging.basicConfig(level=logging.WARN)


dog = Dog(IP_ADDRESS)

def main():
    global dog
    global args

    # Choose a connection method
    dog.conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=dog.ip_address)

    #Set dog to manual mode
    dog.set_mode("MODE_MANUAL")

    def output_img():
        RealSenseVideoTrack.recv()


    async def run_async():
        # Connect to dog + StandUp
        await dog.conn.connect()
        logging.info("Performing 'StandUp' movement...")
        dog.balance_stand()

        # Starte WebRTC/Webserver
        app = web.Application()
        app.on_shutdown.append(on_shutdown)
        app.router.add_get("/", index)
        app.router.add_get("/client.js", javascript)
        app.router.add_post("/offer", offer)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, args.host, args.port)
        await site.start()
        logging.info(f"Web server running at http://{args.host}:{args.port}")

        # Blockiere unendlich, damit der Server läuft
        await asyncio.Event().wait()


    #Parse Arguments
    parser = argparse.ArgumentParser(description="RealSense WebRTC Stream with DogBot")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--video-codec", help="Force a specific video codec (e.g. video/H264)")
    args = parser.parse_args()
    #Start WebRTC-Server
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)

    # Create a new event loop for the asyncio code
    loop = asyncio.new_event_loop()

    # Start the asyncio event loop in a separate thread
    asyncio_thread = threading.Thread(target=run_async, args=(loop,))
    asyncio_thread.start()

    #Start the realsense stream in a seperate thread
    realsense_thread = threading.Thread(target=output_img)
    realsense_thread.start()

    #Start the WebRTC-Server in a seperate thread
    web_thread = threading.Thread(target=run_web_server, args=(app,))
    web_thread.start()

    asyncio.run(run_async())

    try:
        while True:
            if not RealSenseVideoTrack.frame_queue.empty():
                img = RealSenseVideoTrack.frame_queue.get()

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
        RealSenseVideoTrack.pipeline.stop()
        # Stop the asyncio event loop
        loop.call_soon_threadsafe(loop.stop)
        asyncio_thread.join()
        realsense_thread.join()
        web_thread.join()

if __name__ == "__main__":
    main()

