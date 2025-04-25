##################################################################################
#
# Code aus dem Repository https://github.com/aiortc/aiortc/tree/main/examples/webcam kombiniert mit OpenCV
#
##################################################################################

import argparse
import asyncio
import json
import logging
import os
import ssl
import time
import cv2
import fractions
import pyrealsense2 as rs
import numpy as np
import queue
import threading

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
from queue import Queue
ROOT = os.path.dirname(__file__)        # Speicherort dieses Python-Files
pcs = set()                             # zur Speicherung aller aktiven WebRTC-Verbindungen

# Frame-Queue initialisieren
frame_queue = Queue(maxsize=1)

# RealSense Setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

def capture_frames():
    # Align depth frame to color frame
    align_to = rs.stream.color
    align = rs.align(align_to)

    while True:
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)
        
        depth_frame = aligned_frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())

        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
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
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_AUTUMN)

        # Resize mask to match colored image
        obstacle_mask_img = obstacle_mask_3d.reshape(depth_image.shape)
        color_mask = np.zeros_like(color_image)
        color_mask[obstacle_mask_img] = depth_colormap[obstacle_mask_img]
        overlay = cv2.addWeighted(color_image, 0.7, color_mask, 0.3, 0)

        # Umwandlung für WebRTC
        img = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)

        # Bild in die Queue legen
        frame_queue.put(img)


capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

class RealSenseVideoStreamTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self):     # Konstruktor
        super().__init__()    
        self.prev_img = np.zeros((480, 640, 3), dtype=np.uint8)

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        if not frame_queue.empty():
            img = frame_queue.get()
            self.prev_img=img 
        else:
            img = self.prev_img
        
        frame = VideoFrame.from_ndarray(img, format="rgb24")

        frame.pts = pts
        frame.time_base = time_base
        
        return frame

relay = MediaRelay()
video = RealSenseVideoStreamTrack()

async def index(request):
    with open(os.path.join(ROOT, "index.html"), "r") as f:
        return web.Response(content_type="text/html", text=f.read())        # sendet dem Browser die HTML-Datei

async def javascript(request):
    with open(os.path.join(ROOT, "client.js"), "r") as f:
        return web.Response(content_type="application/javascript", text=f.read())   # sendet dem Browser die JavaScript-Datei

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])   # liest Daten vom Browser 
    
    pc = RTCPeerConnection()
    pcs.add(pc)                     # erstellt WebRTC-Verbindung

    @pc.on("connectionstatechange")             # wenn sich der Verbindungsstatus ändert
    async def on_connectionstatechange():
        print("Connection state is %s" % pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)                     # wenn Verbindung fehlschlägt, dann trennen und aus pcs-Liste entfernen

    pc.addTrack(relay.subscribe(video))

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response(
        {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
    )

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC OpenCV Kamera-Streamer")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP Server Host")
    parser.add_argument("--port", type=int, default=8080, help="HTTP Server Port")
    parser.add_argument("--cert-file", help="SSL-Zertifikat (optional)")
    parser.add_argument("--key-file", help="SSL-Key (optional)")
    parser.add_argument("--verbose", "-v", action="count")
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    ssl_context = None
    if args.cert_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)

    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)

    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context)