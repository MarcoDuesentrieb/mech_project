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
from threading import Thread, Lock
import pygame
from dog import Dog, ControlMode

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
from queue import Queue
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import VUI_COLOR

from datetime import datetime, timedelta



# Frame-Queue initialisieren
frame_queue = queue.LifoQueue(maxsize=1)
queue_mutex = Lock()

depth_frame = None
color_frame = None
depth_image = None
color_image = None
last_cb_time = datetime.now()
last_rcv_time = datetime.now()

avoidance_offset = 0.0  # Positive Werte -> Hindernis links -> nach rechts ausweichen

forward_slowdown = 0.0  # 0.0 = keine Verlangsamung, 1.0 = voller Stopp


def process_frame(depth_frame, color_frame, depth_image, color_image, frame_queue):
        
        if depth_frame is None:
            # print("Empty image provided")
            return
        
        # start_time = datetime.now()
        
        # Generate pointcloud
        pcl_processing = True
        overlay = None
        if pcl_processing:
            mapped_frame, color_source = color_frame, color_image
            pcl = rs.pointcloud()
            pcl.map_to(mapped_frame)


            points = pcl.calculate(depth_frame)
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

            # 3D-obstacle mask
            x_values = verts[:, 0]  # x = width
            z_values = verts[:, 2]  # z = depth
            y_values = verts[:, 1]  # y = height refering to axis through cameralens
            depth_threshold = 2.0  # sets distance in meters to detect obstacle

            obstacle_mask_3d = (z_values > 0) & (z_values < depth_threshold) & (y_values < 0.43) & (y_values > -0.2) & (x_values > -0.3) & (x_values < 0.3)
        
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.1), cv2.COLORMAP_AUTUMN)

            # Resize mask to match colored image
            obstacle_mask_img = obstacle_mask_3d.reshape(depth_image.shape)
            color_mask = np.zeros_like(color_image)
            color_mask[obstacle_mask_img] = depth_colormap[obstacle_mask_img]
            overlay = cv2.addWeighted(color_image, 0.7, color_mask, 0.3, 0)

                    # Neue Logik zur Richtungsbestimmung
            global avoidance_offset

            # Aufteilen in linke/rechte Bildhälfte
            mask = obstacle_mask_3d.reshape(depth_image.shape)
            left_mask = mask[:, :int(mask.shape[1] * 0.4)]
            right_mask = mask[:, int(mask.shape[1] * 0.6):]

            # Tiefenwerte für linke/rechte Seite extrahieren
            left_depth = depth_image[:, :int(mask.shape[1] * 0.4)][left_mask]
            right_depth = depth_image[:, int(mask.shape[1] * 0.6):][right_mask]

            left_count = np.count_nonzero(left_mask)
            right_count = np.count_nonzero(right_mask)

            # Mittelwerte berechnen, wenn Punkte vorhanden
            left_mean = np.mean(left_depth) if left_count > 0 else np.inf
            right_mean = np.mean(right_depth) if right_count > 0 else np.inf

            # Debug-Ausgabe (optional)
            # print(f"[Avoid] Links: {left_count} @ {left_mean:.2f} | Rechts: {right_count} @ {right_mean:.2f}")

            # Entscheidungslogik:
            threshold_count = 300     # Mindestanzahl für "Hindernis"
            threshold_depth = 1500    # in mm, also z.B. 1.5 Meter

            # Beide Seiten Hindernisse → Vergleich der Tiefe
            if left_count > threshold_count and right_count > threshold_count:
                if left_mean < right_mean - 200:  # links deutlich näher
                    avoidance_offset = -0.5  # weiche nach rechts aus
                elif right_mean < left_mean - 200:
                    avoidance_offset = 0.5   # weiche nach links aus
                else:
                    avoidance_offset = 0.0   # ungefähr gleich → kein Eingriff

            # Nur links Hindernis
            elif left_count > threshold_count and left_mean < threshold_depth:
                avoidance_offset = -0.5

            # Nur rechts Hindernis
            elif right_count > threshold_count and right_mean < threshold_depth:
                avoidance_offset = 0.5

            # Kein relevantes Hindernis
            else:
                avoidance_offset = 0.0
            global forward_slowdown

            # Zentrum definieren (20 % breite Bildmitte)
            center_mask = mask[:, int(mask.shape[1] * 0.4):int(mask.shape[1] * 0.6)]
            center_depth = depth_image[:, int(mask.shape[1] * 0.4):int(mask.shape[1] * 0.6)][center_mask]

            center_count = np.count_nonzero(center_mask)
            center_mean = np.mean(center_depth) if center_count > 0 else np.inf

            # Werte in mm
            threshold_stop = 700    # < 0.7m → Stop
            threshold_slow = 1500   # < 1.5m → Langsamer

            if center_count > 200:
                if center_mean < threshold_stop:
                    forward_slowdown = 1.0  # harter Stopp
                elif center_mean < threshold_slow:
                    # Skaliere lineare Bremse (z. B. bei 1.2m → slowdown = 0.5)
                    forward_slowdown = 1.0 - (center_mean - threshold_stop) / (threshold_slow - threshold_stop)
                    forward_slowdown = max(0.0, min(1.0, forward_slowdown))
                else:
                    forward_slowdown = 0.0
            else:
                forward_slowdown = 0.0

        # Umwandlung für WebRTC
        img = None
        if overlay is not None:
            img = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
        else: 
            img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # end_time = datetime.now()
        # deltaT = end_time - start_time
        # deltaT_ms = round(deltaT.total_seconds()*1000,2)
        # print(f"compute time processing {deltaT_ms} ms")

        # Bild in die Queue legen
        try:
            if frame_queue.full():
                frame_queue.get()

            frame_queue.put_nowait(img)
        except queue.Full:
            print("frame queue cleaned")
        
        #frame_queue.put(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))


def frame_callback(frame):
    
    global depth_frame
    global color_frame
    global depth_image
    global color_image
    global ts
    global last_cb_time


    # start_time = datetime.now()
    # delta = (start_time - last_cb_time).total_seconds()
    # print("Callback delta: " + str(delta))
    # last_cb_time =  start_time
    # print("new frame")
    
    if not frame.is_frameset():
        print(f"Recieved an invalid datatype: {frame}")
        pass

    
    frameset = frame.as_frameset()

    align_to = rs.stream.color
    align = rs.align(align_to)
    aligned_frames = align.process(frameset)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    ts = frameset.get_timestamp()

    # print("frames written")
    # end_time = datetime.now()
    # deltaT = end_time - start_time
    # deltaT_ms = round(deltaT.total_seconds()*1000,2)
    # print(f"compution time callback {deltaT_ms} ms")

    #process_frame(depth_frame,color_frame,depth_image,color_image, frame_queue)


pcs = set()                             # zur Speicherung aller aktiven WebRTC-Verbindungen

# Constants 
IP_ADDRESS = "192.168.4.202"
ROOT = os.path.dirname(__file__)        # Speicherort dieses Python-Files


# RealSense Setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config,frame_callback)

# Pygame und Joystick initialisieren
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
# Pygame beenden, wenn kein Controller verbunden ist
if pygame.joystick.get_count() == 0:
    print("Kein Controller gefunden.")
    pygame.quit()

def capture_frames():
    # Align depth frame to color frame
    align_to = rs.stream.color
    align = rs.align(align_to)


    while True:
        
        depth_frame_l = None
        color_frame_l = None
        depth_image_l = None
        color_image_l = None

        with queue_mutex:
            depth_frame_l = depth_frame
            color_frame_l = color_frame
            depth_image_l = depth_image
            color_image_l = color_image

        process_frame(depth_frame_l,color_frame_l,depth_image_l,color_image_l, frame_queue)
        

capture_thread = Thread(target=capture_frames, daemon=True)
capture_thread.start()

class RealSenseVideoStreamTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self):     # Konstruktor
        super().__init__()    
        self.prev_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    async def recv(self):
        global last_rcv_time
        start_time = datetime.now()
        delta = (start_time - last_rcv_time).total_seconds()*1000  
        last_rcv_time = start_time

        pts, time_base = await self.next_timestamp()
    
        try:
            self.img = frame_queue.get_nowait()
            self.prev_img = self.img 
        except queue.Empty:
            # self.img = self.prev_img
            # self.img = np.zeros((480, 640, 3), dtype=np.uint8)
            # time.sleep(0.00001)
            print("Queue empty!")
            pass
        
        frame = VideoFrame.from_ndarray(self.img, format="rgb24")

        frame.pts = pts
        frame.time_base = time_base
        
        # end_time = datetime.now()
        # deltaT = end_time - start_time
        # deltaT_ms = round(deltaT.total_seconds()*1000,2)
        
        # print("---")
        # print("recv: " + str(delta) + " ms")
        # print("---")

        return frame

video = RealSenseVideoStreamTrack()
relay = MediaRelay()

async def css_handler(request):
    return web.FileResponse('style.css', headers={"Content-Type": "text/css"})

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

    # pc.addTrack(relay.subscribe(video))
    pc.addTrack(video)

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

dog = Dog(IP_ADDRESS)

# Choose a connection method
dog.conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=dog.ip_address)

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
asyncio_thread = Thread(target=run_asyncio_loop, args=(loop,), daemon=True)
asyncio_thread.start()


def read_controller():
    specialMoves_prev = [0, 0, 0, 0]
    dog.set_mode("MODE_MANUAL")

    try:
        while True:
            key_input = cv2.waitKey(1)

            pygame.event.pump()  # Controller-Events aktualisieren
            
            # xyMove = [-round(joystick.get_axis(0), 2), -round(joystick.get_axis(1), 2)]         # linker Stick x und y Achse für Bewegung

            # mit Ausweichlogik:
            raw_x = -round(joystick.get_axis(0), 2)
            raw_y = -round(joystick.get_axis(1), 2)

            zRot = -round(joystick.get_axis(3), 2)                                           # rechter Stick x Achse für Rotation
            
            if abs(raw_x) > 0.05 or abs(raw_y) > 0.05 or abs(zRot) > 0.05:
                x_with_avoid = raw_x + avoidance_offset
                x_with_avoid = max(-1.0, min(1.0, x_with_avoid))  # Begrenzen
            else:
                x_with_avoid = raw_x  # Kein Eingriff

            #xyMove = [x_with_avoid, raw_y]

            #Jetzt mit Bremse:
            adjusted_y = raw_y * (1.0 - forward_slowdown)
            xyMove = [x_with_avoid, adjusted_y]

            specialMoves = [joystick.get_button(0),                                             
                            joystick.get_button(1),                                             
                            joystick.get_button(2),                                             
                            joystick.get_button(3)]                         # [X, Kreis, Dreieck, Viereck]                              


            if dog.mode is ControlMode.MODE_MANUAL.value:
                dog.process_key(key_input, loop)
                dog.process_controller(xyMove, zRot, specialMoves, specialMoves_prev, loop)    # Funktionsaufruf
            
            specialMoves_prev = specialMoves

            # Sleep briefly to prevent high CPU usage
            time.sleep(0.01)
    finally:
        # Stop the asyncio event loop
        loop.call_soon_threadsafe(loop.stop)
        asyncio_thread.join()

control_thread = Thread(target=read_controller, daemon = True)
control_thread.start()

def main():
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
    app.router.add_static('/', path='.', name='static')


    web.run_app(app, host=args.host, port=args.port, ssl_context=ssl_context)

if __name__ == "__main__":
    main()