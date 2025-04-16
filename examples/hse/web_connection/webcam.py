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

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from av import VideoFrame

ROOT = os.path.dirname(__file__)        # Speicherort dieses Python-Files
pcs = set()                             # zur Speicherung aller aktiven WebRTC-Verbindungen

class OpenCVCameraTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):     # Konstruktor
        super().__init__()
        self.cap = cv2.VideoCapture(0)      # öffnet die Kamera mit Index 0 (wenn mehrere Kameras, dann Index herausfinden)
        
        # Auflösung setzen
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1440)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 810)

        self.counter = 0

    async def recv(self):
        ret, frame = self.cap.read()        # Frames lesen
        if not ret:
            raise RuntimeError("Kamerabild konnte nicht gelesen werden")        # wenn kein Frame gesendet
        
        # Umwandlung für WebRTC
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        new_frame = VideoFrame.from_ndarray(frame, format="rgb24")

        # Setze den PTS für jedes Frame
        new_frame.pts = self.counter
        new_frame.time_base = fractions.Fraction(1, 30)  # Verwende die maximale Framerate
        self.counter += 1

        return new_frame


def create_local_tracks():
    return None, OpenCVCameraTrack()        # gibt eine Videoquelle (Kamera) zurück und kein Audio (None)

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

    audio, video = create_local_tracks()

    if video:
        pc.addTrack(video)          # hängt das Video an die WebRTC Verbindung 

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
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

