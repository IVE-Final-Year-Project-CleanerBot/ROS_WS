import io
import logging
import socketserver
from http import server
from threading import Condition
from picamera2 import Picamera2
from picamera2.outputs import FileOutput

class PassthroughEncoder:
    def __init__(self):
        self._format = "MJPEG"
        self._output = None

    @property
    def format(self):
        return self._format

    @format.setter
    def format(self, fmt):
        self._format = fmt

    def start(self, quality=None, **kwargs):
        if "output" in kwargs:
            self._output = kwargs["output"]

    def encode(self, name, req):
        buf = req.make_buffer(name)
        if self._output is not None:
            self._output.write(buf)

    def stop(self):
        pass

PAGE = """\
<html>
<head>
<title>Picamera2 MJPEG Streaming Demo</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="1280" height="720" />
</body>
</html>
"""

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning('Removed streaming client %s: %s', self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

available_cameras = Picamera2.global_camera_info()
for cam in range(len(available_cameras)):
    print(f'---- Details for camera {cam}')
    tmp_cam = Picamera2(cam)
    print(f'Camera: {available_cameras[cam]["Model"]}')
    print(tmp_cam.sensor_modes)
    print(tmp_cam.sensor_resolution)
    print(tmp_cam.create_video_configuration())
    tmp_cam.close()
    print('\n\n\n')

camera_index = 0
if camera_index < len(available_cameras):
    picam2 = Picamera2(camera_index)

    config = picam2.create_preview_configuration(main={"format": "MJPEG", "size": (1280, 720)})
    picam2.configure(config)
    print(f'Configured camera with: {picam2.camera_config}')

    _original_start_encoder = picam2.start_encoder
    def patched_start_encoder(encoder, output, pts=None, quality=None, name="main"):
        encoder._output = output
        try:
            return _original_start_encoder(encoder, output, pts, quality, name)
        except KeyError as e:
            if e.args[0] == "FrameDurationLimits":
                class DummyDuration:
                    def __init__(self, min_val):
                        self.min = min_val
                dummy = DummyDuration(33333333)
                picam2.camera_ctrl_info["FrameDurationLimits"] = [None, dummy]
                return _original_start_encoder(encoder, output, pts, quality, name)
            else:
                raise
    picam2.start_encoder = patched_start_encoder

    output = StreamingOutput()
    picam2.start_recording(PassthroughEncoder(), output)

    try:
        address = ('', 8081)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    finally:
        picam2.stop_recording()
else:
    print(f"Error: Camera index {camera_index} is out of range. Available cameras: {len(available_cameras)}")