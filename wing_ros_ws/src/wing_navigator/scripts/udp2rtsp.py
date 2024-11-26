#!/usr/bin/env python3

# This code receives UDP stream and outputs RTSP stream.
# Used for getting gazebo camera stream in our GCS system.


import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject


class RestreamingRTSPServer(GstRtspServer.RTSPMediaFactory):
    def __init__(self, port):
        super().__init__()
        self.set_launch((
            f"udpsrc port={port} buffer-size=524288 caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" ! "
            "rtph264depay ! rtph264pay name=pay0 pt=96"
        ))
        self.set_shared(True)


class Server:
    def __init__(self, port, stream_name):
        Gst.init(None)

        self.server = GstRtspServer.RTSPServer()
        factory = RestreamingRTSPServer(port)
        factory.set_shared(True)

        self.server.get_mount_points().add_factory(stream_name, factory)
        self.server.attach(None)
        print(f"RTSP stream ready at rtsp://127.0.0.1:8554/{stream_name}")


if __name__ == "__main__":
    server = Server(port=8500, stream_name="test")
    loop = GObject.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
