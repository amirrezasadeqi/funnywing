# Converting Gazebo camera UDP video stream to RTSP for using in simulation tests

For getting the gazebo camera video stream in the GCS to achieve full system test suit,
you can use MediaMtx as below in the computer used for running GCS:

1. run mediamtx with default configurations:

```bash
cd <mediamtx directory>
./mediamtx    
```

I think, this will start a RTSP stream server that we can stream using ffmpeg into it
and get a stream using the stream URLs provided in the yaml config file.

2. run below FFmpeg command to convert UDP to RTSP and send it to the mediamtx:

```bash
ffmpeg -fflags nobuffer -flags low_delay -protocol_whitelist file,udp,rtp -i <.sdp config file> -c:v copy -f rtsp rtsp://localhost:8554/test
```

where the content of the SDP file should be as below:

```text
v=0
o=- 0 0 IN IP4 127.0.0.1
s=No Name
c=IN IP4 127.0.0.1
t=0 0
m=video 8500 RTP/AVP 96
a=rtpmap:96 H264/90000
```

now if you run GCS application, you can get the gazebo camera in the camera monitor panel.

End!