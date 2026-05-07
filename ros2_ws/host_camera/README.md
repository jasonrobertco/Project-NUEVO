# Pi Camera Host Setup

This folder sets up the Raspberry Pi camera on native Ubuntu and exposes it to
ROS 2 Docker as a normal V4L2 camera at `/dev/video10`.

## Quick Start

Run these commands from the repository root on the Raspberry Pi:

```bash
./ros2_ws/host_camera/install.sh
./ros2_ws/host_camera/check.sh
```

The check command captures test images here:

```text
/tmp/pi_camera_check/host_loopback.jpg
/tmp/pi_camera_check/docker_loopback.jpg
```

If Docker has not been built yet, the host camera parts can still pass. The
Docker capture needs the `ros2-runtime:latest` image:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml build
./ros2_ws/host_camera/check.sh
```

## Expected Success

The camera check should end with:

```text
Results: 9 passed, 0 failed
Images: /tmp/pi_camera_check
```

Verify the host image:

```bash
ls -lh /tmp/pi_camera_check/host_loopback.jpg
```

Copy it to another machine if needed:

```bash
scp 162TA@192.168.1.144:/tmp/pi_camera_check/host_loopback.jpg .
```

## Service Commands

Check whether the camera feed service is running:

```bash
systemctl is-active pi-camera-feed.service
```

Healthy output:

```text
active
```

Bad output:

```text
activating
inactive
failed
```

Show details:

```bash
sudo systemctl status pi-camera-feed.service --no-pager
sudo journalctl -u pi-camera-feed.service -n 80 --no-pager
```

Restart the service:

```bash
sudo systemctl restart pi-camera-feed.service
sleep 3
systemctl is-active pi-camera-feed.service
```

## Direct Host Capture

Capture one frame from the virtual camera:

```bash
ffmpeg -y \
  -f v4l2 \
  -input_format yuyv422 \
  -i /dev/video10 \
  -vframes 1 \
  /tmp/pi_host_camera.jpg
```

Verify it:

```bash
ls -lh /tmp/pi_host_camera.jpg
```

## Docker Capture

The check script uses a disposable Docker container so the camera test does not
depend on other robot hardware such as `/dev/rplidar`.

Manual Docker capture:

```bash
docker run --rm \
  --entrypoint bash \
  --device /dev/video10:/dev/video10 \
  --volume /tmp:/tmp \
  ros2-runtime:latest \
  -lc 'ffmpeg -y -f v4l2 -input_format yuyv422 -i /dev/video10 -vframes 1 /tmp/pi_docker_camera.jpg'
```

Verify it on the host:

```bash
ls -lh /tmp/pi_docker_camera.jpg
```

## Technical Details

The Pi camera is managed on the native Ubuntu host, not inside Docker.

The data path is:

```text
Pi CSI camera
  -> rpicam-vid
  -> ffmpeg
  -> v4l2loopback
  -> /dev/video10
  -> ROS 2 Docker container
```

`install.sh` installs the host dependencies:

- `rpicam-apps-lite`
- `libcamera-tools`
- `libcamera-ipa`
- `ffmpeg`
- `v4l-utils`
- `v4l2loopback-dkms`
- `v4l2loopback-utils`
- matching Raspberry Pi kernel headers when available

It also installs:

```text
/etc/default/pi-camera
/usr/local/bin/pi-camera-feed
/etc/systemd/system/pi-camera-feed.service
/etc/modules-load.d/pi-camera.conf
/etc/modprobe.d/pi-camera.conf
```

The service starts automatically at boot and restarts on failure. The restart
delay is intentionally moderate so a transient camera failure does not hammer
the Pi camera driver.

The default virtual camera is:

```text
/dev/video10
```

The default raw feed is:

```text
640x480 @ 15 Hz
```

The default label is:

```text
Pi Camera
```

Docker accesses `/dev/video10` through the RPi compose file. The compose file
uses `/dev:/dev` plus device cgroup rules so missing optional hardware does not
stop the container from starting.

## Troubleshooting

If `rpicam-hello --list-cameras` says no cameras are available, check:

```bash
rpicam-hello --list-cameras
dpkg -l | grep -E 'libcamera|rpicam|v4l2loopback|ffmpeg|v4l-utils'
```

`libcamera-ipa` must be installed. Without it, libcamera may detect the sensor
but fail to register a usable Raspberry Pi camera.

If the service is stuck in `activating`, inspect the journal:

```bash
sudo journalctl -u pi-camera-feed.service -n 80 --no-pager
```

If the log contains:

```text
Failed to start streaming: Operation already in progress
```

stop the service and reset the Pi camera kernel modules:

```bash
sudo systemctl stop pi-camera-feed.service
sudo modprobe -r imx219 rp1_cfe_downstream pisp_be
sudo modprobe pisp_be
sudo modprobe rp1_cfe_downstream
sudo modprobe imx219
sudo systemctl start pi-camera-feed.service
sleep 3
systemctl is-active pi-camera-feed.service
```

Then rerun:

```bash
./ros2_ws/host_camera/check.sh
```

If `/dev/video10` exists but is not a capture device, the feed service is not
successfully writing frames into the loopback device. Check:

```bash
v4l2-ctl --device=/dev/video10 --info
v4l2-ctl --device=/dev/video10 --list-formats-ext
sudo journalctl -u pi-camera-feed.service -n 80 --no-pager
```

When healthy, `/dev/video10` should advertise `Video Capture` and a usable
format such as `YUYV`.
