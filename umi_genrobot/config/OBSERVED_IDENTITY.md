# Observed DAS Finger Controller RV3 identity

Status: **MODEL CONFIRMED BY LABEL; USB MAPPING CONSISTENT**

## Physical model

The photographed label reads `DAS Finger Controller RV3`. This model uses the
one-camera Finger Controller SDK.

## Serial

```text
/dev/ttyUSB0
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
VID:PID = 1a86:7523 (CH340)
```

## Camera

```text
TSTC USB20 WEB CAMERA, VID:PID = 1bcf:2d4f
/dev/video12 = video capture
/dev/video13 = UVC metadata
```

The capture node advertises MJPEG 1600x1296 at 60/30 fps, matching the Finger
Controller manual. A single capture camera is expected for this model.

The correct Finger SDK opens the CH340 port at 921600 baud. A raw capture proved
that the device returns the expected `das\\r\\n...das\\r\\n` MCUID response.
The original three-second doctor timeout was increased to five seconds, after
which the strict SDK query passed. The MCUID value is intentionally not stored
in this file.

A one-frame MJPEG capture at 1600x1296 and 30 fps also passed without starting
the Finger motor or encoder loop.

## Existing devices that must remain separate

```text
Robotiq: /dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAAQM7QD-if00-port0
Homomorphic teleop: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBJKECV-if00-port0
```

Never identify these devices by transient `/dev/ttyUSBN` numbers.
