# Hardware Wiring Guide
## Power System
- **WARNING:** Do not power Pi Zero 2 W from F405 5V pad. Use an external 5V Step-down BEC.

## Video Pipeline (IMX500 to VTX)
1. IMX500 connects to Pi Zero 2 W via CSI-2 cable.
2. Pi Zero 2 W "TV Out" pad connects to F405 "CAM" or "VIN" pad.
3. F405 overlays OSD and sends video to Foxeer 4.9G VTX.
