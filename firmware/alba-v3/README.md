# ALBA Firmware v3

Final ESP32 firmware used for the ALBA benchtop sensing and actuation system
reported in the MSc thesis:

**Development of an Integrated Sensing and Control Platform for a Soft Robotic
Bladder Assist System**

## Functions

The firmware provides:

- stepper-motor command generation;
- quadrature encoder acquisition;
- flow-sensor pulse acquisition and flow estimation;
- solenoid-valve control;
- auxiliary force-sensor acquisition;
- voiding-state supervision;
- USB serial telemetry and host-side data logging.

## Key configuration

| Item | Final configuration |
|---|---|
| Controller | ESP32-DevKitC-32E |
| Serial baud rate | 115200 |
| Stepper microstepping | 1/16 |
| Motor command resolution | 3200 STEP/rev |
| Encoder resolution | 1600 counts/rev |
| Flow calibration | 1380 pulses/L |
| Flow signal | GPIO27 |
| Valve control | GPIO32 |
| Force input | GPIO34 |

The firmware used for the reported experiments does not implement the
closed-loop PI controller evaluated later in simulation.

## Repository structure

```text
alba-v3/
├── platformio.ini
├── README.md
├── include/
├── src/
│   └── main.cpp
└── scripts/
    ├── log_serial.py
    ├── log_fixed_id.py
    └── requirements.txt
```

`src/` and `include/` contain the embedded firmware source.

`scripts/log_serial.py` provides general serial logging and manual command
support.

`scripts/log_fixed_id.py` provides data acquisition for the fixed-load
experiments.

## Build

The firmware is provided as a PlatformIO project.

Open the `alba-v3` directory in PlatformIO and build/upload using the
`esp32dev` environment defined in `platformio.ini`.

Python dependencies for the host-side logging scripts are listed in:

```text
scripts/requirements.txt
```