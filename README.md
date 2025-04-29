# Toytrain Locomotive Project

This project is designed to control a toy train locomotive using BLE (Bluetooth Low Energy) and motor driver functionalities. The locomotive's speed and direction can be controlled via BLE, and its battery level is monitored and updated in real-time.

## Features
- **BLE Control**: Control the locomotive's motor speed and direction using BLE characteristics.
- **Battery Monitoring**: Monitor and notify the battery level and voltage via BLE.
- **Motor Driver Integration**: Interface with the motor driver to control speed and direction.
- **State Machine**: Implements a state machine to manage locomotive states (Idle, Running, Error).

## How to Use
1. **Setup ESP-IDF**: Ensure you have the ESP-IDF environment set up. Follow the [ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
2. **Build and Flash**:
   ```bash
   idf.py build
   idf.py flash
   ```
3. **Monitor Logs**:
   ```bash
   idf.py monitor
   ```
   Use the logs to debug and observe the locomotive's behavior.

4. **Connect via BLE**:
   - Use a BLE debugging tool (e.g., nRF Connect) to connect to the locomotive.
   - Control the motor speed and direction by writing to the respective BLE characteristics.

## BLE Characteristics
- **Battery Service**:
  - **Battery Level**: UUID `0x2A19`
  - **Battery Voltage**: UUID `0x2B18`
- **Motor Service**:
  - **Motor Speed**: UUID `0xDD01`
  - **Motor Direction**: UUID `0xDD02`

## Project Folder Structure
```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── main.c                  # Main application logic
│   ├── ble_driver_srv.c        # BLE driver service implementation
│   ├── motor_driver.c          # Motor driver implementation
│   └── ble_driver_srv.h        # BLE driver service header
├── README.md                   # Project documentation
```

## Example Logs
Below is an example of the logs you might see during operation:
```
I (508) GATTS_DEMO: Advertising start successfully
I (36898) GATTS_DEMO: Connected Profile0, conn_id 0, remote 61:d3:e3:97:53:93
I (44618) GATTS_DEMO: Notifications enabled for char 0x2A19
I (52898) GATTS_DEMO: Characteristic write Profile1, handle 50
I (21131) Loc Task: Motor Speed: 68, Motor Direction: 1
```

## Notes
- Ensure the BLE client supports the required characteristics and descriptors.
- Modify the connection parameters in `ble_driver_srv.c` if needed for compatibility with your BLE client.

For more details, refer to the [ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/).
