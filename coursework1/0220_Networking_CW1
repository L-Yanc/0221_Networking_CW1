# COMP0221 Networking Coursework - ESP32 Drone Flocking System

## Project Overview

This is an ESP32-based implementation of a distributed drone flocking algorithm with LoRa communication, MQTT telemetry publishing, and security attack simulation modes. The system implements Reynolds flocking rules (separation, cohesion, alignment) with neighbor discovery via LoRa radio and visualization through a Python-based MQTT dashboard.

**Course:** COMP0221 Networking - UCL  
**Platform:** ESP32 with SX1276 LoRa Radio  
**Wireless:** LoRa (868.1 MHz), WiFi (for MQTT/SNTP)  
**Visualization:** HiveMQ MQTT Broker + Python Dashboard

---

## Hardware Requirements

- **ESP32 Development Board** (tested on ESP32-DEVKIT-V1)
- **SX1276 LoRa Radio Module** connected via SPI
  - SCK: GPIO5
  - MISO: GPIO19
  - MOSI: GPIO27
  - CS: GPIO18
  - DIO0: GPIO26
  - RESET: GPIO14
- **WiFi/Internet Connection** (for SNTP time sync and MQTT)
- **Optional:** SD Card (for logging to `/sdcard/app_log.txt`)

---

## Project Structure

```
coursework1/
├── main/
│   ├── include/
│   │   ├── config.hpp           # Global configuration (frequencies, weights, CMAC key)
│   │   ├── comms.hpp            # LoRa communication, neighbor table, packet structure
│   │   ├── control.hpp          # Physics integration, flocking algorithm
│   │   ├── tasks.hpp            # FreeRTOS task management
│   │   ├── attacks.hpp          # Attack mode definitions
│   │   ├── mqtt_publish.hpp     # MQTT telemetry publishing
│   │   ├── wifi_connect.h       # WiFi connection (eduroam/WPA2)
│   │   ├── sntp_time.h          # NTP time synchronization
│   │   └── log_capture.h        # Console output logging to SD card
│   │
│   ├── src/
│   │   ├── main.cpp             # Application entry point
│   │   ├── tasks.cpp            # FreeRTOS task implementations
│   │   ├── comms.cpp            # LoRa RX/TX, CMAC verification, neighbor updates
│   │   ├── control.cpp          # Physics step, flocking logic
│   │   ├── attacks.cpp          # Attack implementations (Replay, Ghost, etc.)
│   │   ├── mqtt_publish.cpp     # MQTT client and JSON serialization
│   │   ├── wifi_connect.c       # WiFi initialization
│   │   ├── sntp_time.c          # SNTP client
│   │   └── log_capture.c        # SD card logging
│   │
│   └── CMakeLists.txt           # ESP-IDF component configuration
│
├── COMP0221-MQTT-Visualiser/
│   ├── visualiser.py            # Python MQTT dashboard
│   ├── packet_structure.json    # Expected JSON format
│   ├── mac_dict.py              # MAC address name mapping
│   └── requirements.txt         # Python dependencies
│
├── CMakeLists.txt               # Project-level CMake configuration
├── sdkconfig                    # ESP-IDF configuration
└── README.md                    # This file
```

---

## Key Features

### 1. **Distributed Flocking Algorithm**
- **Physics Loop** (50 Hz): Position/velocity/yaw integration
- **Flocking Loop** (10 Hz): Reynolds rules (separation, cohesion, alignment)
  - Separation: Avoid crowding neighbors (5m minimum)
  - Cohesion: Steer toward center of mass
  - Alignment: Match neighbor velocities
- **World Boundary**: 100m × 100m × 100m cube
- **Perception Radius**: 30m (neighbors detected within this range)

### 2. **Neighbor Discovery via LoRa**
- **Periodic Transmission** (every 6 seconds): Broadcasts own state
- **Continuous RX**: Monitors for incoming neighbor packets
- **Neighbor Table**: Maintains up to 10 neighbors with 20-second timeout
- **Sequence Number Replay Check**: Prevents processing old packets

### 3. **Security (AES-128 CMAC)**
- **Message Authentication**: All packets signed with shared team key
- **CMAC Tag**: 4-byte truncated from 16-byte AES-128 CMAC
- **Verification**: Rejects invalid/unsigned packets
- **Key**: Stored in `config.hpp` (shared with visualization dashboard)

### 4. **MQTT Telemetry Publishing**
- **Broker**: `broker.hivemq.com:1883` (public HiveMQ)
- **Topic**: `dubai2026`
- **Payload**: JSON with position, velocity, yaw, and CMAC signature
- **Format**: Matches visualizer's expected schema
- **Frequency**: 2 Hz (every 500ms)

### 5. **Wireless Connectivity**
- **WiFi**: Eduroam (WPA2-Enterprise) or standard WPA2
- **Time Sync**: SNTP to `pool.ntp.org`
- **Connection Retry**: Max 5 attempts with backoff

### 6. **Attack Modes** (Simulation)
Compile-time toggleable modes for security testing:
- **None**: Normal operation (default)
- **Replay**: Resend old neighbor packets
- **Ghost**: Inject fake neighbor positions
- **FalseData**: Corrupt position/velocity fields
- **FastTx**: Transmit 10× more frequently
- **InvalidCmac**: Send packets with bad CMAC tags

### 7. **Logging & Monitoring**
- **Serial Console**: Real-time status logs
- **SD Card Logging**: Optional `/sdcard/app_log.txt` (all ESP_LOG output)
- **Telemetry**: Periodic state snapshots
- **MQTT Debug**: Published message IDs and connection status

---

## Building & Flashing

### Prerequisites
```bash
# Install ESP-IDF v5.3+
git clone https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.3
./install.sh
source ./export.sh
```

### Build
```bash
cd coursework1
idf.py build
```

### Flash
```bash
idf.py flash -p /dev/ttyUSB0 -b 460800
```

### Monitor Serial Output
```bash
idf.py monitor -p /dev/ttyUSB0
```

---

## Configuration

All configuration is in [main/include/config.hpp](main/include/config.hpp):

### Wireless & Network
```cpp
TEAM_ID = 0                              // Your team identifier
PROTOCOL_VERSION = 1                    // Packet version
```

### Task Frequencies (Hz)
```cpp
PHYSICS_HZ = 50                          // Physics integration frequency
FLOCK_HZ = 10                            // Flocking algorithm frequency
RADIO_HZ = 4                             // Neighbor TX frequency (every 6 seconds)
TELEMETRY_HZ = 2                         // MQTT publish frequency
```

### Flocking Weights (Reynolds)
```cpp
ALIGNMENT_WEIGHT = 0.02f                 // Velocity alignment strength
COHESION_WEIGHT = 0.05f                  // Center-of-mass attraction
SEPARATION_WEIGHT = 0.15f                // Collision avoidance strength
MIN_SEPARATION_MM = 5000                 // Minimum separation distance (5m)
PERCEPTION_RADIUS_MM = 30000             // Neighbor detection range (30m)
MAX_VELOCITY_MM_S = 500                  // Speed limit (500 mm/s)
```

### Neighbor Table
```cpp
MAX_NEIGHBOURS = 10                      // Maximum stored neighbors
NEIGHBOUR_TIMEOUT_MS = 20000             // Remove neighbor after 20s of silence
```

### LoRa Radio
```cpp
LORA_FREQ_HZ = 868100000                 // 868.1 MHz (EU ISM band)
LORA_BANDWIDTH = 250000                  // 250 kHz bandwidth
LORA_SPREADING_FACTOR = 9                // SF9 (balance range/data-rate)
```

### Attack Modes (Toggle Attacks)
```cpp
// Enable exactly one attack (set to 1), others must be 0
ATTACK_ENABLE_NONE = 1                   // ← Currently enabled
ATTACK_ENABLE_REPLAY = 0
ATTACK_ENABLE_GHOST = 0
ATTACK_ENABLE_FALSEDATA = 0
ATTACK_ENABLE_FASTTX = 0
ATTACK_ENABLE_INVALIDCMAC = 0
```

To enable an attack, set its `_ENABLE` flag to `1` and all others to `0`. Rebuild with `idf.py build flash monitor`.

### WiFi Credentials
Edit [main/src/wifi_connect.c](main/src/wifi_connect.c):
```c
#define USE_EDUROAM 1                    // 1 = Eduroam, 0 = WPA2-PSK

#if USE_EDUROAM
    #define WIFI_SSID "eduroam"
    #define EDUROAM_IDENTITY "zcablya@ucl.ac.uk"
    #define EDUROAM_USERNAME "zcablya@ucl.ac.uk"
    #define EDUROAM_PASSWORD "YourPassword"
#else
    #define WIFI_SSID "YourSSID"
    #define WIFI_PASSWORD "YourPassword"
#endif
```

---

## Network Protocol

### Packet Format (46 bytes)
```
Byte   Field               Size   Type       Notes
────────────────────────────────────────────────────────────
0      version             1      uint8_t    PROTOCOL_VERSION
1      team_id             1      uint8_t    TEAM_ID
2-7    node_id             6      uint8_t[6] MAC address
8-9    seq_number          2      uint16_t   Packet sequence number
10-13  ts_s                4      uint32_t   Unix timestamp (seconds)
14-15  ts_ms               2      uint16_t   Timestamp (milliseconds)
16-19  x_mm                4      uint32_t   X position (mm)
20-23  y_mm                4      uint32_t   Y position (mm)
24-27  z_mm                4      uint32_t   Z position (mm)
28-31  vx_mm_s             4      int32_t    X velocity (mm/s)
32-35  vy_mm_s             4      int32_t    Y velocity (mm/s)
36-39  vz_mm_s             4      int32_t    Z velocity (mm/s)
40-41  yaw_cd              2      uint16_t   Yaw angle (centidegrees, 0-36000)
42-45  mac_tag             4      uint8_t[4] AES-128 CMAC (last 4 bytes)
────────────────────────────────────────────────────────────
Total: 46 bytes
```

### CMAC Verification
- **Input**: Bytes 0-41 (all fields except mac_tag)
- **Algorithm**: AES-128-CMAC (NIST FIPS 198)
- **Key**: 16-byte team key from `config.hpp`
- **Output**: Full 16-byte tag → truncate to last 4 bytes
- **Verification**: Constant-time comparison, reject on mismatch

### MQTT JSON Payload
```json
{
  "version": 1,
  "team_id": 0,
  "node_id": "F0:24:F9:AF:32:88",
  "seq_number": 42,
  "ts_s": 1702983456,
  "ts_ms": 123,
  "x_mm": 45000,
  "y_mm": 51000,
  "z_mm": 10300,
  "vx_mm_s": 100,
  "vy_mm_s": 5,
  "vz_mm_s": -230,
  "yaw_cd": 2700,
  "mac_tag": "a1b2c3d4"
}
```

---

## MQTT Visualization

### Running the Dashboard
```bash
cd COMP0221-MQTT-Visualiser
pip install -r requirements.txt
python visualiser.py
```

The visualizer will:
1. Connect to `broker.hivemq.com:1883`
2. Subscribe to `dubai2026` topic
3. Display all nodes in a 3D scatter plot
4. Update every 500ms with published telemetry
5. Verify CMAC signatures before displaying

**Important**: The visualizer uses the same CMAC key from `packet_structure.json`. Both ESP32 and visualizer must have the same key for communication to work.

---

## Troubleshooting

### WiFi Not Connecting
- Check credentials in `wifi_connect.c`
- For Eduroam: Ensure correct UCL email and password
- Monitor logs for "Retrying connection" messages
- Maximum 5 retry attempts before timeout

**Log Entry:**
```
I (2404) WIFI_CONNECT: Got IP: 10.23.47.70
I (2404) WIFI_CONNECT: Wi-Fi Connected.
```

### MQTT Not Publishing
- Check WiFi is connected first
- Verify broker is reachable: `ping broker.hivemq.com`
- Check logs for `MQTT_EVENT_CONNECTED`
- MQTT may fail silently if not connected

**Log Entry:**
```
I (10424) MQTT: MQTT_EVENT_CONNECTED
```

### Neighbors Not Appearing
- Check LoRa antenna is connected properly
- Verify other drone is transmitting (check its serial logs)
- Check neighbor timeout hasn't expired (20 seconds max)
- Verify CMAC key matches (if signature fails, packet is rejected)

**Log Entries (RX Success):**
```
I (180474) COMMS_RX: RX: from 90:15:06:da:cf:d0 seq=1 pos=(50142,50066,1594)
I (180654) TASKS: telemetry: x=44995 y=51000 z=10290 v=(-198,-10,453) neigh=1
```

**Log Entries (RX Failure - CMAC):**
- No message printed (silently rejected)
- Check neighbor count stays at 0

### Physics/Flocking Not Working
- Check all tasks started: look for `physics_task started`, `flock_task started`, etc.
- Verify velocity is non-zero in telemetry logs
- Check position changes over time (not frozen)

**Log Entry (Normal Operation):**
```
I (185154) TASKS: telemetry: x=44955 y=51000 z=10425 v=(0,0,0) neigh=0
```

### SD Card Logging Not Working
- Ensure SD card is mounted at `/sdcard`
- Check logs for `log_capture_init()` messages
- File should appear as `/sdcard/app_log.txt`
- Verify SD card is formatted (FAT32 recommended)

---

## Task Architecture

### FreeRTOS Tasks (Concurrent Execution)

| Task         | Priority | Frequency | Purpose |
|---|---|---|---|
| `physics_task` | 10 | 50 Hz | Position/velocity/yaw integration |
| `flock_task` | 10 | 10 Hz | Compute flocking control commands |
| `radio_task` | 10 | 4 Hz | Manage neighbor table (cull stale entries) |
| `telemetry_task` | 10 | 2 Hz | Log state & publish MQTT |
| `radio_rx_task` | 10 | Continuous | LoRa RX loop, periodic TX |

**State Management:**
- All tasks access `g_state` (LocalState) through mutex-protected snapshots
- Physics task updates state
- Flocking task reads state and neighbor table
- Telemetry task snapshots for logging

---

## Development Tips

### Adding a New Attack Mode
1. Add enum value in `config.hpp` AttackMode
2. Implement in `attacks.cpp` inside `apply_attacks()`
3. Add config toggle in `config.hpp` (ATTACK_ENABLE_*)
4. Rebuild and test

### Modifying Flocking Weights
1. Edit weights in `config.hpp`
2. Rebuild: `idf.py build`
3. Flash and monitor behavior

### Increasing LoRa Range
1. Increase spreading factor (SF): Currently SF=9, can go to SF=12 (slower)
2. Reduce bandwidth: Currently 250 kHz, can use 125 kHz (slower)
3. Edit in `config.hpp` LORA_* defines

### Debugging Neighbor Issues
- Add printf in `comms::update_neighbour_from_packet()`
- Check `last_seen_ms` timestamp logic
- Verify time doesn't wrap (use `& 0xFFFFFFFF` mask)

---

## CMAC Key Setup

**Current Key** (in `main/src/main.cpp`):
```cpp
const uint8_t TEAM_KEY[16] = {
    0x2B, 0x7E, 0x15, 0x16, 0x22, 0xA0, 0xD2, 0xA6,
    0xAC, 0xF7, 0x19, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};
```

This key must match in:
1. **ESP32 Code** (`main/src/main.cpp`)
2. **Visualizer Dashboard** (`COMP0221-MQTT-Visualiser/visualiser.py`)

To use a different key:
```cpp
// In main/src/main.cpp:
const uint8_t TEAM_KEY[16] = {
    0x00, 0x01, 0x02, 0x03,  // Your 16 bytes here
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B,
    0x0C, 0x0D, 0x0E, 0x0F,
};

// In COMP0221-MQTT-Visualiser/visualiser.py:
SHARED_KEY_HEX = "000102030405060708090A0B0C0D0E0F"
```

---

## Performance Notes

- **RAM Usage**: ~100 KB (neighbor table, state, buffers)
- **CPU Usage**: ~15% average across all tasks
- **LoRa Airtime**: ~1 second per packet (SF9, 250 kHz)
- **Power Consumption**: ~200 mA active (WiFi+LoRa), 50 mA idle

---

## References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/)
- [RadioLib SX1276 Driver](https://github.com/jgromes/RadioLib)
- [NIST FIPS 198 - CMAC](https://nvlpubs.nist.gov/nistpubs/Legacy/SP/nistspecialpublication800-38b.pdf)
- [HiveMQ MQTT Broker](https://www.hivemq.com/)
- [Reynolds Flocking Algorithm](https://www.red3d.com/cwr/boids/)

---

## License & Course Info

**Course**: COMP0221 Networking - UCL  
**Year**: 2025  
**Student**: L-Yanc

This is coursework for educational purposes. The flocking algorithm implementation is based on Reynolds' Boids model, and the security aspects focus on packet authentication via CMAC.

---

## Contact & Support

For issues or questions:
1. Check the [Troubleshooting](#troubleshooting) section
2. Review ESP-IDF logs for detailed error messages
3. Verify all hardware connections and power supply
4. Ensure WiFi and time synchronization are working

---

**Last Updated**: December 19, 2025
