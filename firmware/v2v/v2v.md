# V2V Communication — Second Car (Emergency Response)

**Purpose**:  
Implements a Vehicle-to-Vehicle (V2V) emergency response system on a second car using an **ESP32**.  
It listens for **UDP broadcasts** sent from the primary vehicle’s `communication_node.py` (when a critical event like `"DRVR_FAINTED"` is detected).

---

##  Key Components

- **Hardware**:
  - ESP32
  - 16x2 LCD (I2C, address `0x27`)
  - Buzzer (GPIO 18)
  - Motors (GPIO 25, 26, 32, 33)
- **Libraries**:
  - `Wire.h`
  - `LiquidCrystal_I2C.h`
  - `WiFi.h`
  - `WiFiUdp.h`
- **WiFi Config**:
  - SSID: `ros`
  - Password: `00000000`
  - UDP Port: `5555`
  - Buffer: 255 bytes

---

## Core Functions

| Function           | Description                                             |
|--------------------|---------------------------------------------------------|
| `setupReceiver()`  | Connects to WiFi, starts UDP, disables sleep            |
| `receiveBroadcast()` | Listens for incoming UDP messages                    |
| `Forward_Motors()` | Drives motors forward with very low PWM                |
| `Emergency_Alert()` | Activates buzzer + LCD message in response to alert   |
| `starting_message()` | Displays welcome message at boot                     |

---

##  Workflow

1. **Setup Phase**:
    - Initializes all hardware (motors, buzzer, LCD).
    - Connects to WiFi and starts UDP on port `5555`.
    - Displays `"Welcome, ADMAS!"` and starts moving forward slowly.

2. **Loop Phase**:
    - Listens for broadcast message from primary car.
    - If `"DRVR_FAINTED"` or other message is received:
        - Triggers `Emergency_Alert()`:
          - LCD displays `"EMERGENCY ALERT!"`
          - Buzzer sounds for 1.5 seconds
    - Resets after every alert.

---

##  Role in ADMAS

This ESP32 node serves as a **secondary safety vehicle**:

- Receives emergency broadcasts from the main car (ROS 2 node `communication_node.py`)
- Instantly responds with visual (LCD) and audio (buzzer) alerts
- Prevents rear-end collision or alerts other systems to slow down or assist
- Fully WiFi-based using UDP for rapid V2V messaging

---

