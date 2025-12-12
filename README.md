# ESP32 BLDC PID Tuning Tool (DJI RoboMaster)

A real-time PID tuning framework for **DJI RoboMaster M3508 and M2006** brushless motors using an **ESP32**, **MCP2515 CAN module**, and a **PS4 Controller**.

This project allows you to tune P, I, and D parameters on the fly via the Serial Monitor without re-uploading code, visualize performance using the Serial Plotter, and toggle between different motor types dynamically.

## 🌟 Features

* **Real-time Tuning:** Update $K_p$, $K_i$, and $K_d$ values instantly via Serial commands.
* **Multi-Motor Support:** Control and tune up to 4 motors simultaneously or individually.
* **Motor Type Switching:** Dynamically switch configuration between **M3508** (19:1 gearbox) and **M2006** (36:1 gearbox) to ensure correct RPM calculations and current limits.
* **Visual Debugging:** Built-in "Plotter Mode" (`v` command) formats data for the Arduino Serial Plotter to visualize step response and oscillation.
* **Wireless Control:** Use a PS4 controller (DualShock 4) to drive the motors.
* **Safety Limits:** Automatic current limiting based on motor type (16384 for M3508, 10000 for M2006) and integral anti-windup.

## 🛠 Hardware Requirements

1.  **Microcontroller:** ESP32 Development Board.
2.  **CAN Bus Module:** MCP2515 (SPI to CAN).
    * *Note:* Check the crystal oscillator on your module. It is usually 8MHz or 16MHz.
3.  **Motors:** DJI RoboMaster M3508 (with C620 ESC) or M2006 (with C610 ESC).
4.  **Controller:** Sony DualShock 4 (PS4 Controller).
5.  **Power Supply:** 24V DC Power Supply or LiPo Battery (XT60).
6.  **CAN Cable:** Twisted pair wire for CAN High / CAN Low.

## 🔌 Wiring

### MCP2515 to ESP32
| MCP2515 Pin | ESP32 Pin | Note |
| :--- | :--- | :--- |
| VCC | 5V | 5V is recommended for the TJA1050 transceiver |
| GND | GND | Common Ground |
| CS | **GPIO 5** | Defined in code |
| SO (MISO) | GPIO 19 | Standard VSPI |
| SI (MOSI) | GPIO 23 | Standard VSPI |
| SCK | GPIO 18 | Standard VSPI |
| INT | Not Used | |

### CAN Bus Connection
| MCP2515 | C620 / C610 ESC |
| :--- | :--- |
| H | CAN H |
| L | CAN L |

*Note: Ensure the 120Ω termination resistor jumper is installed on the MCP2515 if it is at the end of the bus.*

## 📦 Software Dependencies

Install the following libraries via the Arduino Library Manager:

1.  **mcp2515** by *autowp* (Interface for the CAN module).
2.  **PS4Controller** (ESP32-PS4) by *Bluepad32* or *SR04* (Ensure you have the library compatible with the `#include <PS4Controller.h>` header).

## 🚀 Setup & Installation

1.  **Check Crystal Frequency:**
    Inspect your MCP2515 module.
    * If the silver crystal says `8.000`, ensure line 44 in `setup()` is:
      `mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);`
    * If it says `16.000`, change it to:
      `mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);`

2.  **PS4 Pairing:**
    You must pair your PS4 controller to the ESP32's Bluetooth MAC address.
    * Get the MAC address from the code: `84:1f:e8:68:f8:a2` (Line 9).
    * Use a tool like [SixaxisPairTool](https://github.com/user-none/SixaxisPairTool) on your PC to set your Controller's Master MAC to this address.
    * *Alternatively,* change the string in `PS4.begin(...)` to the stored MAC address of your specific controller.

3.  **Upload:**
    Connect your ESP32 and upload the sketch using Arduino IDE.

## 🎛 Usage Guide

### 1. Serial Command Reference
Open the Serial Monitor at **115200 baud**. You can type these commands to change settings instantly.

**Syntax Legend:** `[ID]` = Motor ID (1-4). Use `0` to select ALL motors.

| Command Type | Syntax | Example | Description |
| :--- | :--- | :--- | :--- |
| **Proportional (Kp)** | `p[ID]_[Value]` | `p1_15.5` | Sets Kp for Motor 1 to 15.5 |
| **Integral (Ki)** | `i[ID]_[Value]` | `i0_0.05` | Sets Ki for **ALL** motors to 0.05 |
| **Derivative (Kd)** | `d[ID]_[Value]` | `d2_1.2` | Sets Kd for Motor 2 to 1.2 |
| **Set Speed** | `s[Value]` | `s300` | Sets global target speed to 300 RPM |
| **Motor Type** | `t[ID]_[Type]` | `t1_2006` | Sets Motor 1 type to M2006 |
| **Toggle Plotter** | `v` | `v` | Toggles Serial spam for the Plotter |
| **Check Settings** | `?` | `?` | Prints current PID, Configs & Speed |

**Motor Type Examples:**
* `t1_3508`: Configures Motor 1 as M3508 (19:1 ratio, Max Current 16384).
* `t0_2006`: Configures **ALL** motors as M2006 (36:1 ratio, Max Current 10000).

### 2. Tuning Workflow
1.  **Start Safe:** Send `s0` to stop motors. Set PIDs to zero: `p0_0`, `i0_0`, `d0_0`.
2.  **Set Target:** Set a safe testing speed, e.g., `s500`.
3.  **Increase P:** Send `p1_10`, then `p1_20`, etc., until the motor responds and oscillates slightly.
4.  **Add I:** If the motor doesn't reach the target (steady-state error), add small amounts of I: `i1_0.1`.
5.  **Dampen with D:** If the motor overshoots or jitters, add D: `d1_5.0`.
6.  **Visualize:** Type `v` and open the **Serial Plotter** (Tools > Serial Plotter) to see the Target vs. Actual curve.

### 3. PS4 Control
* **Left Stick Y (Up/Down):** Controls the target RPM.
* **Deadzone:** Stick input must be `> 10` to activate motors.
* **Reset:** Releasing the stick resets the PID integral and Previous Error terms to prevents "windup jumps" when starting again.

## ⚠️ Troubleshooting

**Q: The motors are not moving.**
* Check the Serial Monitor. Does it say "CAN MODULE FAILURE"? If so, check wiring.
* Check the crystal on the MCP2515. If it's 16MHz but code says 8MHz, CAN will fail.
* Check that the ESCs have power (Green LED blinking usually).

**Q: The motor beeps 3 times and blinks red.**
* **Overheat Protection:** The internal capacitors are hot. This happens during aggressive PID tuning (violent shaking).
* **Fix:** Stop the motor immediately. Turn off power. Wait 30 seconds. Reduce your `Kp` value significantly before trying again.

**Q: The Serial Plotter is empty or messy.**
* Ensure you typed `v` in the Serial Monitor to enable data streaming.
* Close the Serial Monitor before opening the Serial Plotter (only one can be open at a time).
