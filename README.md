# ORION
Orion is a custom rocket flight system. It includes a ground station and a flight controller that work together to transmit telemetry, monitor the rocket’s status, and perform active control and data logging during flight.

# How to use?
To use Orion, you connect the ground station to a laptop. The UI runs in the Processing IDE, and the main board plus ground station firmware is written and uploaded using PlatformIO. From the interface, you can see the rocket’s current state, switch between modes (for example, test and launch), command servo tests, read raw sensor data, and run calibration routines.

# Why?
I built Orion because ever since I started rocketry in 4th grade, I’ve been inspired by how SpaceX monitors and controls their rockets in real time. I wanted a similar system for my own launches, where I can see how the rocket is performing live, get data during the flight, and not just read everything after recovery.

<img width="353" height="761" alt="Screenshot 2026-03-01 at 1 56 29 AM" src="https://github.com/user-attachments/assets/4f232747-7d8f-47d6-9719-353b2c10f9f3" />

<img width="1521" height="762" alt="Screenshot 2026-03-01 at 1 56 41 AM" src="https://github.com/user-attachments/assets/0e059c54-848a-43b9-b6bf-9c7952376262" />

# Ground Station BOM
| Product Name      | Product Link | Product Cost | Product Amount |
| ----------------- | ------------ | ------------ | -------------- |
| Raspberry Pi Pico | Adafruit     | $4.00        | 1              |
| Adafruit RFM95W   | Adafruit     | $19.95       | 1              |
| Antenna - 2dBi    | Adafruit     | $5.95        | 1              |
| Edge-Launch SMA   | Adafruit     | $2.50        | 1              |
| PCB               | JLCPCB       | $2.00        | 5              |

#Main Flight Computer BOM
| Product Name                  | Product Link | Product Cost | Product Amount |
| ----------------------------- | ------------ | ------------ | -------------- |
| Raspberry Pi Pico             | Adafruit     | $4.00        | 1              |
| LSM6DSO32                     | Already have | $0.00        | 1              |
| BMP390                        | Already have | $0.00        | 1              |
| Flash                         | Already have | $0.00        | 1              |
| Adafruit Ultimate GPS         | Adafruit     | $29.95       | 1              |
| GPS Antenna - External Active | Adafruit     | $21.50       | 1              |
| SMA to uFL                    | Adafruit     | $3.95        | 1              |
| 3v reg (sensors/pico)         | Already have | $0.00        | 1              |
| 5v reg (for servo)            | Already have | $0.00        | 1              |
| Terminal blocks               | Already have | $0.00        | 1              |
| Adafruit RFM95W               | Adafruit     | $19.95       | 1              |
| Antenna - 2dBi                | Adafruit     | $5.95        | 1              |
| Edge-Launch SMA               | Adafruit     | $2.50        | 1              |
| PCB                           | JLCPCB       | $6.20        | 5              |
