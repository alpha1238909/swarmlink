# ESP32 ESP-NOW reference adapter

`esp32_espnow_node.ino` is a small, telemetry-only reference for a nearby ESP32 team. It broadcasts compact `swarmlink.telemetry.compact.v1` JSON over ESP-NOW, then SwarmLink expands it into the same `NodeTelemetry` data used by the readable `swarmlink.telemetry.v1` format. The receiver forwards frames to its USB serial port, where a laptop or Raspberry Pi can record them with the SwarmLink CLI.

It does **not** read a particular GPS, IMU, UWB module, or flight controller by default, and it does not command motors. This is deliberate: position estimates and vehicle safety logic remain specific to each build.

## Bring-up

1. In the Arduino IDE, select an ESP32-family board and open `esp32_espnow_node.ino`.
2. Give each board a unique `NODE_ID`; set `VEHICLE_KIND` to `drone`, `rover`, `sensor`, `boat`, or `unknown`.
3. Flash at least two boards. All must share an ESP-NOW Wi-Fi channel. The reference uses broadcast while bringing up a private bench network.
4. Connect one receiver board to the base station over USB. From the SwarmLink source directory:

   ```bash
   python -m pip install '.[serial]'
   swarmlink bridge-serial --port COM4 --baudrate 115200 --out field-test.jsonl
   ```

   On Linux, the port may be `/dev/ttyUSB0` or `/dev/ttyACM0`. Stop with `Ctrl+C`; accepted records are written as standard SwarmLink JSON Lines.

5. Inspect the recorded mission:

   ```bash
   swarmlink inspect field-test.jsonl
   ```

## Connecting real sensors

Replace `readYourSensors()` with your hardware-specific integration. Map all values into one local ENU frame:

| Packet value | Required convention |
|---|---|
| `position.x_m`, `position.y_m`, `position.z_m` | east, north, up in metres |
| `velocity.x_mps`, `velocity.y_mps`, `velocity.z_mps` | east, north, up in metres per second |
| `timestamp_s` | Unix UTC seconds; synchronize from GPS or NTP for live stale-node checks |
| `battery_percent` | `0` to `100` |

Keep the whole JSON frame below 250 bytes, the ESP-NOW v1 payload limit. The reference has a 240-byte guard. Keep `node_id` and `mission_state` short and omit optional health fields on this low-bandwidth broadcast path.

## Deployment notes

- The sketch uses broadcast and no ESP-NOW encryption so it remains easy to reproduce in a bench test. Treat broadcasts as untrusted.
- For a deployed project, use paired peers where suitable, keep radio keys off Git, and add independent application authentication at the receiving boundary. SwarmLink's Python `PacketAuthenticator` is available for authenticated transport integrations.
- ESP-NOW delivery is best-effort. Test expected range, interference, loss, and latency on the ground; use SwarmLink's fault injection before relying on any live coordination decision.
- The base-station serial bridge treats lines beginning with `#` as invalid telemetry and safely skips them.
