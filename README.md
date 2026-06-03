# ESP32-C6 SuperMini RFID + Buzzer Firmware

PlatformIO ESP-IDF firmware for a MakerGO-style ESP32-C6 Super Mini wired to an MFRC522/RC522 RFID reader and a buzzer.

## Wiring

| RC522 / MFRC522 | ESP32-C6 SuperMini |
|---|---:|
| 3.3V | 3V3 |
| GND | GND |
| SDA / SS / CS | GPIO4 |
| SCK | GPIO7 |
| MOSI | GPIO5 |
| MISO | GPIO6 |
| RST | GPIO3 |
| IRQ | GPIO8 |

| Buzzer | ESP32-C6 SuperMini |
|---|---:|
| + / S / IN | GPIO2 |
| - / GND | GND |

RC522 must be powered from 3.3V, not 5V.

## Build / upload

```powershell
pio run -e esp32-c6-supermini
pio run -e esp32-c6-supermini -t upload
pio device monitor -b 115200
```

## Behavior

- Initializes SPI on GPIO4/5/6/7.
- Resets and initializes the MFRC522.
- Configures RC522 IRQ pin on GPIO8 as an input.
- Scans for ISO14443A cards.
- Prints card UID over serial.
- Chirps buzzer on a newly read card.

## Pin caveat

GPIO8 is marked as BOOT/RGB LED on some ESP32-C6 SuperMini pinouts. It is used here because you requested RC522 IRQ on GPIO8. If the board starts booting strangely, move IRQ to another free input pin and update `RFID_IRQ_PIN` in `platformio.ini`.
