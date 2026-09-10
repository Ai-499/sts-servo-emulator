#!/usr/bin/env python3
"""
Калибровка нуля для эмулятора SO-101 leader arm (регистр 0x50).

Использование:
    python calibrate_zero.py COM5 3        # откалибровать сустав с ID=3
    python calibrate_zero.py /dev/ttyUSB0 1

ВАЖНО: перед запуском физически поставь этот сустав в КРАЙНЕЕ положение
хода (в один из упоров) — так, чтобы весь оставшийся диапазон движения
шёл только в одну сторону от этой точки. Тогда переход шкалы 4095/0
окажется там, куда сустав никогда не доезжает, и "прыжков" при движении
быть не должно.

Требуется: pip install pyserial
"""
import sys
import time
import serial

REG_CALIBRATE_ZERO = 0x50
INST_WRITE = 0x03
BAUDRATE = 1_000_000


def calc_checksum(servo_id: int, length: int, inst: int, params: bytes) -> int:
    s = servo_id + length + inst + sum(params)
    return (~s) & 0xFF


def build_write_packet(servo_id: int, addr: int, data: bytes) -> bytes:
    params = bytes([addr]) + data
    length = len(params) + 2  # instruction + params + checksum-длина
    checksum = calc_checksum(servo_id, length, INST_WRITE, params)
    return bytes([0xFF, 0xFF, servo_id, length, INST_WRITE]) + params + bytes([checksum])


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]
    servo_id = int(sys.argv[2])

    packet = build_write_packet(servo_id, REG_CALIBRATE_ZERO, bytes([0x01]))

    with serial.Serial(port, BAUDRATE, timeout=0.5) as ser:
        time.sleep(2.0)  # дать плате перезагрузиться после открытия порта (DTR reset)
        ser.reset_input_buffer()
        ser.write(packet)
        time.sleep(0.05)
        response = ser.read(ser.in_waiting or 6)

    print(f"Отправлено на ID={servo_id}: {packet.hex(' ')}")
    if response:
        print(f"Ответ платы:        {response.hex(' ')}")
        print("Похоже на корректный ACK." if len(response) >= 6 else "Ответ подозрительно короткий.")
    else:
        print("Ответа нет — проверь порт/ID/что прошивка залита с поддержкой регистра 0x50.")


if __name__ == "__main__":
    main()
