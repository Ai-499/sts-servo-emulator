#!/usr/bin/env python3
"""
Проверка живости платы напрямую по протоколу, в обход bambot.
Шлёт PING всем ID 1..6 и печатает, кто ответил.

Использование:
    python ping_all.py /dev/ttyUSB0

Требуется: pip install pyserial
"""
import sys
import time
import serial

INST_PING = 0x01
BAUDRATE = 1_000_000


def calc_checksum(servo_id: int, length: int, inst: int, params: bytes = b"") -> int:
    s = servo_id + length + inst + sum(params)
    return (~s) & 0xFF


def build_ping(servo_id: int) -> bytes:
    length = 2
    checksum = calc_checksum(servo_id, length, INST_PING)
    return bytes([0xFF, 0xFF, servo_id, length, INST_PING, checksum])


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.3)
    except serial.SerialException as e:
        print(f"НЕ УДАЛОСЬ ОТКРЫТЬ ПОРТ: {e}")
        print("Возможно порт занят (bambot/другой скрипт держит его открытым)"
              " или плата не видна системе (переподключи USB).")
        sys.exit(1)

    with ser:
        time.sleep(2.0)  # ждём авто-перезагрузку платы после открытия порта
        ser.reset_input_buffer()

        alive = []
        for servo_id in range(1, 7):
            ser.reset_input_buffer()
            ser.write(build_ping(servo_id))
            time.sleep(0.02)
            resp = ser.read(ser.in_waiting or 6)
            status = "OK" if len(resp) >= 6 else "нет ответа"
            print(f"ID={servo_id}: {status}  ({resp.hex(' ') if resp else '-'})")
            if len(resp) >= 6:
                alive.append(servo_id)

    print()
    if len(alive) == 6:
        print("Все 6 ID отвечают — плата и прошивка живы. Проблема на стороне bambot/порта в браузере.")
    elif alive:
        print(f"Отвечают только: {alive}. Не все суставы видны — возможна проблема с конкретным ID.")
    else:
        print("Ни один ID не ответил — плата не отвечает вообще (проверь порт, питание, USB-кабель).")


if __name__ == "__main__":
    main()
