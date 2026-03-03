# reader.py
import serial
import time


def read_thread(
    SERIAL_PORT,
    BAUD_RATE,
    READ_CHUNK,
    data_queue,
    queue_lock,
    raw_bytes_buffer,
    raw_lock,
    stop_event,                 # <-- ДОБАВИЛИ
):
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=0.1,
            rtscts=False,
            dsrdtr=False
        )
    except Exception as e:
        print(f"[ОШИБКА] Не удалось открыть порт {SERIAL_PORT}: {e}")
        return

    print(f"[ИНФО] Порт {SERIAL_PORT} открыт")

    cnt = 0
    t0 = time.time()

    while not stop_event.is_set():          # <-- БЫЛО while True
        try:
            data = ser.read(READ_CHUNK)
            if data:
                cnt += len(data)

                with queue_lock:
                    data_queue.append(data)

                with raw_lock:
                    raw_bytes_buffer.extend(data)

            if time.time() - t0 >= 1.0:
                print(f"[RX] {cnt} bytes/s")
                cnt = 0
                t0 = time.time()

            time.sleep(0.001)

        except Exception as e:
            print(f"[ОШИБКА] COM-порт: {e}")
            break

    try:
        ser.close()
    except Exception:
        pass
    print(f"[ИНФО] Порт {SERIAL_PORT} закрыт")