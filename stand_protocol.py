# stand_protocol.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional


@dataclass(frozen=True)
class StandChannel:
    id_hex: int
    name: str
    unit: str
    scale: float  # умножаем raw на scale, чтобы получить отображаемую величину


# По вашему описанию:
# - силы: приходят в мН, отображать в даН => 1 даН = 10 Н = 10000 мН => daN = mN / 10000
# - энкодер: 1 “тик” = 0.225 градуса => deg = raw * 0.225
DEFAULT_CHANNELS: List[StandChannel] = [
    StandChannel(0xC1, "Сила1_AD7799", "даН", 1.0 / 10000.0),
    StandChannel(0xC2, "Сила2_AD7799", "даН", 1.0 / 10000.0),
    StandChannel(0xC3, "Сила1_HX711",  "даН", 1.0 / 10000.0),
    StandChannel(0xC4, "Сила2_HX711",  "даН", 1.0 / 10000.0),
    # Если захотите вместо каких-то сил показывать энкодеры — просто замените каналы:
    # StandChannel(0xC5, "Энкодер1", "град", 0.225),
    # StandChannel(0xC6, "Энкодер2", "град", 0.225),
]


def build_channel_map(channels: List[StandChannel]) -> Dict[int, StandChannel]:
    return {ch.id_hex: ch for ch in channels}
ALLOWED_IDS = {0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6}

SCALE_DAN = 1.0 / 10000.0  # мН -> даН
SCALE_DEG = 0.225          # тики -> градусы (если raw = "кол-во шагов")


def parse_stand_packet(packet: bytes):
    """
    packet = START, LENP, затем M*(ID + 4 байта)
    DATA = int32 big-endian signed
    return: start, lenp, list[(id, raw_int32)]
    """
    if len(packet) < 2:
        raise ValueError("short")
    start = packet[0]
    lenp = packet[1]
    if len(packet) != lenp:
        raise ValueError("lenp mismatch")

    payload = packet[2:]
    if len(payload) == 0 or (len(payload) % 5) != 0:
        raise ValueError("payload not multiple of 5")

    out = []
    for i in range(0, len(payload), 5):
        pid = payload[i]
        if pid not in ALLOWED_IDS:
            raise ValueError(f"bad id {pid:#x}")
        raw = int.from_bytes(payload[i+1:i+5], byteorder="little", signed=True)
        out.append((pid, raw))

    return start, lenp, out


def extract_packets_from_stream(buf: bytearray):
    """
    Строгая ресинхронизация:
    ищем позицию pos, где:
      - есть START и LENP
      - LENP >= 7 и LENP-2 кратно 5
      - хватает данных до pos+LENP
      - каждый подпакет начинается с ID из ALLOWED_IDS
    """
    packets = []

    def looks_like_packet_at(pos: int) -> bool:
        if pos + 2 > len(buf):
            return False

        lenp = buf[pos + 1]
        if lenp < 7 or lenp > 255:
            return False

        end = pos + lenp
        if end > len(buf):
            return False

        payload_len = lenp - 2
        if payload_len <= 0 or (payload_len % 5) != 0:
            return False

        payload = buf[pos + 2:end]
        for i in range(0, payload_len, 5):
            if payload[i] not in ALLOWED_IDS:
                return False

        return True

    while True:
        if len(buf) < 2:
            break

        found = None
        for pos in range(0, len(buf) - 1):
            if looks_like_packet_at(pos):
                found = pos
                break

        if found is None:
            # не даём буферу разрастаться на мусоре
            if len(buf) > 8192:
                del buf[:-256]
            break

        lenp = buf[found + 1]
        pkt = bytes(buf[found:found + lenp])
        del buf[:found + lenp]
        packets.append(pkt)

    return packets