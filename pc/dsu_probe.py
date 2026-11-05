#!/usr/bin/env python3
"""Client DSU de test : verifie le pont sans lancer Dolphin.

Il fait exactement ce que fait Dolphin : demande la liste des manettes,
puis s'abonne aux donnees et affiche ce qu'il recoit.

    python3 pc/dsu_probe.py                 # resume sur 3 s
    python3 pc/dsu_probe.py --watch         # affichage continu
    python3 pc/dsu_probe.py --duration 10 --json
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import time

import dsu_protocol as dsu

MODEL_NAMES = {0: "aucun", 1: "gyro partiel", 2: "gyro complet", 3: "generique"}


def parse_port_info(payload: bytes) -> dict:
    slot, state, model, connection, mac, battery = struct.unpack_from("<BBBB6sB", payload, 0)
    return {
        "slot": slot,
        "connected": state == dsu.STATE_CONNECTED,
        "state": state,
        "model": model,
        "connection": connection,
        "mac": ":".join(f"{b:02x}" for b in mac),
        "battery": battery,
    }


def parse_pad_data(payload: bytes) -> dict:
    slot, state, model, connection, mac, battery, active, counter = struct.unpack_from(
        "<BBBB6sBBI", payload, 0
    )
    _button1, _button2, _ps, _touch = payload[16:20]
    lx, ly, rx, ry = payload[20:24]
    timestamp_us, = struct.unpack_from("<Q", payload, 48)
    ax, ay, az, gp, gy, gr = struct.unpack_from("<ffffff", payload, 56)
    return {
        "slot": slot,
        "connected": state == dsu.STATE_CONNECTED,
        "model": model,
        "connection": connection,
        "mac": ":".join(f"{b:02x}" for b in mac),
        "battery": battery,
        "active": active,
        "counter": counter,
        "timestamp_us": timestamp_us,
        "left_x": (lx - 128) / 127.0,
        "left_y": (ly - 128) / 127.0,
        "right_x": (rx - 128) / 127.0,
        "right_y": (ry - 128) / 127.0,
        "ax": ax,
        "ay": ay,
        "az": az,
        "gyro_pitch": gp,
        "gyro_yaw": gy,
        "gyro_roll": gr,
    }


def _send(sock: socket.socket, server: tuple[str, int], msg_type: int, payload: bytes) -> None:
    # Un client envoie "DSUC" ; le serveur repond "DSUS".
    sock.sendto(dsu.build_client_message(0x12345678, msg_type, payload), server)


def _request_ports(sock: socket.socket, server: tuple[str, int], count: int = 4) -> None:
    _send(sock, server, dsu.MSG_PORTS, struct.pack("<i", count) + bytes(range(count)))


def _request_data(sock: socket.socket, server: tuple[str, int], slot: int) -> None:
    _send(sock, server, dsu.MSG_DATA, struct.pack("<BB6s", 1, slot, bytes(6)))


def _recv(sock: socket.socket, timeout: float) -> tuple[bytes, tuple[str, int]] | None:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            return sock.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            return None
    return None


class Stats:
    def __init__(self) -> None:
        self.count = 0
        self.first: dict | None = None
        self.last: dict | None = None
        self.min: dict[str, float] = {}
        self.max: dict[str, float] = {}

    def add(self, data: dict) -> None:
        self.count += 1
        if self.first is None:
            self.first = data
        self.last = data
        for key in ("ax", "ay", "az", "gyro_pitch", "gyro_yaw", "gyro_roll", "left_x", "left_y"):
            value = data[key]
            self.min[key] = min(self.min.get(key, value), value)
            self.max[key] = max(self.max.get(key, value), value)

    def span(self, key: str) -> float:
        return self.max.get(key, 0.0) - self.min.get(key, 0.0)


def _fmt_range(stats: Stats, key: str, width: int = 5) -> str:
    return f"{stats.min.get(key, 0.0):+.{width}f}..{stats.max.get(key, 0.0):+.{width}f}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Client DSU de test (facon Dolphin).")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=26760)
    parser.add_argument("--slot", type=int, default=0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--json", action="store_true", help="un objet JSON par paquet")
    parser.add_argument("--watch", action="store_true", help="affichage continu")
    parser.add_argument("--require-motion", action="store_true",
                        help="echoue si aucune variation de mouvement n'est recue")
    args = parser.parse_args(argv)

    server = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))
    sock.settimeout(0.2)

    # 1. Liste des manettes (comme le thread "hotplug" de Dolphin).
    _request_ports(sock, server)
    port_info: dict | None = None
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        received = _recv(sock, 0.2)
        if received is None:
            continue
        data, addr = received
        if not dsu.crc_is_valid(data):
            continue
        request = dsu.parse_request(data)
        if request is not None and request.msg_type == dsu.MSG_PORTS and len(request.payload) >= 11:
            info = parse_port_info(request.payload)
            if info["slot"] == args.slot and info["connected"]:
                port_info = info
                break

    if port_info is None:
        print(
            f"Aucune manette DSU sur {args.host}:{args.port}. "
            "Le pont est-il demarre (python3 pc/dsu_bridge.py) ?",
            file=sys.stderr,
        )
        return 2

    if not args.json:
        print(
            f"Controleur DSU : slot {port_info['slot']}, modele {port_info['model']} "
            f"({MODEL_NAMES.get(port_info['model'], '?')}), "
            f"batterie {port_info['battery']}, MAC {port_info['mac']}"
        )

    # 2. Abonnement aux donnees, renouvele chaque seconde comme Dolphin.
    _request_data(sock, server, args.slot)
    stats = Stats()
    start = time.monotonic()
    next_request = start + 1.0
    last_counter: int | None = None
    duplicates = 0

    while time.monotonic() - start < args.duration:
        now = time.monotonic()
        if now >= next_request:
            _request_data(sock, server, args.slot)
            next_request = now + 1.0

        received = _recv(sock, 0.1)
        if received is None:
            continue
        raw, _addr = received
        if not dsu.crc_is_valid(raw):
            print("paquet recu avec un CRC invalide", file=sys.stderr)
            continue
        request = dsu.parse_request(raw)
        if request is None or request.msg_type != dsu.MSG_DATA:
            continue
        if len(request.payload) < 80:
            print(f"paquet de donnees trop court ({len(request.payload)} octets)", file=sys.stderr)
            continue

        data = parse_pad_data(request.payload)
        if last_counter is not None and data["counter"] == last_counter:
            duplicates += 1
        last_counter = data["counter"]
        stats.add(data)

        if args.json:
            print(json.dumps(data))
        elif args.watch:
            angle = math.degrees(math.atan2(data["ax"], data["az"]))
            sys.stdout.write(
                f"\rangle {angle:+7.1f} deg  accel ({data['ax']:+.3f}, {data['ay']:+.3f}, "
                f"{data['az']:+.3f}) g  stick {data['left_x']:+.2f}  paquets {stats.count}"
            )
            sys.stdout.flush()

    if args.watch and not args.json:
        sys.stdout.write("\n")

    if stats.count == 0:
        print(
            "Le controleur est annonce mais aucun paquet de donnees n'est arrive. "
            "Verifie que le pont tourne et que le port UDP n'est pas bloque.",
            file=sys.stderr,
        )
        return 3

    elapsed = time.monotonic() - start
    if not args.json:
        print(f"Paquets    : {stats.count} en {elapsed:.2f} s ({stats.count / elapsed:.1f}/s)")
        if duplicates:
            print(f"           : {duplicates} paquets avec un compteur identique")
        print(f"Accel (g)  : x {_fmt_range(stats, 'ax')}  y {_fmt_range(stats, 'ay')}  "
              f"z {_fmt_range(stats, 'az')}")
        print(f"Gyro (d/s) : pitch {_fmt_range(stats, 'gyro_pitch')}  "
              f"yaw {_fmt_range(stats, 'gyro_yaw')}  roll {_fmt_range(stats, 'gyro_roll')}")
        print(f"Stick      : x {_fmt_range(stats, 'left_x')}  y {_fmt_range(stats, 'left_y')}")

    moved = (
        stats.span("ax") > 0.05
        or stats.span("ay") > 0.05
        or stats.span("az") > 0.05
        or stats.span("gyro_roll") > 1.0
        or stats.span("gyro_pitch") > 1.0
    )
    if args.require_motion and not moved:
        print("Aucun mouvement detecte dans les paquets.", file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
