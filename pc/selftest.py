#!/usr/bin/env python3
"""Auto-test du pont DSU (aucun materiel necessaire).

    python3 pc/selftest.py

Verifie l'encodage des paquets (tailles et CRC attendus par Dolphin), le
traitement de l'inclinaison, puis fait tourner le pont en mode simulation
et un client DSU de bout en bout.
"""

from __future__ import annotations

import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import dsu_bridge  # noqa: E402
import dsu_protocol as dsu  # noqa: E402

FAILURES: list[str] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    print(f"[{'OK  ' if condition else 'FAIL'}] {name}" + (f"  ({detail})" if detail and not condition else ""))
    if not condition:
        FAILURES.append(name)


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def test_protocol() -> None:
    pad = dsu.PadState(
        counter=7,
        timestamp_us=1234,
        accel=(0.1, -0.2, 0.9),
        gyro=(1.0, 2.0, 3.0),
        left_stick=(0.5, -0.5),
    ).to_packet(0xDEADBEEF)
    check("paquet de donnees = 100 octets", len(pad) == 100, str(len(pad)))
    check("taille PortInfo = 32 octets", len(dsu.build_port_info(1)) == 32)
    check("taille VersionRsp = 24 octets", len(dsu.build_version_response(1)) == 24)
    check("CRC32 valide", dsu.crc_is_valid(pad))
    check("magic serveur", pad[:4] == b"DSUS")
    check("type donnees", struct.unpack_from("<I", pad, 16)[0] == dsu.MSG_DATA)

    ax, ay, az = struct.unpack_from("<fff", pad, 20 + 56)
    check(
        "accel aux bons offsets",
        abs(ax - 0.1) < 1e-6 and abs(ay + 0.2) < 1e-6 and abs(az - 0.9) < 1e-6,
        f"{ax},{ay},{az}",
    )
    check("stick 0.5 -> 192", pad[20 + 20] == 192, str(pad[20 + 20]))

    request = dsu.parse_request(
        dsu.build_client_message(1, dsu.MSG_DATA, struct.pack("<BB6s", 1, 0, bytes(6)))
    )
    check(
        "requete client analysee",
        request is not None and request.register_flags == 1 and request.requested_slot == 0,
    )

    check(
        "format DSU reconnu",
        bool(dsu_bridge.RE_DSU.match("DSU,0.0625,-0.0625,0.9961,3.58")),
    )
    check(
        "ancien format du TP reconnu",
        bool(dsu_bridge.RE_LEGACY.search("x = 0.12 gy = -0.03 gz = 0.98 g")),
    )


def test_pipeline() -> None:
    pipeline = dsu_bridge.TiltPipeline(full_scale=30.0, deadzone=2.0, expo=0.0, smooth_ms=0.0)
    start = time.monotonic()
    pipeline.feed(dsu_bridge.Sample(0.0, 0.0, 1.0, start))
    pipeline.feed(
        dsu_bridge.Sample(
            math.sin(math.radians(45)), 0.0, math.cos(math.radians(45)), start + 0.05
        )
    )
    state = pipeline.snapshot()
    check("45 deg -> stick sature", abs(state.stick) > 0.99, f"stick={state.stick:.3f}")

    dead = dsu_bridge.TiltPipeline(full_scale=30.0, deadzone=5.0, expo=0.0, smooth_ms=0.0)
    dead.feed(dsu_bridge.Sample(0.0, 0.0, 1.0, start))
    dead.feed(
        dsu_bridge.Sample(
            math.sin(math.radians(3)), 0.0, math.cos(math.radians(3)), start + 0.05
        )
    )
    check("zone morte a 3 deg", abs(dead.snapshot().stick) < 1e-6)

    stale = dsu_bridge.TiltPipeline(stale_timeout=0.01)
    stale.feed(dsu_bridge.Sample(0.0, 0.0, 1.0, time.monotonic()))
    time.sleep(0.05)
    check("donnees obsoletes -> neutre", not stale.snapshot().fresh)


def test_end_to_end() -> None:
    port = free_udp_port()
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    bridge = subprocess.Popen(
        [
            sys.executable,
            str(HERE / "dsu_bridge.py"),
            "--simulate",
            "--udp-port",
            str(port),
            "--no-console",
            "--stick",
            "--rate",
            "80",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    try:
        time.sleep(0.6)
        result = None
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            result = subprocess.run(
                [
                    sys.executable,
                    str(HERE / "dsu_probe.py"),
                    "--port",
                    str(port),
                    "--duration",
                    "2",
                    "--json",
                    "--require-motion",
                ],
                capture_output=True,
                text=True,
                timeout=20,
            )
            if result.returncode == 0:
                break
            time.sleep(0.4)

        assert result is not None
        check("le client recoit les donnees", result.returncode == 0, (result.stderr or "").strip())
        packets = [json.loads(line) for line in result.stdout.splitlines() if line.startswith("{")]
        check("debit suffisant (~80 Hz)", len(packets) >= 80, f"{len(packets)} paquets")
        if packets:
            xs = [p["ax"] for p in packets]
            check("l'accelerometre varie", max(xs) - min(xs) > 0.2, f"amplitude {max(xs) - min(xs):.2f}")
            sticks = [p["left_x"] for p in packets]
            check("le stick varie", max(sticks) - min(sticks) > 0.4)
            check(
                "gyro derive non nul",
                max(abs(p["gyro_roll"]) for p in packets) > 5.0,
            )
    finally:
        bridge.terminate()
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()


def test_serial_legacy_format() -> None:
    """Simule un Arduino sur un pseudo-terminal, avec l'ancien format du TP."""
    if not hasattr(os, "openpty"):
        return
    import pty

    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    port = free_udp_port()
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    bridge = subprocess.Popen(
        [
            sys.executable,
            str(HERE / "dsu_bridge.py"),
            "--serial",
            slave_name,
            "--udp-port",
            str(port),
            "--no-console",
            "--rate",
            "60",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
    )
    stop = threading.Event()

    def writer() -> None:
        start = time.monotonic()
        while not stop.is_set():
            t = time.monotonic() - start
            angle = math.radians(20 * math.sin(2 * math.pi * 0.5 * t))
            ax, az = math.sin(angle), math.cos(angle)
            os.write(master, f"x = {ax:.2f} gy = 0.00 gz = {az:.2f} g\n".encode())
            time.sleep(0.02)

    thread = threading.Thread(target=writer, daemon=True)
    thread.start()
    try:
        time.sleep(0.8)
        result = subprocess.run(
            [
                sys.executable,
                str(HERE / "dsu_probe.py"),
                "--port",
                str(port),
                "--duration",
                "2",
                "--json",
            ],
            capture_output=True,
            text=True,
            timeout=20,
        )
        check("l'ancien format du TP est lu", result.returncode == 0, result.stderr.strip())
        packets = [json.loads(line) for line in result.stdout.splitlines() if line.startswith("{")]
        check("paquets recus via le port serie", len(packets) >= 60, f"{len(packets)} paquets")
        if packets:
            xs = [p["ax"] for p in packets]
            check("l'inclinaison serie est transmise", max(xs) - min(xs) > 0.2)
    finally:
        stop.set()
        bridge.terminate()
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()
        os.close(master)
        os.close(slave)


def main() -> int:
    test_protocol()
    test_pipeline()
    test_end_to_end()
    test_serial_legacy_format()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} test(s) en echec : {', '.join(FAILURES)}")
        return 1
    print("Tous les tests passent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
