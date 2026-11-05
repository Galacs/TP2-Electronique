#!/usr/bin/env python3
"""Pont entre le BMA220 (Arduino, liaison serie) et le protocole DSU.

Le programme lit l'orientation envoyee par l'Arduino sur le port serie,
la lisse, puis la publie en continu sur le port UDP 26760 au format DSU
(« cemuhook »). Dolphin, Cemu, Yuzu/Ryujinx et la plupart des emulateurs
savent lire ce protocole : la carte devient une manette avec capteur de
mouvement.

Exemples :
    python3 pc/dsu_bridge.py --list-ports
    python3 pc/dsu_bridge.py                       # detection auto du port
    python3 pc/dsu_bridge.py --calibrate           # centrage au demarrage
    python3 pc/dsu_bridge.py --simulate            # sans Arduino
    python3 pc/dsu_bridge.py --stick --range 30    # inclinaison -> stick gauche

Voir pc/README.md pour la configuration cote emulateur.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import re
import select
import socket
import sys
import threading
import time
from dataclasses import dataclass

import dsu_protocol as dsu

LOG = logging.getLogger("dsu")

DEFAULT_UDP_PORT = 26760
DEFAULT_BAUD = 115200
DEFAULT_MAC = "02:00:00:00:00:01"

GYRO_INDEX = {"pitch": 0, "yaw": 1, "roll": 2}

# Une ligne de la variante "dsu_orientation" du sketch Arduino.
RE_DSU = re.compile(
    r"^DSU\s*[,;]\s*(-?\d+(?:\.\d+)?)\s*[,;]\s*(-?\d+(?:\.\d+)?)\s*[,;]\s*(-?\d+(?:\.\d+)?)"
)
# Ancien format du TP : "x = 0.12 gy = -0.03 gz = 0.98 g".
RE_LEGACY = re.compile(
    r"x\s*=\s*(-?\d+(?:\.\d+)?)\s+gy\s*=\s*(-?\d+(?:\.\d+)?)\s+gz\s*=\s*(-?\d+(?:\.\d+)?)"
)


# --------------------------------------------------------------------------
# Acquisition
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Sample:
    """Un echantillon brut de l'accelerometre (en g)."""

    ax: float
    ay: float
    az: float
    received_at: float


@dataclass(frozen=True)
class TiltState:
    """Etat filtre, pret a etre envoye."""

    ax: float
    ay: float
    az: float
    angle: float
    rate: float
    stick: float
    fresh: bool


def tilt_angles(sample: Sample) -> tuple[float, float]:
    """Angles (pitch, roll) en degres a partir du vecteur de gravite."""
    pitch = math.degrees(math.atan2(sample.ax, sample.az))
    roll = math.degrees(math.atan2(sample.ay, sample.az))
    return pitch, roll


class TiltPipeline:
    """Lissage, centrage, zone morte et mise a l'echelle en stick.

    Le filtrage est fait a l'arrivee des echantillons (et non a la cadence
    d'envoi) pour ne pas dependre de la frequence serie.
    """

    def __init__(
        self,
        *,
        axis: str = "pitch",
        invert: bool = False,
        center: float = 0.0,
        full_scale: float = 35.0,
        deadzone: float = 3.0,
        expo: float = 0.3,
        smooth_ms: float = 80.0,
        stale_timeout: float = 1.0,
    ) -> None:
        self.axis = axis
        self.invert = invert
        self.center = center
        self.full_scale = max(full_scale, deadzone + 0.5)
        self.deadzone = max(0.0, deadzone)
        self.expo = min(1.0, max(0.0, expo))
        self.smooth_ms = max(0.0, smooth_ms)
        self.stale_timeout = stale_timeout

        self._lock = threading.Lock()
        self._angle: float | None = None
        self._rate = 0.0
        self._prev_raw: float | None = None
        self._prev_t: float | None = None
        self._last_sample: Sample | None = None
        self._last_sample_at = 0.0
        self.samples = 0

    # -- appels depuis le thread d'acquisition -----------------------------
    def feed(self, sample: Sample) -> None:
        pitch, roll = tilt_angles(sample)
        raw = pitch if self.axis == "pitch" else roll
        if self.invert:
            raw = -raw

        with self._lock:
            now = sample.received_at
            dt = 0.0 if self._prev_t is None else max(1e-3, min(0.25, now - self._prev_t))

            if self._angle is None:
                self._angle = raw
            if dt > 0 and self.smooth_ms > 0:
                tau = self.smooth_ms / 1000.0
                alpha = 1.0 - math.exp(-dt / tau)
                self._angle += alpha * (raw - self._angle)
            elif dt > 0:
                self._angle = raw

            if self._prev_raw is not None and dt > 0:
                instant = (raw - self._prev_raw) / dt
                instant = max(-2000.0, min(2000.0, instant))
                tau = max(0.06, self.smooth_ms / 2000.0)
                alpha = 1.0 - math.exp(-dt / tau)
                self._rate += alpha * (instant - self._rate)

            self._prev_raw = raw
            self._prev_t = now
            self._last_sample = sample
            self._last_sample_at = now
            self.samples += 1

    def set_center(self, center: float) -> None:
        with self._lock:
            self.center = center

    def latest_sample(self) -> Sample | None:
        with self._lock:
            return self._last_sample

    # -- lecture depuis le thread principal --------------------------------
    def snapshot(self) -> TiltState:
        now = time.monotonic()
        with self._lock:
            fresh = (
                self._last_sample is not None and (now - self._last_sample_at) <= self.stale_timeout
            )
            if not fresh or self._last_sample is None or self._angle is None:
                return TiltState(0.0, 0.0, 1.0, self.center, 0.0, 0.0, False)

            sample = self._last_sample
            angle = self._angle
            rate = 0.0 if not fresh else self._rate

        delta = angle - self.center
        if abs(delta) <= self.deadzone:
            norm = 0.0
        else:
            span = max(0.5, self.full_scale - self.deadzone)
            norm = math.copysign(min(1.0, (abs(delta) - self.deadzone) / span), delta)
        # Courbe d'expo : plus de precision pres du centre, course complete aux extremes.
        norm = math.copysign((1.0 - self.expo) * abs(norm) + self.expo * abs(norm) ** 3, norm)
        return TiltState(sample.ax, sample.ay, sample.az, angle, rate, norm, True)


class SimulatedSource(threading.Thread):
    """Source de secours : une inclinaison sinusoidale, sans materiel."""

    def __init__(self, pipeline: TiltPipeline, stop_event: threading.Event) -> None:
        super().__init__(name="simulation", daemon=True)
        self.pipeline = pipeline
        self.stop_event = stop_event

    def run(self) -> None:
        start = time.monotonic()
        while not self.stop_event.is_set():
            t = time.monotonic() - start
            pitch = math.radians(25.0 * math.sin(2 * math.pi * 0.35 * t))
            roll = math.radians(15.0 * math.sin(2 * math.pi * 0.23 * t + 1.0))
            ax = math.sin(pitch)
            ay = math.sin(roll) * math.cos(pitch)
            az = math.cos(pitch) * math.cos(roll)
            self.pipeline.feed(Sample(ax, ay, az, time.monotonic()))
            self.stop_event.wait(0.02)

    def stop(self) -> None:
        # Le thread est deja arrete par stop_event ; rien a fermer.
        pass


class SerialSource(threading.Thread):
    """Lecture du port serie, avec reconnexion automatique."""

    def __init__(
        self,
        pipeline: TiltPipeline,
        stop_event: threading.Event,
        port: str | None = None,
        baud: int = DEFAULT_BAUD,
    ) -> None:
        super().__init__(name="serie", daemon=True)
        self.pipeline = pipeline
        self.stop_event = stop_event
        self.port = port
        self.baud = baud
        self.lines = 0
        self.error: str | None = None
        self._serial = None
        self._permission_warned = False

    # -- detection du port -------------------------------------------------
    @staticmethod
    def list_ports() -> list[tuple[str, str]]:
        try:
            from serial.tools import list_ports as lp
        except ImportError as exc:  # pragma: no cover - depend de l'environnement
            raise SystemExit(
                "pyserial est manquant : installe-le avec « python3 -m pip install pyserial »."
            ) from exc
        return [(p.device, p.description or "") for p in lp.comports()]

    def _autodetect(self) -> str | None:
        ports = self.list_ports()
        if not ports:
            return None
        hints = ("arduino", "ch340", "ch341", "cp210", "ftdi", "usb serial", "usb-serial")
        preferred = [dev for dev, desc in ports if any(h in desc.lower() for h in hints)]
        if preferred:
            return preferred[0]
        acm = [dev for dev, _ in ports if "/ttyACM" in dev or "/ttyUSB" in dev]
        if acm:
            return acm[0]
        return ports[0][0] if len(ports) == 1 else None

    # -- boucle ------------------------------------------------------------
    def run(self) -> None:
        try:
            import serial  # noqa: F401  (import paresseux : mode simulation sans pyserial)
        except ImportError:
            self.error = "pyserial manquant"
            LOG.error("pyserial est manquant : python3 -m pip install pyserial")
            return

        buffer = b""
        while not self.stop_event.is_set():
            if self._serial is None:
                port = self.port or self._autodetect()
                if port is None:
                    self.error = "aucun port serie detecte"
                    self.stop_event.wait(1.0)
                    continue
                try:
                    self._serial = serial.Serial(port, self.baud, timeout=0.2)
                    self.error = None
                    LOG.info("port serie ouvert : %s (%d bauds)", port, self.baud)
                except Exception as exc:  # serial.SerialException et erreurs systeme
                    self.error = str(exc)
                    if "Permission denied" in str(exc) and not self._permission_warned:
                        self._permission_warned = True
                        LOG.error(
                            "acces refuse a %s : ajoute ton utilisateur au groupe serie "
                            "(Arch : uucp, Debian/Ubuntu : dialout) puis reconnecte-toi.",
                            port,
                        )
                    else:
                        LOG.debug("ouverture de %s impossible : %s", port, exc)
                    self.stop_event.wait(1.0)
                    continue

            try:
                chunk = self._serial.read(256)
            except Exception as exc:
                if not self.stop_event.is_set():
                    LOG.warning("lecture serie interrompue : %s", exc)
                self._close()
                self.stop_event.wait(0.5)
                continue

            if not chunk:
                continue
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self._handle_line(line.decode("ascii", "replace").strip())
            if len(buffer) > 4096:  # ligne absurde : on jette
                buffer = b""

    def _handle_line(self, line: str) -> None:
        match = RE_DSU.match(line) or RE_LEGACY.search(line)
        if not match:
            return
        try:
            ax, ay, az = (float(match.group(i)) for i in (1, 2, 3))
        except ValueError:
            return
        self.lines += 1
        if self.lines == 1:
            LOG.info("premiere mesure recue : %s", line)
        self.pipeline.feed(Sample(ax, ay, az, time.monotonic()))

    def status(self) -> str:
        if self._serial is not None:
            name = self._serial.port
            return f"{name} ({self.lines} mesures)"
        return self.error or "connexion..."

    def _close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def stop(self) -> None:
        self._close()


# --------------------------------------------------------------------------
# Serveur DSU
# --------------------------------------------------------------------------


@dataclass
class _Client:
    addr: tuple[str, int]
    slot: int
    last_seen: float


class DSUServer:
    """Serveur UDP DSU : repond aux clients puis streame l'etat de la manette."""

    def __init__(
        self,
        pipeline: TiltPipeline,
        *,
        host: str = "0.0.0.0",
        port: int = DEFAULT_UDP_PORT,
        slot: int = 0,
        mac: bytes = bytes(6),
        model: int = dsu.MODEL_FULL_GYRO,
        battery: int = dsu.BATTERY_FULL,
        rate: float = 60.0,
        stick: bool = False,
        stick_gain: float = 1.0,
        gyro_axis: str = "roll",
        accel_signs: tuple[float, float, float] = (1.0, 1.0, 1.0),
        client_timeout: float = 5.0,
    ) -> None:
        self.pipeline = pipeline
        self.slot = slot
        self.mac = mac
        self.model = model
        self.battery = battery
        self.rate = max(1.0, rate)
        self.stick = stick
        self.stick_gain = stick_gain
        self.gyro_axis = gyro_axis
        self.accel_signs = accel_signs
        self.client_timeout = client_timeout

        self.source_id = int.from_bytes(os.urandom(4), "little") or 1
        self.counter = 0
        self.packets = 0
        self.clients: dict[tuple[str, int], _Client] = {}

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.setblocking(False)
        self.port = port
        self.host = host

    # -- requetes ----------------------------------------------------------
    def _handle(self, data: bytes, addr: tuple[str, int], now: float) -> None:
        request = dsu.parse_request(data)
        if request is None:
            return

        if request.msg_type == dsu.MSG_VERSION:
            self._send(dsu.build_version_response(self.source_id), addr)

        elif request.msg_type == dsu.MSG_PORTS:
            self._handle_list_ports(request, addr)

        elif request.msg_type == dsu.MSG_DATA:
            self._handle_data_request(request, addr, now)

    def _handle_list_ports(self, request: dsu.Request, addr: tuple[str, int]) -> None:
        payload = request.payload
        count = 0
        ids: list[int] = []
        if len(payload) >= 4:
            count = int.from_bytes(payload[0:4], "little", signed=True)
            count = max(0, min(count, 4))
            ids = list(payload[4 : 4 + count])
        if not ids:
            ids = [self.slot]
        if self.slot in ids:
            self._send(
                dsu.build_port_info(
                    self.source_id,
                    slot=self.slot,
                    state=dsu.STATE_CONNECTED,
                    model=self.model,
                    connection=dsu.CONNECTION_USB,
                    mac=self.mac,
                    battery=self.battery,
                ),
                addr,
            )

    def _handle_data_request(
        self, request: dsu.Request, addr: tuple[str, int], now: float
    ) -> None:
        flags = request.register_flags
        if flags == 1 and request.requested_slot != self.slot:
            return
        if flags == 2 and request.requested_mac != self.mac:
            return

        self.clients[addr] = _Client(addr=addr, slot=request.requested_slot, last_seen=now)
        self._send(self._build_packet(now), addr)

    # -- emission ----------------------------------------------------------
    def _build_packet(self, now: float) -> bytes:
        state = self.pipeline.snapshot()
        ax, ay, az = state.ax, state.ay, state.az
        gyro = [0.0, 0.0, 0.0]

        if state.fresh:
            ax *= self.accel_signs[0]
            ay *= self.accel_signs[1]
            az *= self.accel_signs[2]
            if self.gyro_axis in GYRO_INDEX:
                gyro[GYRO_INDEX[self.gyro_axis]] = state.rate
        else:
            ax, ay, az = 0.0, 0.0, 1.0

        stick_x = state.stick * self.stick_gain if (self.stick and state.fresh) else 0.0

        pad = dsu.PadState(
            slot=self.slot,
            state=dsu.STATE_CONNECTED,
            model=self.model,
            connection=dsu.CONNECTION_USB,
            mac=self.mac,
            battery=self.battery,
            active=1,
            counter=self.counter,
            timestamp_us=time.monotonic_ns() // 1000,
            left_stick=(stick_x, 0.0),
            accel=(ax, ay, az),
            gyro=(gyro[0], gyro[1], gyro[2]),
        )
        self.counter = (self.counter + 1) & 0xFFFFFFFF
        return pad.to_packet(self.source_id)

    def _send(self, packet: bytes, addr: tuple[str, int]) -> None:
        try:
            self.socket.sendto(packet, addr)
            self.packets += 1
        except OSError as exc:  # client parti, buffer plein...
            LOG.debug("envoi vers %s impossible : %s", addr, exc)

    def _send_all(self, now: float) -> None:
        expired = [a for a, c in self.clients.items() if now - c.last_seen > self.client_timeout]
        for addr in expired:
            del self.clients[addr]
            LOG.info("client DSU parti : %s:%d", *addr)
        if not self.clients:
            return
        packet = self._build_packet(now)
        for addr in list(self.clients):
            self._send(packet, addr)

    # -- boucle principale -------------------------------------------------
    def serve(self, stop_event: threading.Event, status=None) -> None:
        period = 1.0 / self.rate
        next_tick = time.perf_counter()
        next_status = time.monotonic() + 0.5

        while not stop_event.is_set():
            timeout = max(0.0, next_tick - time.perf_counter())
            try:
                ready, _, _ = select.select([self.socket], [], [], min(timeout, 0.1))
            except (OSError, ValueError):
                break

            if ready:
                while True:
                    try:
                        data, addr = self.socket.recvfrom(2048)
                    except BlockingIOError:
                        break
                    except OSError:
                        return
                    self._handle(data, addr, time.monotonic())

            now = time.perf_counter()
            if now >= next_tick:
                self._send_all(time.monotonic())
                next_tick += period
                if next_tick < now - 0.5:  # on a pris du retard : on recale
                    next_tick = now + period

            if status is not None and time.monotonic() >= next_status:
                next_status = time.monotonic() + 0.5
                status(self)

        self.socket.close()


# --------------------------------------------------------------------------
# Ligne de commande
# --------------------------------------------------------------------------


def _parse_signs(value: str) -> tuple[float, float, float]:
    parts = value.replace(";", ",").split(",")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("format attendu : 1,1,1")
    signs = tuple(-1.0 if float(p) < 0 else 1.0 for p in parts)
    return signs  # type: ignore[return-value]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publie l'orientation du BMA220 (Arduino) au format DSU/cemuhook.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Exemples :\n"
            "  python3 pc/dsu_bridge.py --list-ports\n"
            "  python3 pc/dsu_bridge.py --calibrate\n"
            "  python3 pc/dsu_bridge.py --simulate --console\n"
            "  python3 pc/dsu_bridge.py --stick --range 30 --deadzone 4\n"
        ),
    )
    parser.add_argument("-s", "--serial", help="port serie (defaut : detection automatique)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="vitesse serie (115200)")
    parser.add_argument("--list-ports", action="store_true", help="liste les ports serie et quitte")

    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT, help="port UDP DSU")
    parser.add_argument("--bind", default="0.0.0.0", help="adresse d'ecoute (0.0.0.0 = toutes)")
    parser.add_argument("--rate", type=float, default=60.0, help="paquets DSU par seconde (60)")
    parser.add_argument("--slot", type=int, choices=range(4), default=0, help="emplacement DSU (0)")

    parser.add_argument("--axis", choices=("pitch", "roll"), default="pitch",
                        help="axe d'inclinaison a utiliser (pitch = atan2(ax, az))")
    parser.add_argument("--invert", action="store_true", help="inverse le sens de l'inclinaison")
    parser.add_argument("--center", type=float, help="angle de repos en degres (defaut : 0)")
    parser.add_argument("--calibrate", action="store_true",
                        help="mesure la position de repos au demarrage")
    parser.add_argument("--calibrate-seconds", type=float, default=2.0,
                        help="duree de la calibration (2 s)")

    parser.add_argument("--range", dest="range_deg", type=float, default=35.0,
                        help="angle d'inclinaison donnant la course complete (35 deg)")
    parser.add_argument("--deadzone", type=float, default=3.0, help="zone morte en degres (3)")
    parser.add_argument("--expo", type=float, default=0.3, help="courbe d'expo 0..1 (0.3)")
    parser.add_argument("--smooth", type=float, default=80.0, help="lissage en ms (80)")

    parser.add_argument("--stick", action="store_true",
                        help="mappe aussi l'inclinaison sur le stick gauche (X)")
    parser.add_argument("--stick-gain", type=float, default=1.0, help="gain du stick (1.0)")

    parser.add_argument("--gyro-axis", choices=("none", "pitch", "yaw", "roll"), default="roll",
                        help="axe du gyroscope derive (roll)")
    parser.add_argument("--accel-signs", type=_parse_signs, default=(1.0, 1.0, 1.0),
                        help="signes des axes accelerometre, ex. 1,-1,1")
    parser.add_argument("--mac", default=DEFAULT_MAC, help="adresse MAC annoncee")
    parser.add_argument("--model", type=int, choices=(1, 2), default=2,
                        help="1 = gyro partiel, 2 = gyro complet")

    parser.add_argument("--simulate", action="store_true", help="genere une inclinaison fictive")
    parser.add_argument("--console", dest="console", action="store_true", default=None,
                        help="affiche l'etat en continu")
    parser.add_argument("--no-console", dest="console", action="store_false")
    parser.add_argument("-v", "--verbose", action="count", default=0, help="bavard (-v, -vv)")
    return parser


def _calibrate(pipeline: TiltPipeline, seconds: float) -> float | None:
    print(f"Calibration : ne bouge pas la carte pendant {seconds:.1f} s...", flush=True)
    # Ouvrir le port serie reinitialise l'Arduino : il met ~2 s a redemarrer.
    wait_until = time.monotonic() + 5.0
    while pipeline.latest_sample() is None and time.monotonic() < wait_until:
        time.sleep(0.02)
    if pipeline.latest_sample() is None:
        print("Calibration impossible : aucune mesure serie recue.", file=sys.stderr)
        return None
    start = time.monotonic()
    values: list[float] = []
    last: Sample | None = None
    while time.monotonic() - start < seconds:
        sample = pipeline.latest_sample()
        if sample is not None and sample is not last:
            last = sample
            pitch, roll = tilt_angles(sample)
            values.append(pitch if pipeline.axis == "pitch" else roll)
        time.sleep(0.01)
    if not values:
        print("Calibration impossible : aucune mesure serie recue.", file=sys.stderr)
        return None
    center = sum(values) / len(values)
    # feed() inverse deja l'angle quand --invert est actif : le centre doit
    # donc etre exprime dans le meme repere.
    return -center if pipeline.invert else center


def _status_printer(server: DSUServer, source, pipeline: TiltPipeline):
    def status(_server: DSUServer) -> None:
        state = pipeline.snapshot()
        if state.fresh:
            angle = state.angle
            stick = state.stick
            text = f"angle {angle:+7.1f} deg  stick {stick:+.2f}"
        else:
            text = "angle    n/a      stick  0.00  (pas de donnees)"
        line = f"{text}  clients {len(server.clients)}  paquets {server.packets}"
        if isinstance(source, SerialSource):
            line += f"  serie {source.status()}"
        sys.stdout.write("\r" + line.ljust(100)[:100])
        sys.stdout.flush()

    return status


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    level = logging.WARNING
    if args.verbose == 1:
        level = logging.INFO
    elif args.verbose >= 2:
        level = logging.DEBUG
    logging.basicConfig(level=level, format="%(levelname)s: %(message)s")

    if args.list_ports:
        ports = SerialSource.list_ports()
        if not ports:
            print("Aucun port serie detecte.")
        for device, description in ports:
            print(f"{device:20s} {description}")
        return 0

    if not (0.0 <= args.expo <= 1.0):
        print("--expo doit etre entre 0 et 1", file=sys.stderr)
        return 2
    if args.range_deg <= args.deadzone:
        print("--range doit etre superieur a --deadzone", file=sys.stderr)
        return 2
    if args.rate < 10 or args.rate > 200:
        print("--rate doit etre entre 10 et 200", file=sys.stderr)
        return 2

    try:
        mac = dsu.mac_to_bytes(args.mac)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    pipeline = TiltPipeline(
        axis=args.axis,
        invert=args.invert,
        center=args.center or 0.0,
        full_scale=args.range_deg,
        deadzone=args.deadzone,
        expo=args.expo,
        smooth_ms=args.smooth,
    )

    stop_event = threading.Event()
    if args.simulate:
        source = SimulatedSource(pipeline, stop_event)
    else:
        source = SerialSource(pipeline, stop_event, args.serial, args.baud)

    try:
        server = DSUServer(
            pipeline,
            host=args.bind,
            port=args.udp_port,
            slot=args.slot,
            mac=mac,
            model=dsu.MODEL_FULL_GYRO if args.model == 2 else dsu.MODEL_PARTIAL_GYRO,
            rate=args.rate,
            stick=args.stick,
            stick_gain=args.stick_gain,
            gyro_axis=args.gyro_axis,
            accel_signs=args.accel_signs,
        )
    except OSError as exc:
        print(f"Impossible d'ecouter sur le port {args.udp_port} : {exc}", file=sys.stderr)
        print("Un autre serveur DSU (DS4Windows, BetterJoy...) tourne peut-etre deja.",
              file=sys.stderr)
        return 1

    source.start()
    print(
        f"Pont DSU actif sur udp://{args.bind}:{args.udp_port} (slot {args.slot}, "
        f"{args.rate:.0f} Hz). Ctrl+C pour arreter.",
        flush=True,
    )
    print(f"Cote emulateur : ajouter un serveur DSU 127.0.0.1:{args.udp_port}.", flush=True)
    if args.simulate:
        print("Mode simulation : aucune carte n'est necessaire.", flush=True)

    if args.calibrate:
        center = _calibrate(pipeline, args.calibrate_seconds)
        if center is None:
            source.stop()
            stop_event.set()
            return 1
        pipeline.set_center(center)
        print(f"Centre applique : {center:+.1f} deg", flush=True)

    console = args.console
    if console is None:
        console = sys.stdout.isatty()

    status = _status_printer(server, source, pipeline) if console else None
    try:
        server.serve(stop_event, status)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        source.stop()
        if console:
            sys.stdout.write("\n")
        print(f"Arrete ({server.packets} paquets envoyes).", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
