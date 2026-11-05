"""Protocole DSU (dit « cemuhook ») : encodage des paquets.

Le protocole est decrit ici : https://v1993.github.io/cemuhook-protocol/
et c'est celui qu'utilisent Dolphin, Cemu, Yuzu/Ryujinx, DS4Windows, etc.

Resume :
  - UDP, port 26760 par defaut ;
  - en-tete de 16 octets : "DSUS" cote serveur / "DSUC" cote client,
    version 1001, taille (hors en-tete), CRC32, identifiant, puis le type ;
  - le serveur repond a trois types de requetes : version, liste des
    manettes, et abonnement aux donnees (puis il streame).

Ce module ne depend que de la bibliotheque standard.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field

MAGIC_SERVER = b"DSUS"
MAGIC_CLIENT = b"DSUC"

PROTOCOL_VERSION = 1001

MSG_VERSION = 0x100000
MSG_PORTS = 0x100001
MSG_DATA = 0x100002

# Etats / modeles / connexions (valeurs du protocole).
STATE_DISCONNECTED = 0
STATE_RESERVED = 1
STATE_CONNECTED = 2

MODEL_NONE = 0
MODEL_PARTIAL_GYRO = 1
MODEL_FULL_GYRO = 2

CONNECTION_NONE = 0
CONNECTION_USB = 1
CONNECTION_BLUETOOTH = 2

BATTERY_NONE = 0x00
BATTERY_FULL = 0x05
BATTERY_CHARGING = 0xEE

# Masques des boutons « bruts » (disposition DualShock 4 du protocole).
# Dolphin n'utilise que les champs analogiques, mais d'autres clients
# (Yuzu, Ryujinx...) lisent ces masques.
BUTTON1_BITS = {
    "share": 0x01,
    "l3": 0x02,
    "r3": 0x04,
    "options": 0x08,
    "left": 0x10,
    "down": 0x20,
    "right": 0x40,
    "up": 0x80,
}
BUTTON2_BITS = {
    "triangle": 0x01,
    "circle": 0x02,
    "cross": 0x04,
    "square": 0x08,
    "r1": 0x10,
    "l1": 0x20,
    "r2": 0x40,
    "l2": 0x80,
}

HEADER = struct.Struct("<4sHHII")  # 16 octets
U16 = struct.Struct("<H")
U32 = struct.Struct("<I")
F32 = struct.Struct("<f")

HEADER_SIZE = HEADER.size
VERSION_PACKET_SIZE = HEADER_SIZE + 4 + 4  # + type + u16 + 2 octets de bourrage
PORT_INFO_PACKET_SIZE = HEADER_SIZE + 4 + 12
PAD_DATA_PACKET_SIZE = HEADER_SIZE + 4 + 80  # 100 octets au total


def mac_to_bytes(mac: str | bytes) -> bytes:
    """Convertit "aa:bb:cc:dd:ee:ff" en 6 octets (ou valide des octets)."""
    if isinstance(mac, (bytes, bytearray)):
        data = bytes(mac)
        if len(data) != 6:
            raise ValueError("une adresse MAC fait 6 octets")
        return data
    parts = mac.replace("-", ":").split(":")
    if len(parts) != 6:
        raise ValueError(f"adresse MAC invalide : {mac!r}")
    return bytes(int(p, 16) for p in parts)


def _seal(buffer: bytearray) -> bytes:
    """Calcule le CRC32 du paquet (champ CRC mis a zero) et le renvoie."""
    struct.pack_into("<I", buffer, 8, 0)
    struct.pack_into("<I", buffer, 8, zlib.crc32(buffer) & 0xFFFFFFFF)
    return bytes(buffer)


def _build(magic: bytes, source_id: int, msg_type: int, payload: bytes) -> bytes:
    buffer = bytearray(
        HEADER.pack(magic, PROTOCOL_VERSION, 4 + len(payload), 0, source_id & 0xFFFFFFFF)
    )
    buffer += U32.pack(msg_type)
    buffer += payload
    return _seal(buffer)


def build_message(source_id: int, msg_type: int, payload: bytes = b"") -> bytes:
    """Construit un paquet serveur complet (en-tete + type + charge + CRC)."""
    return _build(MAGIC_SERVER, source_id, msg_type, payload)


def build_client_message(source_id: int, msg_type: int, payload: bytes = b"") -> bytes:
    """Construit un paquet client (magic "DSUC"), utile pour les tests."""
    return _build(MAGIC_CLIENT, source_id, msg_type, payload)


def build_version_response(source_id: int) -> bytes:
    """Reponse a une demande de version : 24 octets attendus par Dolphin."""
    return build_message(source_id, MSG_VERSION, U16.pack(PROTOCOL_VERSION) + b"\x00\x00")


def build_port_info(
    source_id: int,
    *,
    slot: int = 0,
    state: int = STATE_CONNECTED,
    model: int = MODEL_FULL_GYRO,
    connection: int = CONNECTION_USB,
    mac: bytes = bytes(6),
    battery: int = BATTERY_FULL,
) -> bytes:
    """Information sur une manette (reponse a une demande de liste)."""
    payload = struct.pack("<BBBB6sBB", slot, state, model, connection, mac, battery, 0)
    return build_message(source_id, MSG_PORTS, payload)


@dataclass
class PadState:
    """Etat complet d'une manette, tel qu'envoye dans un paquet de donnees."""

    slot: int = 0
    state: int = STATE_CONNECTED
    model: int = MODEL_FULL_GYRO
    connection: int = CONNECTION_USB
    mac: bytes = bytes(6)
    battery: int = BATTERY_FULL
    active: int = 0
    counter: int = 0
    timestamp_us: int = 0

    # Valeurs normalisees : -1.0 .. +1.0 (0 = repos).
    left_stick: tuple[float, float] = (0.0, 0.0)
    right_stick: tuple[float, float] = (0.0, 0.0)

    # Accelerometre en g, gyroscope en deg/s.
    accel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Noms de boutons appuyes (voir BUTTON1_BITS / BUTTON2_BITS).
    pressed: frozenset[str] = field(default_factory=frozenset)

    def to_packet(self, source_id: int) -> bytes:
        """Encode le paquet de donnees (100 octets, format cemuhook)."""
        b1 = 0
        b2 = 0
        for name in self.pressed:
            b1 |= BUTTON1_BITS.get(name, 0)
            b2 |= BUTTON2_BITS.get(name, 0)

        analog = {
            name: 255 if name in self.pressed else 0
            for name in ("square", "cross", "circle", "triangle", "l1", "r1", "l2", "r2")
        }
        dpad = {name: 255 if name in self.pressed else 0 for name in ("left", "down", "right", "up")}

        lx, ly = self.left_stick
        rx, ry = self.right_stick

        payload = struct.pack(
            "<BBBB6sBBI"  # etat + numero de paquet (16 octets)
            "BBBB"  # boutons bruts, PS, touche tactile
            "BBBB"  # sticks gauche/droite
            "BBBB"  # croix directionnelle analogique
            "BBBBBBBB"  # boutons analogiques
            "6s6s"  # deux zones tactiles (inactives)
            "Q"  # horodatage des donnees de mouvement (us)
            "fff"  # accelerometre (g)
            "fff",  # gyroscope (deg/s)
            self.slot,
            self.state,
            self.model,
            self.connection,
            self.mac,
            self.battery,
            self.active,
            self.counter & 0xFFFFFFFF,
            b1,
            b2,
            0,
            0,
            _axis(lx),
            _axis(ly),
            _axis(rx),
            _axis(ry),
            dpad["left"],
            dpad["down"],
            dpad["right"],
            dpad["up"],
            analog["square"],
            analog["cross"],
            analog["circle"],
            analog["triangle"],
            analog["r1"],
            analog["l1"],
            analog["r2"],
            analog["l2"],
            b"\x00" * 6,
            b"\x00" * 6,
            self.timestamp_us & 0xFFFFFFFFFFFFFFFF,
            float(self.accel[0]),
            float(self.accel[1]),
            float(self.accel[2]),
            float(self.gyro[0]),
            float(self.gyro[1]),
            float(self.gyro[2]),
        )
        return build_message(source_id, MSG_DATA, payload)


def _axis(value: float) -> int:
    """Convertit -1.0 .. +1.0 en octet 0..255 (128 = repos)."""
    return max(0, min(255, int(round(128 + 127 * float(value)))))


@dataclass(frozen=True)
class Request:
    """Requete recue d'un client (Dolphin, Cemu, ...)."""

    magic: bytes
    version: int
    length: int
    crc: int
    source_id: int
    msg_type: int
    payload: bytes

    @property
    def register_flags(self) -> int:
        return self.payload[0] if self.msg_type == MSG_DATA and len(self.payload) >= 1 else 0

    @property
    def requested_slot(self) -> int:
        return self.payload[1] if self.msg_type == MSG_DATA and len(self.payload) >= 2 else 0

    @property
    def requested_mac(self) -> bytes:
        if self.msg_type == MSG_DATA and len(self.payload) >= 8:
            return self.payload[2:8]
        return bytes(6)


def parse_request(data: bytes) -> Request | None:
    """Analyse un paquet client ; renvoie None si le paquet est invalide."""
    if len(data) < HEADER_SIZE + 4:
        return None
    magic, version, length, crc, source_id = HEADER.unpack_from(data)
    if magic not in (MAGIC_CLIENT, MAGIC_SERVER):
        return None
    total = HEADER_SIZE + length
    if length < 4 or len(data) < total:
        return None
    (msg_type,) = U32.unpack_from(data, 16)
    return Request(magic, version, length, crc, source_id, msg_type, bytes(data[20:total]))


def crc_is_valid(data: bytes) -> bool:
    """Verifie le CRC32 d'un paquet recu."""
    if len(data) < HEADER_SIZE:
        return False
    (crc,) = U32.unpack_from(data, 8)
    buffer = bytearray(data)
    struct.pack_into("<I", buffer, 8, 0)
    return crc == (zlib.crc32(buffer) & 0xFFFFFFFF)
