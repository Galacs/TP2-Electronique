# Pont orientation → DSU (BMA220 / Arduino Uno)

```
┌──────────┐  I2C   ┌──────────────┐  USB série   ┌────────────────┐  UDP 26760  ┌──────────┐
│ BMA220   │ ─────► │ Arduino Uno  │ ───────────► │ dsu_bridge.py  │ ──────────► │ Dolphin  │
│ 3 axes   │        │ 50 Hz        │  DSU,ax,ay,az│ (lissage,      │   DSU       │ Cemu...  │
└──────────┘        └──────────────┘              │  calibration)  │             └──────────┘
                                                  └────────────────┘
```

Le pont lit l'accéléromètre sur le port série, en déduit l'orientation, et la
publie en continu sur le port UDP **26760** au format **DSU** (dit
« cemuhook »), celui que Dolphin, Cemu, Yuzu/Ryujinx et DS4Windows savent lire.
Côté émulateur, la carte apparaît comme une manette avec capteur de mouvement.

Les boutons ne sont pas générés : dans l'émulateur, garde le clavier pour A/B
(chaque entrée peut venir d'un périphérique différent).

---

## 1. Côté Arduino : la sortie rapide

Le sketch du TP (`src/main.cpp`) n'est **pas** modifié. Une variante autonome
est fournie dans `firmware/dsu_orientation/` : elle envoie à **50 Hz** une
ligne machine par mesure :

```
DSU,ax,ay,az,angle        →   DSU,0.0625,-0.0625,0.9961,3.58
```

Pour la flasher :

```bash
cd firmware/dsu_orientation
pio run -t upload
```

Pour revenir au sketch du TP : `pio run -t upload` à la racine du dépôt.

> Le pont accepte aussi l'ancien format du TP (`x = 0.12 gy = -0.03 gz = 0.98 g`),
> mais il n'est envoyé qu'à 5 Hz : le pilotage sera saccadé (~200 ms de latence).
> La variante 50 Hz est donc fortement recommandée.

Le BMA220 est réglé en ±2 g (16 LSB/g, 6 bits utiles, soit 0,0625 g ≈ 3,6° de
résolution). C'est suffisant pour incliner une manette, pas pour un gyroscope
précis — voir la section « Limites ».

---

## 2. Côté PC : installation

Prérequis : Python 3.10+ et pyserial.

```bash
python3 -m pip install -r pc/requirements.txt
```

Accès au port série (une seule fois, puis se reconnecter) :

```bash
sudo usermod -aG uucp "$USER"      # Arch, Manjaro
sudo usermod -aG dialout "$USER"   # Debian, Ubuntu, Mint
```

Vérifier que la carte est vue :

```bash
python3 pc/dsu_bridge.py --list-ports
```

---

## 3. Lancer le pont

```bash
python3 pc/dsu_bridge.py --calibrate
```

* `--calibrate` : garde la carte immobile 2 secondes au démarrage ; la position
  de repos devient le zéro (indispensable si la carte n'est pas à plat).
* Sans `--calibrate`, le zéro est l'horizontale (`atan2(ax, az) = 0`).
* `Ctrl+C` pour arrêter.
* À l'ouverture du port, l'Arduino redémarre (~2 s) : c'est normal, le pont
  attend les premières mesures avant de calibrer.

Le terminal affiche une ligne d'état :

```
angle   +12.3 deg  stick +0.42  clients 1  paquets 1523  serie /dev/ttyACM0 (812 mesures)
```

`clients 1` signifie qu'un émulateur est abonné ; `paquets` doit augmenter
d'environ 60 par seconde.

### Options utiles

| Option | Défaut | Rôle |
|---|---|---|
| `--serial /dev/ttyACM0` | auto | port série (détection automatique sinon) |
| `--axis pitch\|roll` | `pitch` | axe d'inclinaison utilisé (`pitch` = `atan2(ax, az)`, comme le TP) |
| `--invert` | off | inverse le sens (gauche ↔ droite) |
| `--center ANGLE` | 0 | zéro manuel, en degrés |
| `--range DEG` | 35 | angle d'inclinaison donnant la course complète |
| `--deadzone DEG` | 3 | zone morte autour du centre |
| `--expo 0..1` | 0.3 | courbe de réponse (précision près du centre) |
| `--smooth MS` | 80 | lissage passe-bas (ms) |
| `--stick` | off | mappe aussi l'inclinaison sur le stick gauche (X) |
| `--stick-gain` | 1.0 | gain du stick |
| `--gyro-axis pitch\|yaw\|roll\|none` | `roll` | axe où placer le gyroscope dérivé |
| `--accel-signs 1,1,1` | `1,1,1` | inverse des axes de l'accéléromètre envoyé |
| `--rate HZ` | 60 | cadence d'envoi DSU (10 à 200) |
| `--udp-port` | 26760 | port DSU (à changer seulement en cas de conflit) |
| `--simulate` | off | génère une inclinaison fictive (test sans carte) |
| `--console` / `--no-console` | auto | affichage continu de l'état |
| `-v` / `-vv` | — | journaux détaillés |

Exemples :

```bash
# Mario Kart Wii au volant : l'inclinaison pilote le stick (donc le Tilt Dolphin)
python3 pc/dsu_bridge.py --calibrate --stick --range 30 --deadzone 4

# La carte est montée verticalement : c'est l'autre axe qui tourne
python3 pc/dsu_bridge.py --axis roll --invert

# Tout est inversé et tremble un peu
python3 pc/dsu_bridge.py --invert --deadzone 5 --smooth 120
```

---

## 4. Vérifier sans émulateur

Deux terminaux :

```bash
python3 pc/dsu_bridge.py --simulate --console --stick
python3 pc/dsu_probe.py
```

Le client de test fait exactement ce que fait Dolphin (liste des manettes puis
abonnement aux données) et affiche :

```
Controleur DSU : slot 0, modele 2 (gyro complet), batterie 5, MAC 02:00:00:00:00:01
Paquets    : 184 en 3.02 s (61.0/s)
Accel (g)  : x -0.42..+0.42  y -0.25..+0.15  z +0.88..+1.00
Gyro (d/s) : pitch +0.00..+0.00  yaw +0.00..+0.00  roll -54.5..+56.1
Stick      : x -0.58..+0.57  y +0.00..+0.00
```

`--watch` pour un affichage continu, `--json` pour un objet JSON par paquet,
`--require-motion` pour échouer si rien ne bouge.

Auto-test complet (aucun matériel) :

```bash
python3 pc/selftest.py
```

---

## 5. Configuration Dolphin

### 5.1 Activer le serveur DSU

1. Lancer Dolphin, puis **Controllers** (Manettes).
2. Bouton **Alternate Input Sources** (Sources d'entrée alternatives).
3. Onglet **DSU Client** : cocher **Enable**, puis **Add...**
   * Description : `TP2` (libre)
   * Server IP Address : `127.0.0.1`
   * Server Port : `26760`
4. Laisser le pont tourner : Dolphin interroge le serveur toutes les secondes.
   Le périphérique `DSUClient` (slot 0) apparaît dans les listes de manettes.
   Sinon, rouvrir la fenêtre ou cliquer **Refresh**.

### 5.2 Volant Wii (Mario Kart Wii, pilotage à l'inclinaison)

C'est la voie recommandée pour « juste le mouvement » :

1. Lancer le pont avec `--stick` (l'inclinaison est alors aussi exposée comme
   axe de stick, ce que Dolphin sait transformer en inclinaison du Wiimote).
2. **Controllers → Wii Remotes → Wii Remote 1** : choisir
   **Emulated Wii Remote**, puis **Configure**.
3. Onglet **General and Options** : cocher **Sideways Wii Remote**
   (Wiimote de côté, comme un volant) et mettre **Extension** sur *None*.
4. Onglet **Motion Simulation → Tilt** : cliquer sur **Left** puis incliner la
   carte à gauche (ou choisir l'entrée `DSUClient … Left X-`) ; faire de même
   pour **Right** avec `Left X+`.
5. Dans Mario Kart Wii, choisir le **Wii Wheel** au moment de connecter la
   manette.

Régler ensuite le ressenti avec `--range` (plus petit = plus sensible),
`--deadzone` et `--expo`.

> Variante « accéléromètre brut » : onglet **Motion Input → Accelerometer**,
> mapper les six entrées `Accel Up/Down/Left/Right/Forward/Backward`. C'est ce
> que le pont envoie réellement (vecteur de gravité en g), mais les axes et les
> signes dépendent de la fixation de la carte : ajuster avec `--accel-signs`.

### 5.3 Manette GameCube (stick analogique)

1. Lancer le pont avec `--stick`.
2. **Controllers → GameCube → Port 1** : choisir le périphérique DSU (slot 0).
3. Mapper **Stick X** sur `Left X-` / `Left X+`, puis **A** et **B** sur des
   touches du clavier (même écran de configuration, chaque entrée peut venir
   d'un périphérique différent).
4. Dans Mario Kart Wii, le jeu détecte la manette GameCube.

### 5.4 Si ça tourne du mauvais côté

* Sens inversé : `--invert`.
* L'axe qui réagit n'est pas le bon : `--axis roll` (ou `pitch`).
* En mode accéléromètre brut, inverser un axe : `--accel-signs 1,-1,1`.

---

## 6. Cemu (indicatif)

Cemu parle aussi le protocole DSU : dans ses réglages de manette, choisir
l'API **DSUClient**, serveur `127.0.0.1:26760`, puis mapper le stick ou le
mouvement comme dans Dolphin. Mario Kart 8 n'utilise pas le gyroscope en jeu ;
le pont reste utile pour les jeux qui lisent les capteurs (Splatoon, Zelda...).

---

## 7. Comment ça marche

* Le pont écoute en UDP sur `26760` et répond à trois requêtes DSU :
  version (`0x100000`), liste des manettes (`0x100001`) et abonnement aux
  données (`0x100002`).
* Il annonce une manette au slot 0, modèle « gyro complet », branchée en USB,
  batterie pleine.
* Chaque paquet de données fait 100 octets : sticks 0..255 (128 = repos),
  accéléromètre en **g**, gyroscope en **deg/s**, horodatage en µs, CRC32.
* L'accéléromètre est envoyé tel que mesuré (avec la gravité) ; le gyroscope
  est **dérivé** de la variation d'angle (le BMA220 n'a pas de gyroscope).
* Dès que le port série se tait plus d'une seconde, le pont renvoie une position
  neutre (pas de volant bloqué à droite).
* Le CRC est celui de zlib, calculé champ CRC à zéro : c'est exactement ce que
  vérifie Dolphin.

### Limites

* 6 bits en ±2 g : 0,0625 g par pas, soit ~3,6° d'angle. Le lissage et la zone
  morte masquent l'essentiel, mais ce n'est pas un capteur de précision.
* Pas de gyroscope réel : le « gyro » DSU est une dérivée, donc bruitée et en
  retard. Pour Mario Kart, c'est l'accéléromètre (via le Tilt) qui compte.
* Latence : ~20 ms (50 Hz) + lissage (80 ms par défaut). Réduire `--smooth`
  rend plus réactif mais plus nerveux.

---

## 8. Dépannage

| Symptôme | Piste |
|---|---|
| `Aucun port série détecté` | câble USB données (pas seulement charge), `--list-ports`, groupe `uucp`/`dialout` |
| `Permission denied` sur `/dev/ttyACM0` | `sudo usermod -aG uucp $USER` (Arch) ou `dialout`, puis reconnexion |
| Dolphin ne voit pas la manette | DSU Client activé + serveur ajouté, pont lancé, `ss -lunp \| grep 26760`, attendre 1 s / Refresh |
| `Adresse déjà utilisée` au démarrage | un autre serveur DSU (DS4Windows, BetterJoy) occupe 26760 : `--udp-port 26761` (et changer le port dans Dolphin) |
| La manette apparaît mais rien ne bouge | `python3 pc/dsu_probe.py` : si lui ne bouge pas, le port série/format est en cause |
| Ça tremble au repos | `--deadzone 5 --smooth 120`, vérifier que la carte est fixée |
| Le volant tourne à l'envers | `--invert` |
| Le volant tourne dans le mauvais axe | `--axis roll` |
| Mouvement trop faible/fort | `--range 20` (plus sensible) ou `--range 50` |
| 5 Hz, saccadé | flasher `firmware/dsu_orientation` |

---

## 9. Fichiers

| Fichier | Rôle |
|---|---|
| `pc/dsu_bridge.py` | le pont série → DSU |
| `pc/dsu_protocol.py` | encodage des paquets DSU et CRC32 |
| `pc/dsu_probe.py` | client de test (fait comme Dolphin) |
| `pc/selftest.py` | auto-test complet sans matériel |
| `firmware/dsu_orientation/` | sketch Arduino 50 Hz, projet PlatformIO autonome |

## Références

* Protocole DSU (cemuhook) : <https://v1993.github.io/cemuhook-protocol/>
* Client DSU de Dolphin : <https://wiki.dolphin-emu.org/index.php?title=DSU_Client>
