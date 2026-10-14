# vehicle_vwegolf — dev-line findings

Findings about upstream master that are not (yet) fixed upstream. Signal decode
lives in `vwegolf.dbc`; this file records state, evidence, and open work only.

## 2026-10-14 — state of upstream master `4bb334a59` vs retired dev line `vwegolf` (`2f61f07dd`)

### 1. Battery and charge current/voltage/power are dead on master

- #1524 removed can2/FCAN and moved the `0x191` (`BMS_PowerBus`) handler into
  `IncomingFrameCan3`, assuming the J533 forwards it to KCAN.
- On our car it does not. Capture corpus (`garmzon/vwegolf-captures`,
  `candumps/`, 40 captures, ~2.5M KCAN frames): `0x191` appears 0 times on KCAN.
  The three dual-bus captures
  (`all-168388a82-dirty_ota_0_edge-20260524-224827`,
  `all-3.3.006-192-g2190a497a-dirty_ota_0_edge-20260621-182359`,
  `all-3.3.006-269-gab4f52853_ota_0_edge-20260712-124542`) carry 1.5k–4.6k
  `0x191` frames on FCAN and none on KCAN at the same time.
- Other frames #1524 relies on are forwarded: `0x187`, `0x131`, `0x2AF`, `0x6B4`.
- Telemetry (Influx `ovms-telemetry`) on edge `3.3.006-360-g86dfa0af`:
  `v.b.current`, `v.b.voltage`, `v.b.power` have no points at all since
  2026-09-30; `v.c.current`, `v.c.power`, `v.c.voltage` read 0 throughout an AC
  charge (2026-10-13, SoC 66→85) and a CCS session (2026-10-14, SoC 14→53).
- On the car build `archive/car-aefeffec5` (until 2026-09-14) the same metrics
  had real values (last nonzero charge sample 54 A / 19.5 kW).
- Early captures `kcan-capture.crtd`, `kcan-can3*.crtd` and
  `can2-9cda64d69-…-20260611` mix KCAN-only IDs and FCAN `0x191` under one bus
  label (merged-bus logging artefact). Use the dual-bus `all-*` captures for
  bus-presence claims.
- Unknown: whether other gateway variants forward `0x191`. Asked on #690 (draft).

**Replacement source, proven on the car build:** UDS poll of the BMS (ECU 8C)
on can1 / diag CAN at the J533, req `0x7E5`, resp `0x7ED`, DIDs `0x1E3B`
(voltage) and `0x1E3D` (current). Reference implementation: tag
`archive/car-aefeffec5`, commits `32e29846a`, `55c1e19db`. Matches `0x191`
within 0.5 A typical / 2 A max and voltage exact in the June 21 and July 12
dual-bus captures, but only over -80..+4 A — discharge under load and regen
are unvalidated. Earlier note "0x191 is silent during DC" is false: the June 21
capture has it live at about -67 A × 358 V during a DC charge.

### 2. NM node 0x67 is still impersonated on master

- Node `0x1B000067` is a live KCAN ECU (6369 frames in 21 captures, payload
  varies within a session).
- Master `vehicle_vwegolf.cpp` still transmits as that node from two places,
  both from our own commit `6246d8154` (2026-05-29, upstreamed with #1430):
  - `WakeKcanBus()` — once per `CommandWakeup()`.
  - `SendNmAlive()` — 1 Hz from `Ticker1` while `m_ocu_active`; ends on ACK
    grace (5 s), session cap (30 s), OEM OCU seen, or KCAN idle.
- Climate and charge control are clean: SCjona's `vehicle_vwegolf_bat_ctrl.cpp`
  wakes from spare node `0x7D` (`VWEGOLF_NM_WAKE_NODE`).
- Our fix (node `0x7D`, on-car verified in WI-NM-3, 2026-07-31) lives only on
  the retired `vwegolf` line and was never sent upstream.
- Open decision for the fix PR: payload. Ours was `7D 10 08 01 14 00 00 00`,
  SCjona's is `7D 10 49 85 14 00 00 00`.

### 3. CMake build broken on master

`CMakeLists.txt` lists `src/vehicle_vwegolf_climate.cpp`, which does not exist
(`src/vehicle_vwegolf_bat_ctrl.cpp` does). `component.mk` globs `src/*.cpp`, so
the Make build is unaffected. Fix open upstream: #1553 (nickb24).

### 4. Fork-only features not on master

| Feature | Where it lives |
|---|---|
| UDS BMS poll (charge V/I/P, battery V/I/P while charging) | `archive/car-aefeffec5` |
| Battery SoH via UDS DID `0x74CB` (scale raw/500, provisional) | `archive/car-aefeffec5`, commit `2c99b2724` |
| Camping mode (`xvg camping`, `cc-camp-*` params) | `vwegolf` |
| Web UI page | `vwegolf` (`src/vwegolf_web.cpp`) |
| FCAN hardware acceptance filter | `archive/pr-1403`, `archive/pr-1404` — obsolete while FCAN stays off |

Everything else on the retired line has an equal or better counterpart on
master (charge control, profiles, departure timers, BAP climate with confirm
and retry, door/lock/body, gear, VIN, SoC, trip energy).

### 5. Stale claims on master

- `tests/candumps/README.md:61` and the `candumps/kcan-synthetic.crtd` header
  claim `0x191` on KCAN.
- Comment "0x5AC byte 3 mirrors pack current" did not hold in a quick check
  (July 12 capture: `d[3]` = 139 while `0x191` current ≈ 0).
