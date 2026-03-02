# ==========================================
# ATOM S3 + ATOM CAN Base
# Ver 4.2.1 - BLE Visibility & Pairing Fix
# ==========================================
# [修正のポイント]
# 1. BLE視認性の向上: 名前を "M5AtomS3" に短縮し、アドバタイズ間隔を最適化。
# 2. ペアリング要求への対応: 接続時に "NEED_PIN" を送信し、Webアプリ側の認証を開始。
# 3. 堅牢なインポートと永続化を維持。
# 4. CAN通信の継続 (G5/G6)
# ==========================================
import time
import machine
import bluetooth
import struct
import json
import os
from micropython import const
# --- Robust Imports ---
try:
    import M5
    from M5 import *
    HAS_M5 = True
except ImportError:
    HAS_M5 = False
try:
    from unit import CANUnit
    HAS_CANUNIT = True
except ImportError:
    HAS_CANUNIT = False
try:
    from esp32 import NVS
    HAS_NVS = True
except ImportError:
    HAS_NVS = False
# --- Configuration & Persistence ---
CONFIG_FILE = "/flash/candash.json"
class Config:
    def __init__(self):
        self.data = {
            "grp1_id": 0x4E0,
            "grp2_id": 0x4E1,
            "my_id": 0x5A0,
            "k_meter_id": 0x661,
            "slot_modes": [1, 1, 1, 1, 1, 1, 1],
            "btn_on": ["01"] * 8,
            "btn_off": ["00"] * 8,
            "can_state": [0] * 8
        }
        self.nvs = None
        if HAS_NVS:
            try:
                self.nvs = NVS("can_dash")
            except: pass
        self.load()
    def load(self):
        if self.nvs:
            try:
                v = self.nvs.get_i32("grp1")
                if v: self.data["grp1_id"] = v
                v = self.nvs.get_i32("grp2")
                if v: self.data["grp2_id"] = v
                v = self.nvs.get_i32("my_id")
                if v: self.data["my_id"] = v
                v = self.nvs.get_i32("kid")
                if v: self.data["k_meter_id"] = v
            except: pass
        try:
            with open(CONFIG_FILE, "r") as f:
                saved = json.load(f)
                self.data.update(saved)
        except: pass
    def save(self):
        if self.nvs:
            try:
                self.nvs.set_i32("grp1", self.data["grp1_id"])
                self.nvs.set_i32("grp2", self.data["grp2_id"])
                self.nvs.set_i32("my_id", self.data["my_id"])
                self.nvs.set_i32("kid", self.data["k_meter_id"])
                self.nvs.commit()
            except: pass
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(self.data, f)
        except: pass
cfg = Config()
# --- Hardware Constants ---
TX_PIN = 6
RX_PIN = 5
CAN_BAUDRATE = 1000000
# --- Global Variables ---
can = None
ble_tx_queue = []
ble_rx_queue = []
monitor_vals = [0] * 7
can_error = False
last_can_send = 0
last_ble_send = 0
last_display = 0
last_recovery = 0
_pending_config_send = False
# --- BLE UART ---
UART_UUID = bluetooth.UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
UART_TX   = bluetooth.UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
UART_RX   = bluetooth.UUID("6e400002-b5a3-f393-e0a9-e50e24dcca9e")
_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)
class BLEUART:
    def __init__(self, ble):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._tx, self._rx),) = self._ble.gatts_register_services([
            (UART_UUID, ((UART_TX, bluetooth.FLAG_NOTIFY), (UART_RX, bluetooth.FLAG_WRITE)))
        ])
        self._connections = set()
        self._advertise()
    def _advertise(self):
        try:
            # Device Name: "M5AtomS3" (8 bytes)
            # AD Type 0x01 (Flags): 0x06
            # AD Type 0x09 (Name): Length 9
            self._ble.gap_advertise(100000, b'\x02\x01\x06\x09\x09M5AtomS3')
        except: pass
    def _irq(self, event, data):
        global _pending_config_send
        if event == _IRQ_CENTRAL_CONNECT:
            self._connections.add(data[0])
            # Connection feedback
            time.sleep_ms(300)
            self.send(b"NEED_PIN")
            _pending_config_send = True
        elif event == _IRQ_CENTRAL_DISCONNECT:
            if data[0] in self._connections: self._connections.remove(data[0])
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            try:
                cmd = self._ble.gatts_read(self._rx).decode().strip()
                if cmd: ble_rx_queue.append(cmd)
            except: pass
    def send(self, data):
        for conn in self._connections:
            try: self._ble.gatts_notify(conn, self._tx, data)
            except: pass
# --- CAN Logic ---
def update_hw_filter():
    if not can: return
    try:
        g1 = cfg.data["grp1_id"]
        g2 = cfg.data["grp2_id"]
        code_32 = g1 << 21
        diff = g1 ^ g2
        mask_11bit = diff & 0x7FF
        mask_32 = (mask_11bit << 21) | 0x1FFFFF
        can.setfilter(0, CANUnit.MASK32, 0, [code_32, mask_32])
    except: pass
def init_can():
    global can, can_error
    if not HAS_CANUNIT: return
    try:
        can = CANUnit(0, port=(TX_PIN, RX_PIN), mode=CANUnit.NORMAL, baudrate=CAN_BAUDRATE)
        time.sleep_ms(100)
        update_hw_filter()
        can_error = False
    except:
        can_error = True
def check_can_recovery():
    global can, last_recovery, can_error
    now = time.ticks_ms()
    if time.ticks_diff(now, last_recovery) < 5000: return
    last_recovery = now
    if can and hasattr(can, 'state') and can.state() != CANUnit.RUNNING:
        init_can()
def extract_val(data, pos, mode):
    if len(data) <= pos: return 0
    if mode == 0 or mode == 3: return data[pos]
    if len(data) <= pos + 1: return 0
    if mode == 1 or mode == 4: return data[pos] | (data[pos+1] << 8)
    return (data[pos] << 8) | data[pos+1]
def process_ble_cmd(uart):
    global _pending_config_send
    if not ble_rx_queue: return
    cmd = ble_rx_queue.pop(0)
    try:
        if cmd.startswith("SET_GROUPS="):
            p = cmd.split('=')[1].split(',')
            cfg.data["grp1_id"] = int(p[0], 16)
            cfg.data["grp2_id"] = int(p[1], 16)
            cfg.save()
            update_hw_filter()
            _pending_config_send = True
        elif cmd.startswith("SET_SLOT_MODE="):
            p = cmd.split('=')[1].split(',')
            idx, mode = int(p[0]), int(p[1])
            if 0 <= idx < 7:
                cfg.data["slot_modes"][idx] = mode
                cfg.save()
                uart.send(f"SLOT_MODE_OK={idx},{mode}".encode())
        elif cmd.startswith("ID="):
            cfg.data["my_id"] = int(cmd.split('=')[1], 16)
            cfg.save()
            uart.send(f"ID={hex(cfg.data['my_id'])}".encode())
        elif cmd.startswith("KID="):
            cfg.data["k_meter_id"] = int(cmd.split('=')[1], 16)
            cfg.save()
            uart.send(f"KID={hex(cfg.data['k_meter_id'])}".encode())
        elif cmd.startswith("CFGBTN="):
            p = cmd.split('=')[1].split(',')
            idx = int(p[0])
            on_hex = hex(int(p[1]))[2:].zfill(2)
            off_hex = hex(int(p[2]))[2:].zfill(2)
            if 0 <= idx < 8:
                cfg.data["btn_on"][idx] = on_hex
                cfg.data["btn_off"][idx] = off_hex
                cfg.save()
        elif cmd == "REQUEST_STATE":
            _pending_config_send = True
        elif "=" in cmd:
            p = cmd.split('=')
            if p[0].isdigit():
                idx = int(p[0]) - 1
                val = int(p[1])
                if 0 <= idx < 8:
                    cfg.data["can_state"][idx] = val
                    if can:
                        try:
                            can.send(bytearray(cfg.data["can_state"]), cfg.data["my_id"], timeout=0)
                            uart.send(("STATE=" + ",".join(str(b) for b in cfg.data["can_state"])).encode())
                        except: pass
    except: pass
def process_can_rx():
    global can_error
    if not can: return
    try:
        while True:
            if can.any(0) == 0: break
            msg = can.recv(0, timeout=0)
            if not msg: break
            can_id = msg[0] & 0x7FF
            data = msg[4]
            if can_id == cfg.data["grp1_id"]:
                for i in range(4):
                    monitor_vals[i] = extract_val(data, i*2, cfg.data["slot_modes"][i])
            elif can_id == cfg.data["grp2_id"]:
                for i in range(3):
                    monitor_vals[i+4] = extract_val(data, i*2, cfg.data["slot_modes"][i+4])
            can_error = False
    except:
        can_error = True
# --- Main Execution ---
if HAS_M5:
    M5.begin()
    M5.Display.setTextSize(2)
    M5.Display.clear()
init_can()
ble = bluetooth.BLE()
uart = BLEUART(ble)
while True:
    if HAS_M5: M5.update()
    now = time.ticks_ms()
    process_ble_cmd(uart)
    
    if _pending_config_send:
        _pending_config_send = False
        time.sleep_ms(100)
        uart.send(("STATE=" + ",".join(str(b) for b in cfg.data["can_state"])).encode())
        uart.send(f"GRP1={hex(cfg.data['grp1_id'])}".encode())
        uart.send(f"GRP2={hex(cfg.data['grp2_id'])}".encode())
        uart.send(f"ID={hex(cfg.data['my_id'])}".encode())
        uart.send(f"KID={hex(cfg.data['k_meter_id'])}".encode())
    process_can_rx()
    
    if time.ticks_diff(now, last_ble_send) > 50:
        uart.send(("VALS=" + ",".join(str(v) for v in monitor_vals)).encode())
        last_ble_send = now
    if time.ticks_diff(now, last_can_send) > 100:
        if can:
            try:
                can.send(bytearray(cfg.data["can_state"]), cfg.data["my_id"], timeout=0)
            except: pass
        last_can_send = now
    if time.ticks_diff(now, last_display) > 250:
        check_can_recovery()
        uart.send(b"STATUS=ERR" if can_error else b"STATUS=CAN_OK")
        if HAS_M5:
            M5.Display.setCursor(0, 0)
            M5.Display.print("V4.2.1 FIX")
            M5.Display.setCursor(0, 20)
            M5.Display.print("CAN:%s" % ("ERR" if can_error else "OK "))
        last_display = now
    time.sleep_ms(1)
