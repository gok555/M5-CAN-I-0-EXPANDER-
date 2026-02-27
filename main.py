# ==========================================
# ATOM S3R + Atomic CAN Base + EXT.I/O 2
# Ver 2.0 - Analog→CAN + AUX Keypad
# ==========================================
# INPUT ピン : EXT.I/O 2 ADC読取 → CAN送信
# AUX   ピン : キーパッドON/OFF  → 別CAN ID送信
# ==========================================
import M5
from M5 import *
import time
from hardware import I2C, Pin
from unit import CANUnit
import bluetooth
import struct
from micropython import const
from esp32 import NVS

# ── ハードウェア定義 ──────────────────────
TX_PIN      = 6
RX_PIN      = 5
CAN_BAUDRATE = 500000
I2C_SCL_PIN = 1
I2C_SDA_PIN = 2
EXTIO2_ADDR = 0x45

# ── EXT.I/O 2 レジスタマップ ──────────────
REG_MODE_BASE  = 0x00  # +pin(0-7) : モード設定
REG_ADC_BASE   = 0x20  # +pin*2    : ADC値 2byte LE
REG_OUT_BASE   = 0x10  # +pin      : デジタル出力
EXTIO2_MODE_OUTPUT = 1
EXTIO2_MODE_ADC    = 4

# ── デフォルト設定 ────────────────────────
NUM_CH      = 8
can_tx_id   = 0x500   # ADC用 CAN TX ID (Frame1=id, Frame2=id+1)
sw_can_id   = 0x5A0   # スイッチ用 CAN TX ID (1フレーム8バイト)

# pin_modes: 0=AUX(キーパッドスイッチ), 1=INPUT(ADC)
pin_modes   = [1] * NUM_CH

# スイッチ状態バッファ  can_state[i] = 現在のCANバイト値
can_state   = bytearray(8)

# スイッチON/OFF値 (ユーザー設定, デフォルト: ON=1, OFF=0)
sw_on_vals  = [1] * NUM_CH
sw_off_vals = [0] * NUM_CH

# ── タイミング(ms) ─────────────────────────
CAN_SEND_INT  = 50
BLE_SEND_INT  = 50
DISP_INT      = 600
RECOVERY_INT  = 5000

# ── 状態変数 ──────────────────────────────
can           = None
i2c_bus       = None
extio_found   = False
can_error     = False
ble_tx_queue  = []
ble_rx_queue  = []
nvs           = None
_pending_cfg  = False

last_can_send = 0
last_ble_send = 0
last_display  = 0
last_recovery = 0

adc_raw       = [0] * NUM_CH   # 12bit ADC生値

# ── BLE UART (Nordic UART Service) ───────
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
            (UART_UUID, ((UART_TX, bluetooth.FLAG_NOTIFY),
                         (UART_RX, bluetooth.FLAG_WRITE)))
        ])
        self._connections = set()
        self._advertise()

    def _advertise(self):
        try:
            self._ble.gap_advertise(100000, b'\x02\x01\x06\x10\x09M5S3R_AnaCAN')
        except: pass

    def _irq(self, event, data):
        global _pending_cfg
        if event == _IRQ_CENTRAL_CONNECT:
            self._connections.add(data[0])
            _pending_cfg = True
        elif event == _IRQ_CENTRAL_DISCONNECT:
            if data[0] in self._connections:
                self._connections.remove(data[0])
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            try:
                cmd = self._ble.gatts_read(self._rx).decode().strip()
                if cmd:
                    ble_rx_queue.append(cmd)
            except: pass

    def send(self, data):
        for conn in self._connections:
            try: self._ble.gatts_notify(conn, self._tx, data)
            except: pass

    @property
    def connected(self):
        return len(self._connections) > 0


# ── NVS 保存 ──────────────────────────────
def nvs_save_all():
    if not nvs: return
    try:
        nvs.set_i32("can_tx_id", can_tx_id)
        nvs.set_i32("sw_can_id", sw_can_id)
        for i in range(NUM_CH):
            nvs.set_i32(f"pin_mode_{i}", pin_modes[i])
            nvs.set_i32(f"sw_on_{i}",   sw_on_vals[i])
            nvs.set_i32(f"sw_off_{i}",  sw_off_vals[i])
            nvs.set_i32(f"sw_state_{i}", int(can_state[i]))
        nvs.commit()
    except Exception as e:
        print("NVS Save Err:", e)


def nvs_save_state():
    """スイッチ状態だけ高速保存"""
    if not nvs: return
    try:
        for i in range(NUM_CH):
            nvs.set_i32(f"sw_state_{i}", int(can_state[i]))
        nvs.commit()
    except: pass


# ── EXT.I/O 2 ────────────────────────────
def apply_pin_modes():
    """設定に基づき EXT.I/O 2 の各ピンモードを書き込む"""
    if not extio_found: return
    for i in range(NUM_CH):
        hw_mode = EXTIO2_MODE_ADC if pin_modes[i] == 1 else EXTIO2_MODE_OUTPUT
        try:
            i2c_bus.writeto_mem(EXTIO2_ADDR, REG_MODE_BASE + i, bytes([hw_mode]))
            time.sleep_ms(3)
        except: pass


def read_adc_all():
    """INPUT チャンネルの ADC 値を取得 (12bit, 0-4095)"""
    if not extio_found: return
    for i in range(NUM_CH):
        if pin_modes[i] != 1:
            adc_raw[i] = 0
            continue
        try:
            d = i2c_bus.readfrom_mem(EXTIO2_ADDR, REG_ADC_BASE + i * 2, 2)
            adc_raw[i] = (d[0] | (d[1] << 8)) & 0x0FFF
        except:
            adc_raw[i] = 0


# ── CAN ──────────────────────────────────
def build_can_frame(start_ch):
    """4ch分を8バイトCANフレームへパック (2byte LE, 12bit)"""
    buf = bytearray(8)
    for j in range(4):
        val = adc_raw[start_ch + j] & 0x0FFF
        buf[j * 2]     = val & 0xFF
        buf[j * 2 + 1] = (val >> 8) & 0x0F
    return buf


def safe_can_send(cid, data):
    global can_error
    if can:
        try:
            can.send(data, cid, timeout=0)
            can_error = False
        except:
            can_error = True


def check_can_recovery():
    global can, last_recovery, can_error
    now = time.ticks_ms()
    if time.ticks_diff(now, last_recovery) < RECOVERY_INT: return
    last_recovery = now
    try:
        if can and can.state() != CANUnit.RUNNING:
            can_error = True
            can.deinit()
            time.sleep_ms(50)
            can = CANUnit(0, port=(TX_PIN, RX_PIN),
                          mode=CANUnit.NORMAL, baudrate=CAN_BAUDRATE)
            time.sleep_ms(100)
            can_error = False
    except: pass


def send_sw_can():
    """スイッチ状態を sw_can_id で CAN 送信"""
    safe_can_send(sw_can_id, can_state)


# ── BLE コマンド処理 ──────────────────────
def queue_config_sync():
    """接続時に全設定をクライアントへ送信"""
    ble_tx_queue.append(f"TXID={hex(can_tx_id)}".encode())
    ble_tx_queue.append(f"SW_TXID={hex(sw_can_id)}".encode())
    ble_tx_queue.append(("PIN_MODES=" + ",".join(str(m) for m in pin_modes)).encode())
    ble_tx_queue.append(("SW_ON="  + ",".join(str(v) for v in sw_on_vals)).encode())
    ble_tx_queue.append(("SW_OFF=" + ",".join(str(v) for v in sw_off_vals)).encode())
    ble_tx_queue.append(("SW_VALS=" + ",".join(str(b) for b in can_state)).encode())


def process_ble_cmd():
    global can_tx_id, sw_can_id, pin_modes, sw_on_vals, sw_off_vals
    if not ble_rx_queue: return
    cmd = ble_rx_queue.pop(0)
    try:
        # ── 設定要求 ──────────────────────────
        if cmd == "REQUEST_STATE":
            queue_config_sync()
            return

        # ── ADC CAN TX ID   SET_TXID=0x501 ───
        if cmd.startswith("SET_TXID="):
            can_tx_id = int(cmd.split('=')[1], 16)
            nvs_save_all()
            ble_tx_queue.append(f"TXID={hex(can_tx_id)}".encode())

        # ── スイッチ CAN TX ID  SW_TXID=0x5A0 ─
        elif cmd.startswith("SW_TXID="):
            sw_can_id = int(cmd.split('=')[1], 16)
            nvs_save_all()
            ble_tx_queue.append(f"SW_TXID={hex(sw_can_id)}".encode())

        # ── ピンモード変更  SET_PIN_MODE=3,0 ──
        elif cmd.startswith("SET_PIN_MODE="):
            p = cmd.split('=')[1].split(',')
            pin = int(p[0]); mode = int(p[1])
            if 0 <= pin < NUM_CH:
                pin_modes[pin] = mode
                if mode == 0:           # AUXになったらOFF値をセット
                    can_state[pin] = sw_off_vals[pin]
                apply_pin_modes()
                nvs_save_all()
                ble_tx_queue.append(
                    ("PIN_MODES=" + ",".join(str(m) for m in pin_modes)).encode()
                )
                ble_tx_queue.append(
                    ("SW_VALS=" + ",".join(str(b) for b in can_state)).encode()
                )

        # ── スイッチON/OFF値設定  SW_CFG=pin,on,off ──
        elif cmd.startswith("SW_CFG="):
            p = cmd.split('=')[1].split(',')
            pin = int(p[0]); on_v = int(p[1]); off_v = int(p[2])
            if 0 <= pin < NUM_CH:
                sw_on_vals[pin]  = on_v  & 0xFF
                sw_off_vals[pin] = off_v & 0xFF
                if can_state[pin] != sw_on_vals[pin]:
                    can_state[pin] = sw_off_vals[pin]
                nvs_save_all()
                ble_tx_queue.append(
                    f"SW_CFG_OK={pin},{sw_on_vals[pin]},{sw_off_vals[pin]}".encode()
                )

        # ── スイッチ操作  SW_SET=pin,val ─────
        elif cmd.startswith("SW_SET="):
            p = cmd.split('=')[1].split(',')
            pin = int(p[0]); val = int(p[1])
            if 0 <= pin < NUM_CH:
                can_state[pin] = val & 0xFF
                send_sw_can()
                nvs_save_state()
                ble_tx_queue.append(
                    ("SW_VALS=" + ",".join(str(b) for b in can_state)).encode()
                )

    except Exception as e:
        print("BLE CMD Err:", cmd, e)


def send_adc_packet(uart):
    """ADC生値 8ch 分を BLE で送信  AVALS=1023,..."""
    parts = ",".join(str(adc_raw[i]) for i in range(NUM_CH))
    try: uart.send(("AVALS=" + parts).encode())
    except: pass


# ── ディスプレイ (ATOMS3R 128x128px) ───
_disp_row = 0


def disp_line(text, color=0xFFFF):
    global _disp_row
    M5.Display.setCursor(0, _disp_row * 13)
    M5.Display.setTextColor(color, 0x0000)
    M5.Display.print(text[:21])
    _disp_row += 1


def update_display():
    global _disp_row
    M5.Display.clear(0x0000)
    _disp_row = 0
    M5.Display.setTextSize(1)

    can_col = 0xF800 if can_error else 0x07E0
    ble_col = 0x07E0 if uart.connected else 0x8410

    disp_line("= AnaCAN v2.0 =", 0x07FF)
    disp_line(f"CAN:{'ERR' if can_error else ' OK '}", can_col)
    disp_line(f"ADC:{hex(can_tx_id)}", 0xFFFF)
    disp_line(f"SW :{hex(sw_can_id)}", 0xFFFF)
    disp_line(
        f"BLE:{'ON ' if uart.connected else '---'} IO:{'Y' if extio_found else 'N'}",
        ble_col
    )
    disp_line("-" * 21, 0x4208)

    # AUX スイッチ状態
    for i in range(NUM_CH):
        if pin_modes[i] == 0:
            is_on = (can_state[i] == sw_on_vals[i])
            col   = 0x07E0 if is_on else 0xF800
            disp_line(f"P{i}:{'ON ' if is_on else 'OFF'} val={can_state[i]}", col)

    # INPUT ADC値 (上位4ch分)
    shown = 0
    for i in range(NUM_CH):
        if pin_modes[i] == 1 and shown < 4:
            v = adc_raw[i]
            volt = v * 3.3 / 4095.0
            disp_line(f"P{i}:{v:4d} {volt:.1f}V", 0xFFFF)
            shown += 1


# ──────────────────────────────────────────
# 初期化
# ──────────────────────────────────────────
M5.begin()
M5.Display.setTextSize(1)
M5.Display.clear(0x0000)
M5.Display.setTextColor(0x07FF, 0x0000)
M5.Display.print("Booting v2.0...")

# NVS ロード
try:
    nvs = NVS("ana_can")
    try: can_tx_id = nvs.get_i32("can_tx_id")
    except: pass
    try: sw_can_id = nvs.get_i32("sw_can_id")
    except: pass
    for i in range(NUM_CH):
        try: pin_modes[i]   = nvs.get_i32(f"pin_mode_{i}")
        except: pass
        try: sw_on_vals[i]  = nvs.get_i32(f"sw_on_{i}")
        except: pass
        try: sw_off_vals[i] = nvs.get_i32(f"sw_off_{i}")
        except: pass
        try: can_state[i]   = nvs.get_i32(f"sw_state_{i}") & 0xFF
        except: pass
    print(f"NVS: ADC={hex(can_tx_id)} SW={hex(sw_can_id)}")
except Exception as e:
    print("NVS Init Err:", e)

# I2C + EXT.I/O 2
try:
    i2c_bus = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=200000)
    scan = i2c_bus.scan()
    extio_found = EXTIO2_ADDR in scan
    if extio_found:
        apply_pin_modes()
        print("EXT.I/O 2: OK")
    else:
        print("EXT.I/O 2: NOT found", [hex(a) for a in scan])
except Exception as e:
    print("I2C Fail:", e)

# CAN
try:
    can = CANUnit(0, port=(TX_PIN, RX_PIN),
                  mode=CANUnit.NORMAL, baudrate=CAN_BAUDRATE)
    time.sleep_ms(100)
    print("CAN Init OK")
except Exception as e:
    print("CAN Init Fail:", e)
    can_error = True

# BLE
ble  = bluetooth.BLE()
uart = BLEUART(ble)
print("BLE: M5S3R_AnaCAN")

time.sleep_ms(200)
update_display()

# ──────────────────────────────────────────
# メインループ
# ──────────────────────────────────────────
while True:
    M5.update()
    now = time.ticks_ms()

    process_ble_cmd()

    if _pending_cfg:
        _pending_cfg = False
        ble_tx_queue.append(b"__SYNC_DELAY__")

    read_adc_all()

    # CAN 送信 (20Hz)
    if time.ticks_diff(now, last_can_send) >= CAN_SEND_INT:
        if any(pin_modes[i] == 1 for i in range(NUM_CH)):
            safe_can_send(can_tx_id,     build_can_frame(0))
            safe_can_send(can_tx_id + 1, build_can_frame(4))
        if any(pin_modes[i] == 0 for i in range(NUM_CH)):
            send_sw_can()
        last_can_send = now

    # BLE 送信
    if ble_tx_queue:
        item = ble_tx_queue.pop(0)
        if item == b"__SYNC_DELAY__":
            time.sleep_ms(300)
            queue_config_sync()
        else:
            try: uart.send(item)
            except: pass
    elif time.ticks_diff(now, last_ble_send) >= BLE_SEND_INT:
        if uart.connected:
            send_adc_packet(uart)
            ble_tx_queue.append(
                ("SW_VALS=" + ",".join(str(b) for b in can_state)).encode()
            )
            ble_tx_queue.append(
                b"STATUS=ERR" if can_error else b"STATUS=CAN_OK"
            )
        last_ble_send = now

    # ディスプレイ更新 (低頻度)
    if time.ticks_diff(now, last_display) >= DISP_INT:
        check_can_recovery()
        update_display()
        last_display = now

    time.sleep_ms(2)
