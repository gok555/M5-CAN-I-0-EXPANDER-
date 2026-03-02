# ==========================================
# ATOM S3R + Atomic CAN Base + EXT.I/O 2
# Ver 2.6 - CAN RX改善 + 設定永続化
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
CAN_BAUDRATE = 1000000  # デフォルト 1Mbps (NVSで変更可能)
# 対応ボーレート: 125000, 250000, 500000, 1000000
I2C_SCL_PIN = 1
I2C_SDA_PIN = 2
EXTIO2_ADDR = 0x45

# ── EXT.I/O 2 レジスタマップ ──────────────
# 公式: github.com/m5stack/M5Unit-ExtIO2
REG_MODE_BASE  = 0x00  # モード設定レジスタ
REG_ADC_BASE   = 0x40  # 12bit ADC (公式仕様: 0x40-0x4F, 2byte/ch)
REG_OUT_BASE   = 0x10  # +pin      : デジタル出力
EXTIO2_MODE_OUTPUT = 0x01  # デジタル出力
EXTIO2_MODE_ADC    = 0x02  # ★ ADCモード (実機確認済み、4ではなく2)

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

# ── CAN受信 AUX制御 ───────────────────────
can_rx_id    = 0x600      # EMTRONからの受信CAN ID (ユーザー設定可能)
rx_on_vals   = [1] * NUM_CH  # 受信値がこれならAUX ON
rx_off_vals  = [0] * NUM_CH  # 受信値がこれならAUX OFF
can_lock_until  = 0       # CAN受信後、この時刻までBLE SW_SETを無視
_can_rx_nvsave  = False   # CAN RX後のNVS保存フラグ
CAN_LOCK_MS  = 2000       # CAN優先ロック時間 (ms)
can_endian   = 0          # 0=リトルエンディアン, 1=ビッグエンディアン

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
        # アドバタイズパケット構造:
        #   [0x02, 0x01, 0x06]           Flags (LE General Discoverable, BR/EDR Not Supported)
        #   [0x0D, 0x09, <name 12bytes>] Complete Local Name (length = 1+12 = 13 = 0x0D)
        # ※ 以前は length=0x10(16) で 3 バイト多く、スマホ側でデバイス名を認識できなかった
        name = b'M5S3R_AnaCAN'
        adv = bytes([0x02, 0x01, 0x06,
                     len(name) + 1, 0x09]) + name
        try:
            self._ble.gap_advertise(100_000, adv)
        except Exception as e:
            print("BLE advertise error:", e)

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
        nvs.set_i32("can_rx_id", can_rx_id)
        nvs.set_i32("can_baud", CAN_BAUDRATE)
        nvs.set_i32("can_endian", can_endian)
        for i in range(NUM_CH):
            nvs.set_i32(f"pin_mode_{i}", pin_modes[i])
            nvs.set_i32(f"sw_on_{i}",   sw_on_vals[i])
            nvs.set_i32(f"sw_off_{i}",  sw_off_vals[i])
            nvs.set_i32(f"sw_state_{i}", int(can_state[i]))
            nvs.set_i32(f"rx_on_{i}",   rx_on_vals[i])
            nvs.set_i32(f"rx_off_{i}",  rx_off_vals[i])
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
    if not extio_found: return
    for i in range(NUM_CH):
        hw_mode = EXTIO2_MODE_ADC if pin_modes[i] == 1 else EXTIO2_MODE_OUTPUT
        try:
            i2c_bus.writeto_mem(EXTIO2_ADDR, REG_MODE_BASE + i, bytes([hw_mode]))
            time.sleep_ms(10)
        except Exception as e:
            print(f"apply_pin_modes P{i} err: {e}")
    time.sleep_ms(50)


_adc_log_count = 0
def read_adc_all():
    """INPUT チャンネルの ADC 値を取得 (12bit, 0-4095)"""
    global _adc_log_count
    if not extio_found: return
    for i in range(NUM_CH):
        if pin_modes[i] != 1:
            adc_raw[i] = 0
            continue
        try:
            reg = REG_ADC_BASE + i * 2  # 公式仕様: 2byte aligned
            i2c_bus.writeto(EXTIO2_ADDR, bytes([reg]))
            time.sleep_ms(1)
            d = i2c_bus.readfrom(EXTIO2_ADDR, 2)
            if len(d) == 2:
                adc_raw[i] = (d[0] | (d[1] << 8)) & 0x0FFF
            else:
                adc_raw[i] = 0
        except Exception as e:
            adc_raw[i] = 0
    # 起動後最初の10回はシリアルにも出力して確認
    _adc_log_count += 1
    if _adc_log_count <= 10:
        print(f"ADC raw: {adc_raw[:4]}")


# ── CAN ──────────────────────────────────
def build_can_frame(start_ch):
    """4ch分を8バイトCANフレームへパック (LE/BE切替対応, 12bit)"""
    buf = bytearray(8)
    for j in range(4):
        val = adc_raw[start_ch + j] & 0x0FFF
        if can_endian == 0:  # リトルエンディアン
            buf[j * 2]     = val & 0xFF
            buf[j * 2 + 1] = (val >> 8) & 0x0F
        else:                # ビッグエンディアン
            buf[j * 2]     = (val >> 8) & 0x0F
            buf[j * 2 + 1] = val & 0xFF
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
    # EXT.I/O 2 ピンモード定期再適用 (STM32リセット対策)
    apply_pin_modes()
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


def process_can_rx():
    """CAN受信 → AUX制御 (CAN優先)"""
    global can_lock_until, _can_rx_nvsave
    if not can: return
    try:
        # 1ループで最大4フレームまで処理(ループブロック防止)
        for _ in range(4):
            if not can.any(0): break
            msg = can.recv(0)
            if not msg or len(msg) < 5: continue
            rx_id = msg[0]
            if rx_id != can_rx_id: continue
            data = msg[4]
            changed = False
            for i in range(NUM_CH):
                if pin_modes[i] != 0: continue
                if i >= len(data): continue
                v = data[i]
                if v == rx_on_vals[i]:
                    can_state[i] = sw_on_vals[i] & 0xFF
                    changed = True
                elif v == rx_off_vals[i]:
                    can_state[i] = sw_off_vals[i] & 0xFF
                    changed = True
            if changed:
                can_lock_until = time.ticks_add(time.ticks_ms(), CAN_LOCK_MS)
                send_sw_can()
                _can_rx_nvsave = True  # NVS保存はメインループで後処理
                ble_tx_queue.append(
                    ("SW_VALS=" + ",".join(str(b) for b in can_state)).encode()
                )
                ble_tx_queue.append(
                    ("RX_ACT=" + ",".join(str(b) for b in can_state)).encode()
                )
    except Exception as e:
        pass  # CAN RX エラーは無視


# ── BLE コマンド処理 ──────────────────────
def queue_config_sync():
    """接続時に全設定をクライアントへ送信"""
    # I2C / EXT.IO2 状態を通知
    io_status = "IO:OK" if extio_found else "IO:NG(EXT.IO2未検出)"
    ble_tx_queue.append(f"LOG={io_status}".encode())
    ble_tx_queue.append(f"TXID={hex(can_tx_id)}".encode())
    ble_tx_queue.append(f"SW_TXID={hex(sw_can_id)}".encode())
    ble_tx_queue.append(f"RX_ID={hex(can_rx_id)}".encode())
    ble_tx_queue.append(f"ENDIAN={can_endian}".encode())
    ble_tx_queue.append(f"BAUD={CAN_BAUDRATE}".encode())
    ble_tx_queue.append(("PIN_MODES=" + ",".join(str(m) for m in pin_modes)).encode())
    ble_tx_queue.append(("SW_ON="  + ",".join(str(v) for v in sw_on_vals)).encode())
    ble_tx_queue.append(("SW_OFF=" + ",".join(str(v) for v in sw_off_vals)).encode())
    ble_tx_queue.append(("RX_ON="  + ",".join(str(v) for v in rx_on_vals)).encode())
    ble_tx_queue.append(("RX_OFF=" + ",".join(str(v) for v in rx_off_vals)).encode())
    ble_tx_queue.append(("SW_VALS=" + ",".join(str(b) for b in can_state)).encode())


def process_ble_cmd():
    global can_tx_id, sw_can_id, can_rx_id, can_endian, CAN_BAUDRATE, pin_modes, sw_on_vals, sw_off_vals, rx_on_vals, rx_off_vals
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
                # CAN優先ロック中はBLE SW_SETを無視
                if time.ticks_diff(time.ticks_ms(), can_lock_until) < 0:
                    ble_tx_queue.append(b"SW_LOCK=CAN")
                else:
                    can_state[pin] = val & 0xFF
                    send_sw_can()
                    nvs_save_state()
                    ble_tx_queue.append(
                        ("SW_VALS=" + ",".join(str(b) for b in can_state)).encode()
                    )

        # ── CAN受信ID設定  SET_RX_ID=0x600 ──
        elif cmd.startswith("SET_RX_ID="):
            can_rx_id = int(cmd.split('=')[1], 16)
            nvs_save_all()
            print("SET_RX_ID: " + hex(can_rx_id))
            ble_tx_queue.append(f"RX_ID={hex(can_rx_id)}".encode())

        # ── CAN受信 ON値設定  RX_ON=pin,val ─
        elif cmd.startswith("RX_ON="):
            p = cmd.split('=')[1].split(',')
            pin = int(p[0]); val = int(p[1])
            if 0 <= pin < NUM_CH:
                rx_on_vals[pin] = val
                nvs_save_all()
                ble_tx_queue.append(f"RX_ON_OK={pin},{val}".encode())

        # ── CAN受信 OFF値設定  RX_OFF=pin,val
        elif cmd.startswith("RX_OFF="):
            p = cmd.split('=')[1].split(',')
            pin = int(p[0]); val = int(p[1])
            if 0 <= pin < NUM_CH:
                rx_off_vals[pin] = val
                nvs_save_all()
                ble_tx_queue.append(f"RX_OFF_OK={pin},{val}".encode())

        # ── エンディアン設定  SET_ENDIAN=0(LE) / SET_ENDIAN=1(BE)
        elif cmd.startswith("SET_ENDIAN="):
            can_endian = int(cmd.split('=')[1]) & 1
            nvs_save_all()
            ble_tx_queue.append(f"ENDIAN={can_endian}".encode())

        # ── CANボーレート変更  SET_BAUD=1000000 ──
        elif cmd.startswith("SET_BAUD="):
            baud = int(cmd.split('=')[1])
            if baud in [125000, 250000, 500000, 1000000]:
                CAN_BAUDRATE = baud
                nvs_save_all()
                # CANを再初期化
                try:
                    if can: can.deinit()
                    time.sleep_ms(50)
                    can = CANUnit(0, port=(TX_PIN, RX_PIN),
                                  mode=CANUnit.NORMAL, baudrate=CAN_BAUDRATE)
                    can_error = False
                    print("CAN reinit: " + str(CAN_BAUDRATE) + "bps OK")
                except Exception as e:
                    can_error = True
                    print("CAN reinit ERR: " + str(e))
                ble_tx_queue.append(f"BAUD={CAN_BAUDRATE}".encode())

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
    baud_k = CAN_BAUDRATE // 1000
    disp_line(f"BAUD:{baud_k}k {'LE' if can_endian==0 else 'BE'}", 0xFFFF)
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
M5.Display.print("Booting v2.6...")

# NVS ロード
try:
    nvs = NVS("ana_can")
    try: can_tx_id = nvs.get_i32("can_tx_id")
    except: pass
    try: sw_can_id = nvs.get_i32("sw_can_id")
    except: pass
    try: can_rx_id = nvs.get_i32("can_rx_id")
    except: pass
    try: CAN_BAUDRATE = nvs.get_i32("can_baud")
    except: pass
    try: can_endian = nvs.get_i32("can_endian")
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
        try: rx_on_vals[i]  = nvs.get_i32(f"rx_on_{i}")
        except: pass
        try: rx_off_vals[i] = nvs.get_i32(f"rx_off_{i}")
        except: pass
    print(f"NVS: ADC={hex(can_tx_id)} SW={hex(sw_can_id)}")
except Exception as e:
    print("NVS Init Err:", e)

# I2C + EXT.I/O 2
try:
    i2c_bus = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=100000)
    time.sleep_ms(100)
    scan = i2c_bus.scan()
    found_addrs = [hex(a) for a in scan]
    print(f"I2C scan: {found_addrs}")
    extio_found = EXTIO2_ADDR in scan
    if extio_found:
        print("EXT.I/O 2: Found at 0x45 ✅")
        time.sleep_ms(200)    # STM32起動待ち
        apply_pin_modes()
        time.sleep_ms(300)    # STM32 ADCモード確定待ち
        print("EXT.I/O 2: ADCモード設定完了")
    else:
        print(f"EXT.I/O 2: NOT found ❌  scan={found_addrs}")
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
print("=" * 32)
print("BLE アドバタイズ開始")
print("  デバイス名: M5S3R_AnaCAN")
print("  接続手順:")
print("  1. Chrome(PC/Android) HTTPS で開く")
print("  2. CONNECT ボタンを押す")
print("  3. M5S3R_AnaCAN を選択")
print("=" * 32)

time.sleep_ms(200)
update_display()

# ──────────────────────────────────────────
# メインループ
# ──────────────────────────────────────────
while True:
    M5.update()
    now = time.ticks_ms()

    process_can_rx()   # CAN受信 (EMTRON → AUX制御, CAN優先)

    # CAN RX後のNVS保存を遅延実行(メインループ負荷軽減)
    if _can_rx_nvsave:
        _can_rx_nvsave = False
        nvs_save_state()

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
