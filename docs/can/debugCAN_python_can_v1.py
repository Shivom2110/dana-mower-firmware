"""
debugCAN_python_can_v1.py
Interactive VS Code / Python-cell version of Jens' debugCAN.m

IMPORTANT:
- This file is structured as VS Code cells using "# %%".
- Run ONE CELL AT A TIME in the order described below.
- Do NOT run the entire file at once.
- Confirm the CAN bitrate before opening the bus.
- Keep the mower/wheels safely restrained and motor mechanically unloaded for bench testing.

Requires:
    pip install python-can cantools

Also requires:
    - PEAK PCAN driver
    - GSL_DBC_v0.dbc in the same folder (or update DBC_PATH)
    - PEAK PCAN-USB adapter
"""

# %% CELL 1 — IMPORTS + SETTINGS
import time
import threading
import can
import cantools

DBC_PATH = "GSL_DBC_v0.dbc"
PCAN_CHANNEL = "PCAN_USBBUS1"

# IMPORTANT:
# Jens' MATLAB file DISPLAYED the existing bus speed.
# Its 250000 configuration line was commented out.
# Put the confirmed inverter CAN bitrate here before running Cell 3.
BITRATE = 250000
BITRATE_CONFIRMED = False

TX_PERIOD_S = 0.01  # 10 ms

print("Cell 1 complete.")
print("Before Cell 3, confirm BITRATE and set BITRATE_CONFIRMED = True.")


# %% CELL 2 — LOAD DBC + INSPECT RPDO MESSAGES
db = cantools.database.load_file(DBC_PATH)

print("DBC loaded:", DBC_PATH)

for name in ["RPDO1", "RPDO2", "RPDO3", "RPDO4", "TPDO4"]:
    try:
        msg_def = db.get_message_by_name(name)
        print(
            f"{name}: ID=0x{msg_def.frame_id:X}, "
            f"length={msg_def.length}, "
            f"signals={[s.name for s in msg_def.signals]}"
        )
    except KeyError:
        print(f"{name}: NOT FOUND in DBC")

print("Cell 2 complete. Nothing has been transmitted.")


# %% CELL 3 — OPEN PEAK CAN CHANNEL
if not BITRATE_CONFIRMED:
    raise RuntimeError(
        "BITRATE has not been confirmed. "
        "Confirm the inverter CAN bitrate, set BITRATE_CONFIRMED = True, "
        "then rerun this cell."
    )

bus = can.Bus(
    interface="pcan",
    channel=PCAN_CHANNEL,
    bitrate=BITRATE,
)

print(f"Connected to {PCAN_CHANNEL} at {BITRATE} bit/s.")


# %% CELL 4 — HELPER FUNCTIONS
def default_signal_values(message_name):
    """
    Build a value dictionary for every signal in a DBC message.

    This helps mimic MATLAB canMessage(...), where the message object exists
    before Jens changes selected signals.
    """
    msg_def = db.get_message_by_name(message_name)

    values = {}
    for signal in msg_def.signals:
        if signal.initial is not None:
            values[signal.name] = signal.initial
        else:
            values[signal.name] = 0

    return values


def encode_message(message_name, signal_values):
    msg_def = db.get_message_by_name(message_name)

    data = msg_def.encode(
        signal_values,
        strict=False
    )

    return can.Message(
        arbitration_id=msg_def.frame_id,
        data=data,
        is_extended_id=msg_def.is_extended_frame,
    )


def decode_message(msg):
    try:
        msg_def = db.get_message_by_frame_id(msg.arbitration_id)
        decoded = msg_def.decode(
            msg.data,
            decode_choices=False
        )
        return msg_def.name, decoded
    except Exception:
        return None, None


print("Cell 4 complete.")


# %% CELL 5 — CREATE RPDO1-RPDO4 VALUES
# Start from DBC defaults / zero values.
rpdo1_values = default_signal_values("RPDO1")
rpdo2_values = default_signal_values("RPDO2")
rpdo3_values = default_signal_values("RPDO3")
rpdo4_values = default_signal_values("RPDO4")

# Values explicitly set in Jens' MATLAB file.
rpdo1_values["Request_for_Mains_State"] = 8

rpdo4_values["Batt_Dischg_Curr_Limit"] = 20
rpdo4_values["Batt_Chg_Curr_Limit"] = 0

# Keep the motor-control requests OFF initially.
for key in [
    "Motor1_ControlModeReq",
    "Motor1_PWMOutEnableReq",
    "Motor1_RefSpeedTorque_EnableReq",
    "Motor1_SpeedRef_Lim",
    "Motor1_TorqueRef_Lim",
    "Motor1_SpeedLimitReq",
    "Motor1_EmergencyStopReq",
    "Motor1_SafetStopReq",
]:
    if key in rpdo2_values:
        rpdo2_values[key] = 0

print("RPDO values created.")
print("RPDO1 Request_for_Mains_State =", rpdo1_values.get("Request_for_Mains_State"))
print("RPDO4 discharge limit =", rpdo4_values.get("Batt_Dischg_Curr_Limit"))
print("RPDO4 charge limit =", rpdo4_values.get("Batt_Chg_Curr_Limit"))
print("Cell 5 complete. Nothing has been transmitted yet.")


# %% CELL 6 — DEFINE THE BACKGROUND PERIODIC SENDER
# Running this cell only DEFINES the sender.
# It does not start transmission yet.

tx_stop_event = threading.Event()
tx_thread = None
tx_lock = threading.Lock()


def periodic_sender():
    """
    Continuously transmit RPDO1, RPDO2, RPDO3, RPDO4.

    The dictionaries are reread every cycle, so later cells can change
    RPDO2 values while this sender keeps running.
    """
    next_cycle = time.perf_counter()

    while not tx_stop_event.is_set():
        with tx_lock:
            frames = [
                encode_message("RPDO1", dict(rpdo1_values)),
                encode_message("RPDO2", dict(rpdo2_values)),
                encode_message("RPDO3", dict(rpdo3_values)),
                encode_message("RPDO4", dict(rpdo4_values)),
            ]

        for frame in frames:
            bus.send(frame)

        next_cycle += TX_PERIOD_S
        delay = next_cycle - time.perf_counter()

        if delay > 0:
            time.sleep(delay)
        else:
            # If the PC falls behind, restart timing instead of
            # trying to "catch up" with a burst of frames.
            next_cycle = time.perf_counter()


print("Cell 6 complete. Periodic sender defined but NOT started.")


# %% CELL 7 — START RPDO1-RPDO4 PERIODIC TRANSMISSION
# Run this ONCE after CAN is confirmed healthy.
# RPDO1-RPDO4 will continue in the background while later cells run.

if tx_thread is not None and tx_thread.is_alive():
    print("Periodic RPDO sender is already running.")
else:
    tx_stop_event.clear()
    tx_thread = threading.Thread(
        target=periodic_sender,
        daemon=True
    )
    tx_thread.start()
    print("Periodic RPDO1-RPDO4 transmission STARTED.")
    print("They are being refreshed approximately every 10 ms.")


# %% CELL 8 — RECEIVE / DECODE FEEDBACK FOR A FEW SECONDS
# Use this immediately after Cell 7 and whenever you want to inspect feedback.

MONITOR_SECONDS = 5.0

end_time = time.monotonic() + MONITOR_SECONDS

print(f"Listening for CAN feedback for {MONITOR_SECONDS:.1f} seconds...")

while time.monotonic() < end_time:
    msg = bus.recv(timeout=0.2)

    if msg is None:
        continue

    name, signals = decode_message(msg)

    if name is not None:
        print(
            f"RX 0x{msg.arbitration_id:X}  "
            f"{name}: {signals}"
        )
    else:
        print(
            f"RX 0x{msg.arbitration_id:X}  "
            f"RAW: {msg.data.hex(' ')}"
        )

print("Feedback monitor finished.")


# %% CELL 9 — REQUEST PWM ENABLE ONLY
# IMPORTANT:
# Do this only after:
# - CAN is healthy
# - no Bus-Off condition
# - required encoder commissioning/faults are resolved
# - expected mains/contactor state is verified
#
# This does NOT yet enable the speed/torque reference.

with tx_lock:
    rpdo2_values["Motor1_ControlModeReq"] = 1
    rpdo2_values["Motor1_PWMOutEnableReq"] = 1

    # Keep actual speed/torque reference disabled for now.
    rpdo2_values["Motor1_RefSpeedTorque_EnableReq"] = 0
    rpdo2_values["Motor1_SpeedRef_Lim"] = 0
    rpdo2_values["Motor1_TorqueRef_Lim"] = 0

    rpdo2_values["Motor1_SpeedLimitReq"] = 0
    rpdo2_values["Motor1_EmergencyStopReq"] = 0
    rpdo2_values["Motor1_SafetStopReq"] = 0

print("PWM enable REQUEST set in RPDO2.")
print("Waiting for TPDO4 confirmation should be the next step.")


# %% CELL 10 — WAIT FOR TPDO4: Motor1_PWM_Output == 1
PWM_CONFIRM_TIMEOUT_S = 10.0

confirmation_received = False
deadline = time.monotonic() + PWM_CONFIRM_TIMEOUT_S

print("Waiting for TPDO4 PWM confirmation...")

while time.monotonic() < deadline:
    msg = bus.recv(timeout=0.1)

    if msg is None:
        continue

    name, signals = decode_message(msg)

    if name == "TPDO4":
        print("TPDO4:", signals)

        if signals.get("Motor1_PWM_Output") == 1:
            confirmation_received = True
            print("PWM OUTPUT CONFIRMED by TPDO4.")
            break

if not confirmation_received:
    print(
        "NO PWM CONFIRMATION RECEIVED. "
        "Do NOT run the speed-enable cell."
    )


# %% CELL 11 — ENABLE SPEED/TORQUE REFERENCE
# Run ONLY if Cell 10 printed:
# "PWM OUTPUT CONFIRMED by TPDO4."

if not confirmation_received:
    raise RuntimeError(
        "PWM was not confirmed. Speed/torque reference will NOT be enabled."
    )

with tx_lock:
    rpdo2_values["Motor1_RefSpeedTorque_EnableReq"] = 1

print("Speed/torque reference ENABLE request set.")


# %% CELL 12 — APPLY LOW TEST SPEED / TORQUE LIMIT
# Jens' MATLAB example used 200 rpm and 50%.
# These are copied from his debug file; choose values appropriate for the
# controlled bench setup and approved test procedure.

TEST_SPEED_RPM = 200
TEST_TORQUE_LIMIT_PCT = 50

if not confirmation_received:
    raise RuntimeError(
        "PWM was not confirmed. Speed command blocked."
    )

with tx_lock:
    rpdo2_values["Motor1_SpeedRef_Lim"] = TEST_SPEED_RPM
    rpdo2_values["Motor1_TorqueRef_Lim"] = TEST_TORQUE_LIMIT_PCT

print(
    f"RPDO2 speed reference set to {TEST_SPEED_RPM} rpm, "
    f"torque-reference limit set to {TEST_TORQUE_LIMIT_PCT}%."
)


# %% CELL 13 — STOP / DISABLE MOTOR
# Use this before stopping periodic CAN transmission.

with tx_lock:
    rpdo2_values["Motor1_ControlModeReq"] = 0
    rpdo2_values["Motor1_PWMOutEnableReq"] = 0
    rpdo2_values["Motor1_RefSpeedTorque_EnableReq"] = 0
    rpdo2_values["Motor1_SpeedRef_Lim"] = 0
    rpdo2_values["Motor1_TorqueRef_Lim"] = 0
    rpdo2_values["Motor1_SpeedLimitReq"] = 0
    rpdo2_values["Motor1_EmergencyStopReq"] = 0
    rpdo2_values["Motor1_SafetStopReq"] = 0

print("Motor STOP / DISABLE values loaded into RPDO2.")
print("Periodic RPDO transmission is still running.")


# %% CELL 14 — OPTIONAL: MONITOR FEEDBACK AFTER STOP
MONITOR_SECONDS = 3.0

end_time = time.monotonic() + MONITOR_SECONDS

print(f"Monitoring feedback for {MONITOR_SECONDS:.1f} seconds...")

while time.monotonic() < end_time:
    msg = bus.recv(timeout=0.2)

    if msg is None:
        continue

    name, signals = decode_message(msg)

    if name is not None:
        print(
            f"RX 0x{msg.arbitration_id:X}  "
            f"{name}: {signals}"
        )

print("Post-stop monitor finished.")


# %% CELL 15 — STOP PERIODIC RPDO TRANSMISSION
tx_stop_event.set()

if tx_thread is not None:
    tx_thread.join(timeout=1.0)

print("Periodic RPDO transmission STOPPED.")


# %% CELL 16 — CLOSE CAN INTERFACE
bus.shutdown()
print("PEAK CAN interface closed.")
