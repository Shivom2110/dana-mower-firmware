"""V2_CAN_Test.py — CAN-only Dana inverter diagnostic.
Startup cleanup only clears software resources owned by this program; it does
not reset the physical CAN bus or inverter faults.
"""
import time
import can
import cantools

DBC_PATH="GSL_DBC_v0.dbc"
PCAN_CHANNEL="PCAN_USBBUS1"
BITRATE=250000
RPDO_PERIOD=0.010
RPDO_STAGGER=0.001

bus=None
rpdo_tasks=[]

def cleanup_can():
    global bus, rpdo_tasks
    for task in rpdo_tasks:
        try: task.stop()
        except Exception as e: print("Task-stop warning:", e)
    rpdo_tasks.clear()
    if bus is not None:
        try: bus.shutdown()
        except Exception as e: print("PCAN-close warning:", e)
        bus=None

# Fresh software state before opening PCAN.
cleanup_can()

db=cantools.database.load_file(DBC_PATH)
print("DBC loaded:",DBC_PATH)

def defaults(msg):
    return {s.name:(s.initial if s.initial is not None else 0) for s in msg.signals}

def create_rpdo(name,updates):
    m=db.get_message_by_name(name)
    values=defaults(m); values.update(updates)
    return can.Message(arbitration_id=m.frame_id,data=m.encode(values),
                       is_extended_id=m.is_extended_frame)

rpdo1=create_rpdo("RPDO1",{"Request_for_Mains_State":8})
rpdo2=create_rpdo("RPDO2",{})
rpdo3=create_rpdo("RPDO3",{})
rpdo4=create_rpdo("RPDO4",{"Batt_Dischg_Curr_Limit":20,"Batt_Chg_Curr_Limit":0})

print("Connecting to PCAN...")
bus=can.Bus(interface="pcan",channel=PCAN_CHANNEL,bitrate=BITRATE)
print("Connected:",PCAN_CHANNEL,"@",BITRATE,"bit/s")

def start_periodic():
    for i,msg in enumerate((rpdo1,rpdo2,rpdo3,rpdo4)):
        task=bus.send_periodic(msg,RPDO_PERIOD)
        rpdo_tasks.append(task)
        if i<3: time.sleep(RPDO_STAGGER)

try:
    for m in (rpdo1,rpdo2,rpdo3,rpdo4):
        print("TX prepared:",hex(m.arbitration_id),m.data.hex(" "))
    start_periodic()
    print("\nCAN-ONLY TEST RUNNING — no PWM/RPM motor command.")
    print("Press Ctrl+C to stop.\n")
    while True:
        msg=bus.recv(timeout=1.0)
        if msg is None:
            print("No CAN message received.")
        else:
            print("RX:",hex(msg.arbitration_id),msg.data.hex(" "))
except KeyboardInterrupt:
    print("\nStop requested.")
finally:
    cleanup_can()
    print("Periodic tasks stopped; PCAN closed.")
