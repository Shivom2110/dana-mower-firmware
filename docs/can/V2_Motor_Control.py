"""V2_Motor_Control.py — interactive Dana Motor 1 control.
Run V2_CAN_Test.py successfully first. Startup cleanup only clears software
resources owned by this program; it does not reset physical CAN/inverter faults.
Use normal hardware safety/E-stop provisions during testing.
"""
import time
import can
import cantools

DBC_PATH="GSL_DBC_v0.dbc"
PCAN_CHANNEL="PCAN_USBBUS1"
BITRATE=250000
RPDO_PERIOD=0.010
RPDO_STAGGER=0.001
MIN_COMMAND_RPM=-10000
MAX_COMMAND_RPM=10000

bus=None
rpdo_tasks=[]
rpdo2_task=None
motor_enabled=False

def cleanup_can():
    global bus,rpdo_tasks,rpdo2_task
    for task in rpdo_tasks:
        try: task.stop()
        except Exception as e: print("Task-stop warning:",e)
    rpdo_tasks.clear(); rpdo2_task=None
    if bus is not None:
        try: bus.shutdown()
        except Exception as e: print("PCAN-close warning:",e)
        bus=None

# Fresh software state before opening PCAN.
cleanup_can()

db=cantools.database.load_file(DBC_PATH)

def defaults(msg):
    return {s.name:(s.initial if s.initial is not None else 0) for s in msg.signals}

def create_rpdo(name,updates):
    m=db.get_message_by_name(name)
    values=defaults(m); values.update(updates)
    return can.Message(arbitration_id=m.frame_id,data=m.encode(values),
                       is_extended_id=m.is_extended_frame)

rpdo1=create_rpdo("RPDO1",{"Request_for_Mains_State":8})
rpdo2_values=defaults(db.get_message_by_name("RPDO2"))
rpdo2=create_rpdo("RPDO2",rpdo2_values)
rpdo3=create_rpdo("RPDO3",{})
rpdo4=create_rpdo("RPDO4",{"Batt_Dischg_Curr_Limit":20,"Batt_Chg_Curr_Limit":0})

print("Connecting to PCAN...")
bus=can.Bus(interface="pcan",channel=PCAN_CHANNEL,bitrate=BITRATE)
print("Connected:",PCAN_CHANNEL,"@",BITRATE,"bit/s")

def start_periodic():
    global rpdo2_task
    msgs=(rpdo1,rpdo2,rpdo3,rpdo4)
    for i,msg in enumerate(msgs):
        task=bus.send_periodic(msg,RPDO_PERIOD)
        rpdo_tasks.append(task)
        if i==1: rpdo2_task=task
        if i<3: time.sleep(RPDO_STAGGER)

def update_rpdo2(updates):
    global rpdo2
    rpdo2_values.update(updates)
    rpdo2=create_rpdo("RPDO2",rpdo2_values)
    if rpdo2_task is None: raise RuntimeError("RPDO2 task not started")
    rpdo2_task.modify_data(rpdo2)
    print("RPDO2 update:",updates,"|",rpdo2.data.hex(" "))

def enable_motor():
    global motor_enabled
    update_rpdo2({"Motor1_ControlModeReq":1,
                  "Motor1_PWMOutEnableReq":1,
                  "Motor1_RefSpeedTorque_EnableReq":1,
                  "Motor1_SpeedRef_Lim":0,
                  "Motor1_TorqueRef_Lim":50})
    motor_enabled=True
    print("Motor command enabled at 0 RPM.")

def shutdown_motor():
    global motor_enabled
    if not motor_enabled or rpdo2_task is None: return
    print("\nRequesting motor shutdown...")
    update_rpdo2({"Motor1_SpeedRef_Lim":0})
    # Software delay only; not proof that the drivetrain has physically stopped.
    time.sleep(0.5)
    update_rpdo2({"Motor1_RefSpeedTorque_EnableReq":0,
                  "Motor1_PWMOutEnableReq":0})
    time.sleep(0.1)
    motor_enabled=False

def rpm_loop():
    while True:
        try:
            text=input("Enter desired RPM (q or Ctrl+C to stop): ").strip()
            if text.lower()=="q": return
            rpm=int(text)
            if not MIN_COMMAND_RPM <= rpm <= MAX_COMMAND_RPM:
                print("RPM outside configured range.")
                continue
            update_rpdo2({"Motor1_SpeedRef_Lim":rpm})
        except ValueError:
            print("Enter an integer RPM or q.")
        except KeyboardInterrupt:
            print("\nCtrl+C detected.")
            return

try:
    start_periodic()
    print("RPDO1-RPDO4 transmitting every 10 ms with 1 ms start stagger.")
    enable_motor()
    rpm_loop()
except KeyboardInterrupt:
    print("\nCtrl+C detected.")
finally:
    try: shutdown_motor()
    except Exception as e: print("Motor-shutdown warning:",e)
    cleanup_can()
    print("Periodic tasks stopped; PCAN closed.")
