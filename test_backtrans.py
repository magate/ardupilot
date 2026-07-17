import time
import argparse
from pymavlink import mavutil

def send_rc_continuous(master, rc3_value, duration_sec):
    """Sends RC override continuously for the given duration to prevent ArduPilot from timing out the RC signal."""
    start = time.time()
    while time.time() - start < duration_sec:
        # Send RC3 (throttle), and center roll/pitch/yaw (1500)
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            1500, 1500, rc3_value, 1500, 0, 0, 0, 0
        )
        time.sleep(0.5)

def send_rc_until_alt(master, rc3_value, target_alt, is_climbing=True, tolerance=0.5, max_timeout=90):
    """Continuously sends RC throttle until the vehicle reaches *target_alt* ± *tolerance*.
    *is_climbing* – True for ascents, False for descents.
    *max_timeout* – How long (seconds) we will wait before giving up.
    """
    start = time.time()
    while time.time() - start < max_timeout:
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            1500, 1500, rc3_value, 1500, 0, 0, 0, 0
        )
        msg = master.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            alt = msg.alt
            # Use tolerance so we don't get stuck at 29.0 when target is 30.0
            if is_climbing and alt >= target_alt - tolerance:
                print(f"Altitude reached (or within tolerance): {alt:.1f}m")
                return
            elif not is_climbing and alt <= target_alt + tolerance:
                print(f"Altitude reached (or within tolerance): {alt:.1f}m")
                return
        time.sleep(0.1)
    print(f"WARNING: Altitude target {target_alt}±{tolerance}m not reached within {max_timeout}s.")

def set_mode(master, mode_name):
    print(f"mode {mode_name.lower()}")
    if mode_name == 'QHOVER':
        master.set_mode_apm(18)
    elif mode_name == 'FBWA':
        master.set_mode_apm(5)
    elif mode_name == 'QLOITER':
        master.set_mode_apm(19)
    else:
        print("Unknown mode")
    time.sleep(1)

def run_tests(q_backtrans_ms):
    print("Connecting to SITL...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    master.wait_heartbeat()
    
    print("param set ARMING_CHECK 0")
    master.mav.param_set_send(
        master.target_system, 1,
        b'ARMING_CHECK',
        0.0,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(1)
    
    print(f"\nPhase 3: Change the Parameter")
    print(f"param set Q_BACKTRANS_MS {q_backtrans_ms}")
    master.mav.param_set_send(
        master.target_system, 1,
        b'Q_BACKTRANS_MS',
        float(q_backtrans_ms),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(2)
    
    print("\nPhase 4: Arm and Take Off (VTOL Mode)")
    set_mode(master, 'QHOVER')
    
    print("arm throttle")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("ARMED")
    
    print("rc 3 1700 (Climbing to ~50m)...")
    send_rc_until_alt(master, 1700, 49.0, is_climbing=True)
    
    print("rc 3 1500 (Hovering for 5s)...")
    send_rc_continuous(master, 1500, 5)
    
    print("\nPhase 5: Forward Transition (VTOL -> Fixed-Wing)")
    set_mode(master, 'FBWA')
    print("Waiting 20 seconds for forward transition...")
    send_rc_continuous(master, 1500, 20)
    
    print("\nPhase 6: Back Transition (Fixed-Wing -> VTOL)")
    set_mode(master, 'QLOITER')
    
    wait_time = (q_backtrans_ms / 1000.0) + 20.0
    print(f"Waiting {wait_time} seconds for back transition...")
    send_rc_continuous(master, 1500, wait_time)
    
    print("\nPhase 7: Land and Disarm")
    print("rc 3 1300 (Descending to ~3m, or max 20s if it hits a hill)...")
    send_rc_until_alt(master, 1300, 3.0, is_climbing=False, max_timeout=20)
    
    print("rc 3 1000 (Touchdown for 5s)...")
    send_rc_continuous(master, 1000, 5)
    
    print("disarm (forcing to ensure log saves)")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196.0, 0, 0, 0, 0, 0
    )
    time.sleep(2)
    print("DISARMED")
    
    print("\n✅ Simulation flight complete.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ms', type=int, default=4000)
    args = parser.parse_args()
    if not (0 <= args.ms <= 65535):
        raise argparse.ArgumentTypeError("ms must be in 0-65535")
    run_tests(args.ms)
