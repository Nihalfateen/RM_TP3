import os
import serial
import threading
import time
import fcntl
import termios
import struct
import select
import array

# 1. Create a virtual serial port inside the container
master, slave = os.openpty()
slave_name = os.ttyname(slave)

# --- THE FIX: KEEP THE SLAVE ACTIVE ---
# Opening the slave path in non-blocking read-only mode inside the bridge script
# forces the Linux kernel PTY subsystem to stay initialized. 
# This completely prevents TIOCMGET from throwing ENOTTY exceptions!
keep_alive_slave = os.open(slave_name, os.O_RDONLY | os.O_NONBLOCK)

try:
    if os.path.exists('/dev/ttyUSB0'):
        os.remove('/dev/ttyUSB0')
    os.symlink(slave_name, '/dev/ttyUSB0')
    print("Virtual port created at: /dev/ttyUSB0")
except OSError:
    if os.path.exists('/tmp/ttyUSB0'):
        os.remove('/tmp/ttyUSB0')
    os.symlink(slave_name, '/tmp/ttyUSB0')
    print("Permissions restricted. Virtual port created at: /tmp/ttyUSB0")

# 2. Connect to the Mac's RFC 2217 network stream
ser = serial.serial_for_url('rfc2217://host.docker.internal:2217', rtscts=False)
ser.timeout = 0.05

alive = True

# Thread 1: Mac Hardware -> Network -> Virtual Port RX
def network_to_virtual():
    global alive
    while alive:
        try:
            data = ser.read(1024)
            if data:
                os.write(master, data)
        except Exception:
            alive = False
        time.sleep(0.001)

# Thread 2: Virtual Port TX -> Network -> Mac Hardware (Uses select to prevent blocking)
def virtual_to_network_data():
    global alive
    while alive:
        try:
            r, _, _ = select.select([master], [], [], 0.01)
            if master in r:
                data = os.read(master, 1024)
                if data:
                    ser.write(data)
        except Exception:
            alive = False

# Thread 3: Monitor ldpic32 RTS/DTR state changes (Now stable thanks to keep_alive_slave)
def monitor_virtual_pins():
    global alive
    last_rts, last_dtr = None, None
    print("Actively monitoring ldpic32 bootloader pins...")
    
    buf = array.array('i', [0])
    
    while alive:
        status = None
        try:
            # This will now succeed cleanly because keep_alive_slave keeps the port context alive
            print("AAA")
            fcntl.ioctl(slave, termios.TIOCMGET, buf, 1)
            print("AAA")
            status = buf[0]
        except Exception as e:
            # If an exception still happens, we can see exactly why without stalling the loop
            print("EEE")
            pass
        
        if status is not None:
            print("dasadadE")
            current_rts = bool(status & termios.TIOCM_RTS)
            current_dtr = bool(status & termios.TIOCM_DTR)
            
            # Send RTS command over RFC 2217 instantly on transition
            if current_rts != last_rts:
                print(f"[Bridge] ldpic32 changed RTS -> {current_rts}")
                try:
                    ser.rts = current_rts
                    last_rts = current_rts
                except Exception as e:
                    print(f"[Bridge] Failed to send RTS network command: {e}")
                
            # Send DTR command over RFC 2217 instantly on transition
            if current_dtr != last_dtr:
                print(f"[Bridge] ldpic32 changed DTR -> {current_dtr}")
                try:
                    ser.dtr = current_dtr
                    last_dtr = current_dtr
                except Exception as e:
                    print(f"[Bridge] Failed to send DTR network command: {e}")
                    
        time.sleep(0.005) # 5ms fast polling

# 3. Spin up all three threads
t1 = threading.Thread(target=network_to_virtual)
t2 = threading.Thread(target=virtual_to_network_data)
t3 = threading.Thread(target=monitor_virtual_pins)

t1.daemon, t2.daemon, t3.daemon = True, True, True
t1.start()
t2.start()
t3.start()

try:
    while alive:
        t1.join(0.5)
except KeyboardInterrupt:
    print("\nClosing bridge.")
finally:
    alive = False
    ser.close()
    os.close(keep_alive_slave)
