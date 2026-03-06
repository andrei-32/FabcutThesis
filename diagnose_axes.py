#!/usr/bin/env python3
"""
Diagnostic script to test GRBL axis configuration.
This script will help identify if A-axis is properly configured
and if there are any wiring or configuration issues.
"""

import serial
import time
import sys
import glob

def find_grbl_port():
    """Find GRBL controller port."""
    # Try common ports
    ports = ['/dev/ttyACM0', '/dev/ttyACM1', 'COM3', 'COM4', 'COM5']
    ports.extend(glob.glob('/dev/ttyACM*'))
    
    for port in ports:
        try:
            print(f"Trying {port}...")
            ser = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)
            ser.write(b'$I\n')
            time.sleep(0.5)
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            if 'GRBL' in response.upper() or 'VER' in response.upper():
                print(f"✓ Found GRBL on {port}")
                return ser
            ser.close()
        except:
            continue
    
    print("✗ No GRBL controller found")
    return None

def send_command(ser, command, wait_time=0.5):
    """Send command and get response."""
    ser.write((command + '\n').encode())
    time.sleep(wait_time)
    response = []
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            response.append(line)
    return response

def main():
    print("="*60)
    print("GRBL AXIS DIAGNOSTIC")
    print("="*60)
    
    # Connect to GRBL
    ser = find_grbl_port()
    if not ser:
        return
    
    try:
        # Clear any alarms
        print("\nClearing alarms...")
        send_command(ser, '$X')
        time.sleep(1)
        
        # Get GRBL version and build info
        print("\n--- GRBL Version Info ---")
        responses = send_command(ser, '$I', wait_time=1.0)
        for resp in responses:
            print(resp)
        
        # Get current GRBL settings
        print("\n--- Checking Axis Configuration ---")
        responses = send_command(ser, '$$', wait_time=1.5)
        
        # Parse and display axis-related settings
        axis_settings = {
            'X': {'steps': None, 'max_rate': None, 'accel': None, 'max_travel': None},
            'Y': {'steps': None, 'max_rate': None, 'accel': None, 'max_travel': None},
            'Z': {'steps': None, 'max_rate': None, 'accel': None, 'max_travel': None},
            'A': {'steps': None, 'max_rate': None, 'accel': None, 'max_travel': None},
        }
        
        for line in responses:
            if line.startswith('$'):
                if '=' in line:
                    setting, value = line.split('=')
                    
                    # Steps per unit
                    if setting == '$100': axis_settings['X']['steps'] = value
                    elif setting == '$101': axis_settings['Y']['steps'] = value
                    elif setting == '$102': axis_settings['Z']['steps'] = value
                    elif setting == '$103': axis_settings['A']['steps'] = value
                    
                    # Max rates
                    elif setting == '$110': axis_settings['X']['max_rate'] = value
                    elif setting == '$111': axis_settings['Y']['max_rate'] = value
                    elif setting == '$112': axis_settings['Z']['max_rate'] = value
                    elif setting == '$113': axis_settings['A']['max_rate'] = value
                    
                    # Acceleration
                    elif setting == '$120': axis_settings['X']['accel'] = value
                    elif setting == '$121': axis_settings['Y']['accel'] = value
                    elif setting == '$122': axis_settings['Z']['accel'] = value
                    elif setting == '$123': axis_settings['A']['accel'] = value
                    
                    # Max travel
                    elif setting == '$130': axis_settings['X']['max_travel'] = value
                    elif setting == '$131': axis_settings['Y']['max_travel'] = value
                    elif setting == '$132': axis_settings['Z']['max_travel'] = value
                    elif setting == '$133': axis_settings['A']['max_travel'] = value
        
        # Display axis configuration
        print("\nAxis Configuration:")
        print("-" * 60)
        print(f"{'Axis':<6}{'Steps/unit':<15}{'Max Rate':<15}{'Accel':<15}{'Max Travel':<15}")
        print("-" * 60)
        
        for axis in ['X', 'Y', 'Z', 'A']:
            settings = axis_settings[axis]
            configured = any(v is not None for v in settings.values())
            
            if configured:
                print(f"{axis:<6}"
                      f"{settings['steps'] or 'N/A':<15}"
                      f"{settings['max_rate'] or 'N/A':<15}"
                      f"{settings['accel'] or 'N/A':<15}"
                      f"{settings['max_travel'] or 'N/A':<15}")
            else:
                print(f"{axis:<6}{'NOT CONFIGURED':^60}")
        
        print()
        
        # Check if A-axis is configured
        if axis_settings['A']['steps'] is None:
            print("⚠️  WARNING: A-axis is NOT configured in GRBL firmware!")
            print("   Your GRBL firmware may only support 3 axes (X, Y, Z).")
            print("   You need grblHAL or GRBL with 4-axis support.")
        else:
            print("✓ A-axis IS configured in GRBL firmware")
        
        # Test jogging each axis
        print("\n--- Testing Axis Movement ---")
        print("This will send small jog commands to each axis.")
        print("Watch which motor actually moves for each command.\n")
        
        input("Press Enter to test X-axis...")
        print("Sending: $J=G91 X0.001 F100")
        send_command(ser, '$J=G91 X0.001 F100')
        time.sleep(1)
        
        input("Press Enter to test Y-axis...")
        print("Sending: $J=G91 Y0.001 F100")
        send_command(ser, '$J=G91 Y0.001 F100')
        time.sleep(1)
        
        input("Press Enter to test Z-axis...")
        print("Sending: $J=G91 Z0.001 F100")
        send_command(ser, '$J=G91 Z0.001 F100')
        time.sleep(1)
        
        input("Press Enter to test A-axis...")
        print("Sending: $J=G91 A0.001 F100")
        responses = send_command(ser, '$J=G91 A0.001 F100')
        time.sleep(1)
        
        # Check for errors
        if responses:
            print("\nGRBL responses:")
            for resp in responses:
                print(f"  {resp}")
                if 'error' in resp.lower():
                    print("  ⚠️  Error detected! A-axis command was rejected.")
        
        print("\n" + "="*60)
        print("DIAGNOSIS COMPLETE")
        print("="*60)
        print("\nPlease report which motor moved for each command:")
        print("- If Z motor moves when sending A command: WIRING ISSUE")
        print("- If nothing moves for A command: FIRMWARE/CONFIGURATION ISSUE")
        print("- If A motor moves when sending Z command: WIRING ISSUE")
        
    except Exception as e:
        print(f"\nError during diagnostic: {e}")
    
    finally:
        ser.close()
        print("\nConnection closed.")

if __name__ == '__main__':
    main()
