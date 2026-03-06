#!/usr/bin/env python3
"""
Manually configure A-axis settings in GRBL.
This script will check if your firmware supports 4 axes and configure it.
"""

import serial
import time
import sys
import glob

def find_grbl_port():
    """Find GRBL controller port."""
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
    print("="*70)
    print("GRBL A-AXIS CONFIGURATION TOOL")
    print("="*70)
    
    ser = find_grbl_port()
    if not ser:
        return
    
    try:
        # Clear alarms
        print("\nClearing alarms...")
        send_command(ser, '$X')
        time.sleep(1)
        
        # Get version info
        print("\n--- GRBL Version ---")
        responses = send_command(ser, '$I', wait_time=1.0)
        for resp in responses:
            print(resp)
            if 'grblHAL' in resp.lower():
                print("✓ You have grblHAL - supports 4+ axes")
            elif 'GRBL 1.1' in resp:
                print("⚠️  GRBL 1.1 typically only supports 3 axes (X, Y, Z)")
                print("   You may need grblHAL for A-axis support")
        
        # Get current settings
        print("\n--- Current Settings ---")
        responses = send_command(ser, '$$', wait_time=1.5)
        
        a_axis_settings = {}
        for line in responses:
            if line.startswith('$'):
                if '=' in line:
                    setting, value = line.split('=')
                    if setting in ['$103', '$113', '$123', '$133']:
                        a_axis_settings[setting] = value
                        print(f"{setting} = {value}")
        
        # Check if A-axis is configured
        if not a_axis_settings or all(v == '0' or v == '0.000' for v in a_axis_settings.values()):
            print("\n⚠️  A-AXIS IS NOT CONFIGURED!")
            print("\nPossible reasons:")
            print("1. Your firmware doesn't support 4 axes (need grblHAL)")
            print("2. Settings need to be manually configured")
            
            print("\n" + "="*70)
            choice = input("\nDo you want to try configuring A-axis now? (y/n): ")
            
            if choice.lower() == 'y':
                print("\nConfiguring A-axis settings...")
                
                a_settings = {
                    '$103': '254.000',    # A steps per inch
                    '$113': '6000.000',   # A max rate
                    '$123': '150.000',    # A acceleration
                    '$133': '200.000'     # A max travel
                }
                
                for setting, value in a_settings.items():
                    cmd = f"{setting}={value}"
                    print(f"Sending: {cmd}")
                    responses = send_command(ser, cmd, wait_time=0.3)
                    
                    # Check for errors
                    for resp in responses:
                        if 'error' in resp.lower():
                            print(f"  ❌ ERROR: {resp}")
                            print("  → Your firmware does NOT support A-axis!")
                            print("\n" + "="*70)
                            print("SOLUTION: You need to flash grblHAL firmware")
                            print("="*70)
                            return
                        elif resp == 'ok':
                            print(f"  ✓ {setting} configured")
                
                # Verify settings were saved
                print("\n--- Verifying A-axis Configuration ---")
                responses = send_command(ser, '$$', wait_time=1.5)
                
                configured = False
                for line in responses:
                    if line.startswith('$103') or line.startswith('$113') or \
                       line.startswith('$123') or line.startswith('$133'):
                        print(line)
                        if '=' in line:
                            _, value = line.split('=')
                            if float(value) > 0:
                                configured = True
                
                if configured:
                    print("\n✓ SUCCESS! A-axis is now configured!")
                else:
                    print("\n❌ FAILED: Settings were not saved")
                    print("   Your firmware likely does NOT support 4 axes")
        else:
            print("\n✓ A-axis IS configured!")
            print("\nConfigured settings:")
            for setting, value in a_axis_settings.items():
                print(f"  {setting} = {value}")
        
        # Test A-axis
        if a_axis_settings and any(float(v) > 0 for v in a_axis_settings.values()):
            print("\n" + "="*70)
            test = input("\nDo you want to test A-axis movement? (y/n): ")
            
            if test.lower() == 'y':
                print("\nSending: $J=G91 A0.1 F100")
                print("Watch which motor moves!")
                responses = send_command(ser, '$J=G91 A0.1 F100', wait_time=1.0)
                
                for resp in responses:
                    print(f"  Response: {resp}")
                    if 'error' in resp.lower():
                        print("  ❌ A-axis command rejected!")
                    elif resp == 'ok':
                        print("  ✓ Command accepted")
        
        print("\n" + "="*70)
        print("FIRMWARE INFORMATION")
        print("="*70)
        print("\nIf A-axis settings are rejected:")
        print("• Standard GRBL 1.1 only supports 3 axes (X, Y, Z)")
        print("• You need grblHAL for 4+ axis support")
        print("\nTo flash grblHAL:")
        print("1. Visit: https://github.com/grblHAL")
        print("2. Download firmware for your board (RP2040, ESP32, etc.)")
        print("3. Flash using Web Installer or platform tools")
        print("="*70)
        
    except Exception as e:
        print(f"\nError: {e}")
    
    finally:
        ser.close()
        print("\nConnection closed.")

if __name__ == '__main__':
    main()
