# A-Axis Motor Control Issue - Troubleshooting Guide

## Problem Description
- **A+ and A- buttons**: Do nothing when pressed
- **Z+ and Z- buttons**: Lock/move the A motor instead of Z motor

## Possible Causes

### 1. Hardware Wiring Issue (Most Likely)
The Z motor driver is physically connected to the A motor, or the motor cables are swapped.

**Solution:**
- Check your CNC controller board wiring
- Verify that the A motor is connected to the A-axis driver port
- Verify that the Z motor is connected to the Z-axis driver port
- Swap the motor connections if they are reversed

### 2. GRBL Firmware Limitation
Your GRBL firmware may only support 3 axes (X, Y, Z) and not 4 axes.

**How to Check:**
- Run the diagnostic script: `python diagnose_axes.py`
- Look for A-axis configuration in GRBL settings ($103, $113, $123, $133)
- If A-axis settings are missing or show as "NOT CONFIGURED", you need different firmware

**Solution:**
- Flash grblHAL firmware (supports up to 6 axes)
- Or GRBL 1.1h with 4-axis support
- Standard GRBL 1.1 only supports 3 axes

### 3. GRBL Configuration Issue
A-axis is disabled or misconfigured in GRBL settings.

**Solution:**
- Verify A-axis settings exist:
  - $103 = steps per inch/mm for A
  - $113 = max rate for A
  - $123 = acceleration for A  
  - $133 = max travel for A
- If missing, your firmware doesn't support A-axis

## Diagnostic Steps

### Step 1: Enable Debug Mode (Already Done)
Debug logging has been enabled in your application. When you run the app:
1. Open a terminal
2. Run: `python main_app.py`
3. Watch for `[GRBL DEBUG]` and `[JOG DEBUG]` messages
4. Press A+ and Z+ buttons
5. Note which commands are actually sent to GRBL

### Step 2: Run Diagnostic Script
```bash
python diagnose_axes.py
```

This will:
- Check if A-axis is configured in GRBL
- Send test commands to each axis
- Show which motor actually moves

### Step 3: Verify Motor Connections
1. Disconnect power from your CNC machine
2. Follow the motor cable from the A motor to the controller board
3. Verify it's plugged into the A-axis driver port (not Z-axis port)
4. Check your controller board documentation for correct port labels

## Quick Test

With debug mode enabled, when you press buttons you should see:
```
[JOG DEBUG] Jogging axis A by 1.000
[JOG DEBUG] Sending jog command: axis=A, delta=1.000
[GRBL DEBUG] Sending jog command: $J=G91 A1.000 F100
```

If you see "axis A" being sent but the Z motor moves, it's a **wiring issue**.
If you see an error response from GRBL, it's a **firmware issue**.

## Expected GRBL Settings for A-Axis

Your configuration file sets these A-axis parameters:
- $103 = 254.00000 (steps/inch)
- $113 = 6000.000 (max rate, inch/min)
- $123 = 150.000 (acceleration, inch/sec²)
- $133 = 200.000 (max travel, mm)

If your GRBL doesn't accept these settings, you need 4-axis firmware.

## Recommended Actions

1. **First**: Run `python diagnose_axes.py` to identify the issue
2. **If wiring issue**: Swap motor connections on controller board
3. **If firmware issue**: Flash grblHAL or 4-axis GRBL firmware
4. **Report back**: Share the diagnostic script output for further help
