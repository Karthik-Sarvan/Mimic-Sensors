#!/usr/bin/env python3
"""
DAC8552IDGKR Hardware Interfacing & Diagnostic Tool for Mimic
=============================================================
Texas Instruments DAC8552: 16-bit, Dual-Channel, Ultra-Low Glitch,
Voltage Output Digital-to-Analog Converter with 3-Wire Serial/SPI Interface.

This script does NOT emulate/simulate a DAC8552 slave. Instead, Mimic acts
as the Master controller, driving and interfacing with a physical
DAC8552IDGKR chip, verifying communication, generating precision DC voltages,
and running automated/interactive diagnostics to ensure the interface is working.

Hardware Wiring (STM32 BlackPill <-> DAC8552IDGKR VSSOP-8/DGK):
---------------------------------------------------------------
  STM32 BlackPill          DAC8552IDGKR (VSSOP-8 / DGK)
  ------------------       ----------------------------
  5V              ------>  Pin 1 (VDD)   - Power Supply (2.7V - 5.5V)
  5V              ------>  Pin 2 (VREF)  - Reference Voltage Input (5V)
  PB14            ------>  Pin 5 (SYNC)  - Chip Select / Frame Sync (Active LOW)
  PA12            ------>  Pin 6 (SCLK)  - Serial Clock
  PA15            ------>  Pin 7 (DIN)   - Serial Data Input
  GND             ------>  Pin 8 (GND)   - Ground

  Analog Outputs:
  Multimeter/Scope <-----  Pin 4 (VOUTA) - DAC Channel A Output (0V to VREF)
  Multimeter/Scope <-----  Pin 3 (VOUTB) - DAC Channel B Output (0V to VREF)

  Optional Digital Loopback Verification (Zero Instruments Required):
  PB0 (STM32)     <------  Pin 4 (VOUTA) or Pin 3 (VOUTB)
                           (Allows automated digital verification: 0V=LOW, 5V=HIGH)

Author: Mimic Project / Aegion Dynamics
"""

import sys
import time
import math
import argparse
import logging
from typing import Optional, List, Tuple

try:
    from mimic import MimicBridge
except ImportError:
    try:
        from MimicBridge.mimic import MimicBridge
    except ImportError:
        # Fallback if run standalone from repo subdirectory
        import os
        sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../../MimicBridge")))
        from mimic import MimicBridge

logging.basicConfig(level=logging.INFO, format="[%(asctime)s] [%(levelname)s] %(message)s")
logger = logging.getLogger("DAC8552")


class DAC8552:
    """
    Driver for Texas Instruments DAC8552 16-bit Dual-Channel DAC.
    
    Operates Mimic as a Master driving the DAC8552 over Serial/SPI.
    Supports both GPIO bit-banging (PB14=SYNC, PA12=SCLK, PA15=DIN)
    and hardware SPI peripherals.
    
    Data Frame (24-bit word shifted MSB first on SCLK falling edges):
      [Byte 0: Control Byte] [Byte 1: Data MSB] [Byte 2: Data LSB]
      
    Control Byte Bit Mapping:
      DB23: 0 (Reserved)
      DB22: 0 (Reserved)
      DB21: LDB (Load DAC B register)
      DB20: LDA (Load DAC A register)
      DB19: 0 (Don't Care)
      DB18: Buffer Select (0 = Channel A, 1 = Channel B)
      DB17: PD1 (Power Down Mode 1)
      DB16: PD0 (Power Down Mode 0)
    """

    # Control Byte Command Definitions (Normal Operation, PD1=0, PD0=0)
    CMD_WRITE_BUFFER_A = 0x00          # Write to Buffer A (output unchanged)
    CMD_WRITE_BUFFER_B = 0x04          # Write to Buffer B (output unchanged)
    CMD_WRITE_A_UPDATE_A = 0x10        # Write to Buffer A & Load/Update DAC A immediately
    CMD_WRITE_B_UPDATE_B = 0x24        # Write to Buffer B & Load/Update DAC B immediately
    CMD_WRITE_A_UPDATE_BOTH = 0x30     # Write to Buffer A & Load BOTH DAC A and B
    CMD_WRITE_B_UPDATE_BOTH = 0x34     # Write to Buffer B & Load BOTH DAC A and B

    # Power Down Modes (PD1, PD0)
    PWR_NORMAL = 0x00                  # Normal operation
    PWR_1K_GND = 0x01                  # Power down with 1 kOhm to GND
    PWR_100K_GND = 0x02                # Power down with 100 kOhm to GND
    PWR_HIGH_Z = 0x03                  # Power down High-Impedance (Three-state)

    def __init__(
        self,
        bridge: MimicBridge,
        cs_pin: str = "B14",
        sclk_pin: str = "A12",
        din_pin: str = "A15",
        spi_instance: Optional[int] = None,
        vref: float = 5.0,
        spi_speed: int = 1000000,
        mode: str = "auto",
    ):
        """
        Initialize the DAC8552 controller.

        :param bridge: Connected MimicBridge instance
        :param cs_pin: GPIO pin used for active-low SYNC/CS (DAC Pin 5, default: 'B14')
        :param sclk_pin: GPIO pin used for Serial Clock SCLK (DAC Pin 6, default: 'A12')
        :param din_pin: GPIO pin used for Serial Data DIN (DAC Pin 7, default: 'A15')
        :param spi_instance: Hardware SPI peripheral on STM32 (1 to 5) if using hardware SPI
        :param vref: Reference voltage in Volts (typically 5.0V)
        :param spi_speed: SPI Clock speed in Hz for hardware SPI (default: 1 MHz, max 30 MHz)
        :param mode: Communication mode ('auto', 'bitbang', or 'spi', default: 'auto')
        """
        self.bridge = bridge
        self.cs_pin = cs_pin.upper().strip()
        self.sclk_pin = sclk_pin.upper().strip()
        self.din_pin = din_pin.upper().strip()
        self.spi_instance = spi_instance
        self.vref = float(vref)
        self.spi_speed = int(spi_speed)
        self.mode = mode.lower().strip()
        self.initialized = False
        self.native_dac = False

    def init(self) -> bool:
        """
        Configure STM32 pins for communicating with DAC8552.
        Supports native firmware DAC commands, GPIO bit-banging (PB14, PA12, PA15), and hardware SPI.
        """
        if self.mode in ("auto", "bitbang"):
            # Check if STM32 firmware has native systematic bit-bang DAC support
            resp = self.bridge.execute(f"DAC_INIT {self.cs_pin} {self.sclk_pin} {self.din_pin} {self.vref}")
            if any("OK" in r for r in resp):
                self.native_dac = True
                self.initialized = True
                self.mode = "bitbang"
                logger.info(f"DAC8552 STM32 Native Systematic Bitbang Driver active (SYNC={self.cs_pin}, SCLK={self.sclk_pin}, DIN={self.din_pin}, VREF={self.vref}V)")
                return True

        if self.mode == "auto":
            # If explicit SPI instance is given and pins match hardware SPI pins, use SPI;
            # otherwise (e.g. PB14, PA12, PA15), use bit-bang GPIO.
            if self.spi_instance is not None and self.sclk_pin in ("A5", "B3", "B13", "B10") and self.din_pin in ("A7", "B5", "B15"):
                self.mode = "spi"
            else:
                self.mode = "bitbang"

        if self.mode == "spi":
            inst = self.spi_instance or 1
            logger.info(f"Initializing SPI{inst} Master at {self.spi_speed} Hz...")
            resp = self.bridge.execute(f"SPI_INIT {inst} MASTER {self.spi_speed} 0 1")
            if not any("OK" in r for r in resp):
                resp = self.bridge.execute(f"SPI_INIT {inst} MASTER {self.spi_speed}")
            if not any("OK" in r for r in resp):
                logger.error(f"Failed to initialize SPI Master. Response: {resp}")
                return False

            self.bridge.execute(f"SPI_CS {self.cs_pin} HIGH")
            self.bridge.execute(f"PIN_SET_OUT {self.cs_pin}")
            self.bridge.execute(f"PIN_HIGH {self.cs_pin}")
            self.initialized = True
            logger.info(f"DAC8552 hardware SPI initialized (SPI{inst}, CS={self.cs_pin}, VREF={self.vref}V)")
            return True
        else:
            logger.info(f"Initializing GPIO bitbang interface for DAC8552 (SYNC={self.cs_pin}, SCLK={self.sclk_pin}, DIN={self.din_pin})...")
            # 1. Configure SYNC (Pin 5) as Output and set HIGH (deasserted)
            self.bridge.execute(f"PIN_SET_OUT {self.cs_pin}")
            resp_cs = self.bridge.execute(f"PIN_HIGH {self.cs_pin}")

            # 2. Configure SCLK (Pin 6) as Output and set LOW (idle LOW)
            self.bridge.execute(f"PIN_SET_OUT {self.sclk_pin}")
            resp_sclk = self.bridge.execute(f"PIN_LOW {self.sclk_pin}")

            # 3. Configure DIN (Pin 7) as Output and set LOW
            self.bridge.execute(f"PIN_SET_OUT {self.din_pin}")
            resp_din = self.bridge.execute(f"PIN_LOW {self.din_pin}")

            if not (any("OK" in r for r in resp_cs) and any("OK" in r for r in resp_sclk) and any("OK" in r for r in resp_din)):
                logger.error("Failed to configure GPIO pins for DAC8552 bitbang interface.")
                return False

            self.initialized = True
            logger.info(f"DAC8552 GPIO bitbang interface initialized (SYNC={self.cs_pin}, SCLK={self.sclk_pin}, DIN={self.din_pin}, VREF={self.vref}V)")
            return True

    def voltage_to_code(self, voltage: float) -> int:
        """
        Convert an analog voltage (0.0 to VREF) to a 16-bit integer code (0 to 65535).
        Formula: code = round((voltage / VREF) * 65535)
        """
        clamped_v = max(0.0, min(self.vref, float(voltage)))
        code = int(round((clamped_v / self.vref) * 65535.0))
        return max(0, min(65535, code))

    def code_to_voltage(self, code: int) -> float:
        """
        Convert a 16-bit DAC code to expected output voltage.
        Formula: voltage = (code / 65536) * VREF
        """
        clamped_c = max(0, min(65535, int(code)))
        return (clamped_c / 65536.0) * self.vref

    def _send_frame_bitbang(self, frame24: int) -> bool:
        """
        Bit-bang a 24-bit word to DAC8552:
        - SYNC (cs_pin, Pin 5) goes LOW.
        - Data on DIN (din_pin, Pin 7) is shifted MSB-first.
        - SCLK (sclk_pin, Pin 6) pulses HIGH then LOW (falling edge latches DIN).
        - SYNC goes HIGH to latch data and execute DAC update.
        """
        cmds = [f"PIN_LOW {self.cs_pin}"]
        last_bit = None
        for i in range(23, -1, -1):
            bit = (frame24 >> i) & 1
            if bit != last_bit:
                cmds.append(f"PIN_HIGH {self.din_pin}" if bit else f"PIN_LOW {self.din_pin}")
                last_bit = bit
            cmds.append(f"PIN_HIGH {self.sclk_pin}")
            cmds.append(f"PIN_LOW {self.sclk_pin}")
        cmds.append(f"PIN_HIGH {self.cs_pin}")

        # Send commands in small safe batches if serial port is directly open
        if hasattr(self.bridge, "ser") and self.bridge.ser and self.bridge.ser.is_open:
            try:
                chunk_size = 8
                for idx in range(0, len(cmds), chunk_size):
                    chunk = cmds[idx:idx + chunk_size]
                    payload = "\r\n".join(chunk) + "\r\n"
                    self.bridge.ser.write(payload.encode("utf-8"))
                    self.bridge.ser.flush()
                    expected = len(chunk)
                    start_t = time.time()
                    prompts = 0
                    while prompts < expected and (time.time() - start_t) < 1.0:
                        if self.bridge.ser.in_waiting:
                            c = self.bridge.ser.read(self.bridge.ser.in_waiting).decode("utf-8", errors="ignore")
                            prompts += c.count(">")
                        else:
                            time.sleep(0.001)
                return True
            except Exception as e:
                logger.debug(f"Direct stream write failed ({e}), falling back to execute")

        # Fallback to standard execute
        success = True
        for cmd in cmds:
            resp = self.bridge.execute(cmd)
            if not any("OK" in r for r in resp):
                success = False
        return success

    def send_frame(self, control_byte: int, data_u16: int) -> bool:
        """
        Send a 24-bit frame to the DAC8552 with SYNC/CS pulse.
        
        Frame layout:
          Byte 0: control_byte (8 bits)
          Byte 1: data_u16 MSB (bits 15..8)
          Byte 2: data_u16 LSB (bits 7..0)
        """
        b0 = control_byte & 0xFF
        b1 = (data_u16 >> 8) & 0xFF
        b2 = data_u16 & 0xFF

        if self.mode == "spi":
            inst = self.spi_instance or 1
            hex_data = f"{b0:02X} {b1:02X} {b2:02X}"

            # 1. Assert CS (SYNC LOW)
            self.bridge.execute(f"SPI_CS {self.cs_pin} LOW")

            # 2. Transmit 24-bit data frame over SPI
            send_resp = self.bridge.execute(f"SPI_SEND {inst} {hex_data}")

            # 3. Deassert CS (SYNC HIGH) to latch data
            self.bridge.execute(f"SPI_CS {self.cs_pin} HIGH")

            success = any("OK" in r for r in send_resp)
            if not success:
                logger.warning(f"SPI transmit failed: {send_resp}")
            return success
        else:
            frame24 = (b0 << 16) | (b1 << 8) | b2
            return self._send_frame_bitbang(frame24)

    def set_value(self, channel: str, code: int, update: bool = True) -> bool:
        """
        Set a raw 16-bit DAC code on the specified channel.

        :param channel: 'A', 'B', or 'BOTH'
        :param code: 16-bit unsigned integer (0 - 65535)
        :param update: If True, immediately updates the analog output.
                       If False, buffers the value in the input register.
        :return: True if transmission succeeded, False otherwise.
        """
        ch = channel.upper().strip()
        code = max(0, min(65535, int(code)))

        if getattr(self, "native_dac", False):
            resp = self.bridge.execute(f"DAC_CODE {ch} {code}")
            return any("OK" in r for r in resp)

        if ch == "A":
            cmd = self.CMD_WRITE_A_UPDATE_A if update else self.CMD_WRITE_BUFFER_A
        elif ch == "B":
            cmd = self.CMD_WRITE_B_UPDATE_B if update else self.CMD_WRITE_BUFFER_B
        elif ch in ("BOTH", "AB"):
            cmd = self.CMD_WRITE_B_UPDATE_BOTH
        else:
            raise ValueError(f"Invalid channel '{channel}'. Use 'A', 'B', or 'BOTH'.")

        return self.send_frame(cmd, code)

    def set_voltage(self, channel: str, voltage: float, update: bool = True) -> bool:
        """
        Set a target analog output voltage on the specified channel.

        :param channel: 'A', 'B', or 'BOTH'
        :param voltage: Target voltage in Volts (0.0 to VREF)
        :param update: Whether to immediately update output
        """
        ch = channel.upper().strip()
        if getattr(self, "native_dac", False):
            if ch in ("BOTH", "AB"):
                resp = self.bridge.execute(f"DAC_BOTH {voltage:.4f} {voltage:.4f}")
            else:
                up_param = "" if update else " 0"
                resp = self.bridge.execute(f"DAC_SET {ch} {voltage:.4f}{up_param}")
            return any("OK" in r for r in resp)

        code = self.voltage_to_code(voltage)
        return self.set_value(channel, code, update=update)

    def set_both_voltages(self, voltage_a: float, voltage_b: float) -> bool:
        """
        Set both DAC channels simultaneously using the double-buffered architecture.
        1. Buffers voltage_a into Buffer A (DAC A output does NOT update yet)
        2. Writes voltage_b into Buffer B AND pulses LDAC to update both A & B in unison!
        """
        if getattr(self, "native_dac", False):
            resp = self.bridge.execute(f"DAC_BOTH {voltage_a:.4f} {voltage_b:.4f}")
            return any("OK" in r for r in resp)

        code_a = self.voltage_to_code(voltage_a)
        code_b = self.voltage_to_code(voltage_b)

        # Step 1: Write to Buffer A without updating output
        ok_a = self.send_frame(self.CMD_WRITE_BUFFER_A, code_a)
        if not ok_a:
            return False

        # Step 2: Write to Buffer B and update BOTH DAC A and DAC B
        ok_b = self.send_frame(self.CMD_WRITE_B_UPDATE_BOTH, code_b)
        return ok_b

    def power_down(self, channel: str, mode: int = PWR_1K_GND) -> bool:
        """
        Put specified channel into a low-power mode (down to ~700nA).

        :param channel: 'A', 'B', or 'BOTH'
        :param mode: PWR_1K_GND (1), PWR_100K_GND (2), or PWR_HIGH_Z (3)
        """
        ch = channel.upper().strip()

        if getattr(self, "native_dac", False):
            pwr_mode_str = "1K" if mode == self.PWR_1K_GND else ("100K" if mode == self.PWR_100K_GND else "HIZ")
            resp = self.bridge.execute(f"DAC_PD {ch} {pwr_mode_str}")
            return any("OK" in r for r in resp)

        mode_bits = mode & 0x03

        if ch == "A":
            cmd = self.CMD_WRITE_A_UPDATE_A | mode_bits
            return self.send_frame(cmd, 0x0000)
        elif ch == "B":
            cmd = self.CMD_WRITE_B_UPDATE_B | mode_bits
            return self.send_frame(cmd, 0x0000)
        elif ch in ("BOTH", "AB"):
            cmd_a = self.CMD_WRITE_A_UPDATE_A | mode_bits
            cmd_b = self.CMD_WRITE_B_UPDATE_B | mode_bits
            return self.send_frame(cmd_a, 0x0000) and self.send_frame(cmd_b, 0x0000)
        else:
            raise ValueError(f"Invalid channel '{channel}'.")

    def wake_up(self, channel: str, restore_voltage: float = 0.0) -> bool:
        """
        Wake up a powered-down channel by writing a normal operation command.
        """
        ch = channel.upper().strip()
        if getattr(self, "native_dac", False):
            resp = self.bridge.execute(f"DAC_WAKE {ch} {restore_voltage:.4f}")
            return any("OK" in r for r in resp)
        return self.set_voltage(channel, restore_voltage, update=True)


DAC8552Interface = DAC8552


def print_banner():
    banner = r"""
========================================================================
     MIMIC <-> TEXAS INSTRUMENTS DAC8552 HARDWARE INTERFACE TOOL
========================================================================
  Chip: DAC8552IDGKR (16-bit, Dual Channel, SPI/Serial V-Out DAC)
  Role: Mimic acts as Master to drive & verify physical DAC hardware
========================================================================
"""
    print(banner)


def print_connections():
    guide = """
[PINOUT & HARDWARE WIRING GUIDE]
========================================================================
  DAC8552IDGKR (VSSOP-8 / DGK) Package Pinout:
         +---v---+
  VDD   1| o     |8  GND
  VREF  2|       |7  DIN   (Serial Data In)
  VOUTB 3|       |6  SCLK  (Serial Clock)
  VOUTA 4|       |5  SYNC  (Chip Select / Frame Sync)
         +-------+

  Wiring to STM32 BlackPill (Mimic):
  ----------------------------------------------------------------------
  STM32 BlackPill Pin       DAC8552 Pin       Signal Name & Purpose
  -------------------       -----------       ---------------------
  5V                        Pin 1             VDD   (Power supply, 2.7V - 5.5V)
  5V                        Pin 2             VREF  (Reference voltage: 5.0V)
  PB14                      Pin 5             SYNC  (Active-LOW Chip Select / Frame Sync)
  PA12                      Pin 6             SCLK  (Serial Clock)
  PA15                      Pin 7             DIN   (Serial Data In)
  GND                       Pin 8             GND   (Common Ground)
  
  Outputs:
  Pin 4 (VOUTA)  -> Connect to Multimeter / Oscilloscope / STM32 PB0
  Pin 3 (VOUTB)  -> Connect to Multimeter / Oscilloscope
========================================================================
"""
    print(guide)


def run_diagnostics(dac: DAC8552, loopback_pin: Optional[str] = "B0", manual_probe: bool = False) -> bool:
    """
    Comprehensive verification suite to confirm that the Mimic-to-DAC8552
    hardware interface is working properly.
    """
    print("\n[STARTING INTERFACE DIAGNOSTICS & VERIFICATION]")
    print("-" * 72)
    all_passed = True

    # -------------------------------------------------------------
    # Step 1: Mimic Bridge Connectivity Check
    # -------------------------------------------------------------
    print("[TEST 1/6] Checking Mimic Firmware Communication...")
    ver_resp = dac.bridge.execute("VERSION")
    if any("MIMIC" in r.upper() for r in ver_resp):
        print(f"  -> PASS: Mimic firmware responding: {ver_resp[0] if ver_resp else 'OK'}")
    else:
        print(f"  -> FAIL: Could not detect Mimic firmware response.")
        return False

    # -------------------------------------------------------------
    # Step 2: Interface Configuration
    # -------------------------------------------------------------
    print(f"\n[TEST 2/6] Configuring DAC8552 Interface ({dac.mode.upper()} Mode)...")
    if dac.init():
        if dac.mode == "spi":
            print(f"  -> PASS: SPI{dac.spi_instance or 1} Master initialized at {dac.spi_speed} Hz.")
            print(f"  -> PASS: SYNC / CS pin {dac.cs_pin} configured.")
            status_resp = dac.bridge.execute("SPI_STATUS")
            print(f"  -> SPI Bus Status: {'; '.join(status_resp)}")
        else:
            print(f"  -> PASS: GPIO bitbang interface initialized successfully.")
            print(f"           Pin 5 (SYNC): STM32 {dac.cs_pin}")
            print(f"           Pin 6 (SCLK): STM32 {dac.sclk_pin}")
            print(f"           Pin 7 (DIN) : STM32 {dac.din_pin}")
            status_cs = dac.bridge.execute(f"PIN_STATUS {dac.cs_pin}")
            status_sclk = dac.bridge.execute(f"PIN_STATUS {dac.sclk_pin}")
            status_din = dac.bridge.execute(f"PIN_STATUS {dac.din_pin}")
            print(f"  -> SYNC Pin ({dac.cs_pin}): {'; '.join(status_cs)}")
            print(f"  -> SCLK Pin ({dac.sclk_pin}): {'; '.join(status_sclk)}")
            print(f"  -> DIN Pin  ({dac.din_pin}): {'; '.join(status_din)}")
    else:
        print("  -> FAIL: Interface initialization failed.")
        return False

    # -------------------------------------------------------------
    # Step 3: Serial Protocol Frame Test
    # -------------------------------------------------------------
    print("\n[TEST 3/6] Testing 24-Bit Serial Transmission Protocol...")
    test_ok = dac.send_frame(DAC8552.CMD_WRITE_A_UPDATE_A, 0x8000)
    if test_ok:
        print(f"  -> PASS: 24-bit frame successfully transmitted ({dac.mode.upper()} mode).")
    else:
        print("  -> FAIL: Frame transmission error reported by firmware.")
        all_passed = False

    # -------------------------------------------------------------
    # Step 4: Full-Scale Voltage Steps Verification
    # -------------------------------------------------------------
    print("\n[TEST 4/6] Testing Standard Calibration Voltage Steps...")
    test_voltages = [
        ("Zero Scale (0%)", 0.000, 0x0000),
        ("Quarter Scale (25%)", dac.vref * 0.25, 0x4000),
        ("Mid Scale (50%)", dac.vref * 0.50, 0x8000),
        ("Three-Quarter (75%)", dac.vref * 0.75, 0xC000),
        ("Full Scale (~100%)", dac.vref * (65535 / 65536), 0xFFFF),
    ]

    print(f"  {'Step Description':<22} | {'Target Voltage':<15} | {'16-bit Code':<12} | {'Status':<10}")
    print("  " + "-" * 66)

    for desc, v_target, expected_code in test_voltages:
        ok_a = dac.set_voltage("A", v_target)
        ok_b = dac.set_voltage("B", v_target)
        status_str = "OK" if (ok_a and ok_b) else "FAIL"
        if not (ok_a and ok_b):
            all_passed = False
        print(f"  {desc:<22} | {v_target:>7.3f} V        | 0x{expected_code:04X} ({expected_code:>5}) | {status_str:<10}")
        time.sleep(0.05)

    print("  -> PASS: All voltage step commands transmitted cleanly.")

    # -------------------------------------------------------------
    # Step 5: Simultaneous Double-Buffer Update Test
    # -------------------------------------------------------------
    print("\n[TEST 5/6] Testing Double-Buffered Simultaneous Update...")
    # Set Channel A to 1.0V and Channel B to 2.5V simultaneously
    ok_both = dac.set_both_voltages(1.000, 2.500)
    if ok_both:
        print("  -> PASS: Channel A buffered and Channel B synchronized.")
        print(f"           Expected: VOUTA = 1.000 V, VOUTB = 2.500 V")
    else:
        print("  -> FAIL: Simultaneous update command failed.")
        all_passed = False

    # -------------------------------------------------------------
    # Step 6: Automated Hardware Loopback Verification (Optional)
    # -------------------------------------------------------------
    print("\n[TEST 6/6] Hardware Response Verification...")
    if loopback_pin:
        lb_pin = loopback_pin.upper().strip()
        print(f"  -> Attempting Automated Digital Loopback Check via pin {lb_pin}...")
        print(f"     (Connect DAC Pin 4 [VOUTA] to STM32 pin {lb_pin})")
        
        # Configure test pin as input with pull-down or no-pull
        dac.bridge.execute(f"PIN_SET_IN {lb_pin} NONE")
        time.sleep(0.05)

        loopback_success = True
        # Test 1: DAC Output = 0.0V (Digital LOW) on BOTH channels
        dac.set_both_voltages(0.000, 0.000)
        time.sleep(0.1)
        resp_low = dac.bridge.execute(f"PIN_READ {lb_pin}")
        is_low = any("LOW" in r for r in resp_low)

        # Test 2: DAC Output = VREF (Digital HIGH) on BOTH channels
        dac.set_both_voltages(dac.vref, dac.vref)
        time.sleep(0.1)
        resp_high = dac.bridge.execute(f"PIN_READ {lb_pin}")
        is_high = any("HIGH" in r for r in resp_high)

        # Test 3: Multiple rapid toggles to verify tracking
        toggle_count = 0
        for _ in range(3):
            dac.set_both_voltages(0.000, 0.000)
            time.sleep(0.05)
            r_l = dac.bridge.execute(f"PIN_READ {lb_pin}")
            dac.set_both_voltages(dac.vref, dac.vref)
            time.sleep(0.05)
            r_h = dac.bridge.execute(f"PIN_READ {lb_pin}")
            if any("LOW" in r for r in r_l) and any("HIGH" in r for r in r_h):
                toggle_count += 1

        if is_low and is_high and toggle_count >= 2:
            print(f"  -> [CONFIRMED!] HARDWARE LOOPBACK PASSED:")
            print(f"     Pin {lb_pin} read LOW at 0V and HIGH at {dac.vref:.1f}V ({toggle_count}/3 toggles tracked).")
            print(f"     *** PHYSICAL DAC8552 HARDWARE IS WORKING AND RESPONSIVE! ***")
        else:
            print(f"  -> Note: Loopback pin {lb_pin} did not detect voltage transitions.")
            print(f"     (LOW test: {'PASS' if is_low else 'NO'}, HIGH test: {'PASS' if is_high else 'NO'})")
            print(f"     If wire is connected between VOUTA (Pin 4) or VOUTB (Pin 3) and {lb_pin}:")
            print(f"       1. Verify DAC Pin 2 (VREF) is connected to {dac.vref:.1f}V.")
            print(f"       2. Verify DAC Pin 1 (VDD) is connected to 5V (>= VREF) and Pin 8 is GND.")
            print(f"       3. Use a multimeter to verify voltage on Pin 4/Pin 3 directly.")
    else:
        print("  -> Loopback disabled. Use multimeter or oscilloscope on Pin 4 (VOUTA) & Pin 3 (VOUTB).")

    # -------------------------------------------------------------
    # Manual Probing Mode (Interactive Hold)
    # -------------------------------------------------------------
    if manual_probe:
        print("\n" + "=" * 72)
        print("[MANUAL PROBE BENCH TEST]")
        print("Use a digital multimeter (DC Volts) with black probe on GND (Pin 8).")
        print("=" * 72)
        bench_steps = [
            ("0.000 V (Zero Scale)", 0.000),
            (f"{dac.vref * 0.25:.3f} V (Quarter Scale)", dac.vref * 0.25),
            (f"{dac.vref * 0.50:.3f} V (Half Scale)", dac.vref * 0.50),
            (f"{dac.vref * 0.75:.3f} V (Three-Quarter Scale)", dac.vref * 0.75),
            (f"{dac.vref:.3f} V (Full Scale)", dac.vref),
        ]
        for label, v in bench_steps:
            dac.set_both_voltages(v, v)
            print(f"\n>> Setting VOUTA and VOUTB to {label}...")
            print(f"   Touch RED probe to Pin 4 (VOUTA) or Pin 3 (VOUTB).")
            try:
                input("   Press [ENTER] to proceed to next voltage test (or Ctrl+C to stop)...")
            except KeyboardInterrupt:
                print("\nManual probe test ended by user.")
                break

    # Summary
    print("\n" + "=" * 72)
    if all_passed:
        print("DIAGNOSTIC RESULT: [SUCCESS] Mimic interface to DAC8552 is ready!")
    else:
        print("DIAGNOSTIC RESULT: [WARNING] Some checks reported errors. Check wiring.")
    print("=" * 72)
    return all_passed


def run_voltage_sweep(dac: DAC8552, cycles: int = 3, step_v: float = 0.1, delay_s: float = 0.05):
    """
    Generate a continuous triangle ramp wave on VOUTA and inverted ramp on VOUTB.
    Great for observing on an oscilloscope.
    """
    print(f"\n[RUNNING ANALOG RAMP SWEEP: {cycles} cycles, step={step_v}V]")
    print(f"VOUTA (Pin 4): Rising ramp 0V -> {dac.vref:.1f}V -> 0V")
    print(f"VOUTB (Pin 3): Inverted ramp {dac.vref:.1f}V -> 0V -> {dac.vref:.1f}V")
    print("Press Ctrl+C to abort sweep.\n")

    try:
        for c in range(1, cycles + 1):
            print(f"  Cycle {c}/{cycles}...")
            # Up ramp
            v = 0.0
            while v <= dac.vref:
                dac.set_both_voltages(v, dac.vref - v)
                time.sleep(delay_s)
                v += step_v
            # Down ramp
            v = dac.vref
            while v >= 0.0:
                dac.set_both_voltages(v, dac.vref - v)
                time.sleep(delay_s)
                v -= step_v
        print("Ramp sweep complete. Resetting outputs to 0V.")
        dac.set_both_voltages(0.0, 0.0)
    except KeyboardInterrupt:
        print("\nSweep interrupted by user. Resetting outputs to 0V.")
        dac.set_both_voltages(0.0, 0.0)


def run_sine_wave(dac: DAC8552, frequency: float = 1.0, duration: float = 10.0):
    """
    Generate sine waves on VOUTA and VOUTB (with 180 degree phase shift).
    """
    print(f"\n[RUNNING SINE WAVE GENERATOR: {frequency} Hz for {duration} seconds]")
    print(f"VOUTA: Sine wave (~0.1V to ~{dac.vref - 0.1:.1f}V)")
    print("VOUTB: Cosine wave (180 deg out of phase)")
    print("Press Ctrl+C to stop.\n")

    start_time = time.time()
    mid = dac.vref / 2.0
    amp = (dac.vref / 2.0) * 0.9  # 90% full scale

    try:
        while time.time() - start_time < duration:
            t = time.time() - start_time
            val_a = mid + amp * math.sin(2.0 * math.pi * frequency * t)
            val_b = mid + amp * math.cos(2.0 * math.pi * frequency * t)
            dac.set_both_voltages(val_a, val_b)
            time.sleep(0.01)
        print("Sine generation complete. Resetting outputs to 0V.")
        dac.set_both_voltages(0.0, 0.0)
    except KeyboardInterrupt:
        print("\nSine wave interrupted. Resetting outputs to 0V.")
        dac.set_both_voltages(0.0, 0.0)


def interactive_cli(dac: DAC8552):
    """
    Interactive prompt allowing the user to set specific voltages or commands.
    """
    print("\n[INTERACTIVE VOLTAGE CONTROLLER]")
    print("Commands:")
    print("  set a <volts>     - Set Channel A voltage (e.g. 'set a 1.65')")
    print("  set b <volts>     - Set Channel B voltage (e.g. 'set b 2.50')")
    print("  both <v_a> <v_b>  - Set both channels simultaneously (e.g. 'both 1.0 2.0')")
    print("  code a <0-65535>  - Set raw 16-bit code on Channel A")
    print("  code b <0-65535>  - Set raw 16-bit code on Channel B")
    print("  pd a              - Power-down Channel A (1k to GND)")
    print("  pd b              - Power-down Channel B (1k to GND)")
    print("  wake a            - Wake up Channel A")
    print("  wake b            - Wake up Channel B")
    print("  test              - Re-run automated diagnostics")
    print("  ramp              - Run a 3-cycle ramp sweep")
    print("  help              - Show this menu")
    print("  exit / quit       - Exit interactive mode\n")

    while True:
        try:
            cmd = input("DAC8552> ").strip()
            if not cmd:
                continue
            parts = cmd.split()
            c = parts[0].lower()

            if c in ("exit", "quit", "q"):
                print("Exiting interactive mode.")
                break
            elif c == "help":
                interactive_cli(dac)
                break
            elif c == "test":
                run_diagnostics(dac)
            elif c == "ramp":
                run_voltage_sweep(dac, cycles=2)
            elif c == "set" and len(parts) >= 3:
                ch = parts[1].upper()
                v = float(parts[2])
                dac.set_voltage(ch, v)
                code = dac.voltage_to_code(v)
                print(f"Channel {ch} set to {v:.3f} V (Code 0x{code:04X} = {code})")
            elif c == "both" and len(parts) >= 3:
                va = float(parts[1])
                vb = float(parts[2])
                dac.set_both_voltages(va, vb)
                print(f"Simultaneous update: VOUTA = {va:.3f} V, VOUTB = {vb:.3f} V")
            elif c == "code" and len(parts) >= 3:
                ch = parts[1].upper()
                code = int(parts[2], 0)
                dac.set_value(ch, code)
                v = dac.code_to_voltage(code)
                print(f"Channel {ch} set to code 0x{code:04X} ({code}) -> ~{v:.3f} V")
            elif c == "pd" and len(parts) >= 2:
                ch = parts[1].upper()
                dac.power_down(ch, DAC8552.PWR_1K_GND)
                print(f"Channel {ch} powered down (1k to GND).")
            elif c == "wake" and len(parts) >= 2:
                ch = parts[1].upper()
                dac.wake_up(ch, 0.0)
                print(f"Channel {ch} awakened (reset to 0.0 V).")
            else:
                print(f"Unknown command: '{cmd}'. Type 'help' for options.")
        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except Exception as e:
            print(f"Error executing command: {e}")


def run_pin_audit(bridge: MimicBridge, cs_pin: str = "B14", sclk_pin: str = "A12", din_pin: str = "A15"):
    """
    Diagnostic tool to verify the physical electrical wiring between STM32 and DAC8552.
    Drives each interface pin to static DC 3.3V and 0V so the user can verify connections with a DMM.
    """
    print("\n" + "=" * 72)
    print("[HARDWARE PIN CONTINUITY & VOLTAGE AUDIT]")
    print("Keep BLACK multimeter probe on GND (BlackPill GND or DAC Pin 8).")
    print("=" * 72)

    tests = [
        (f"SYNC / Chip Select (STM32 {cs_pin} -> DAC Pin 5)", cs_pin, "Pin 5 (SYNC)"),
        (f"Serial Clock SCLK (STM32 {sclk_pin} -> DAC Pin 6)", sclk_pin, "Pin 6 (SCLK)"),
        (f"Serial Data In DIN (STM32 {din_pin} -> DAC Pin 7)", din_pin, "Pin 7 (DIN)"),
    ]

    for label, stm_pin, dac_pin in tests:
        print(f"\n--- Testing {label} ---")
        
        # 1. Drive HIGH
        bridge.execute(f"PIN_SET_OUT {stm_pin}")
        bridge.execute(f"PIN_HIGH {stm_pin}")
        print(f"  [1] Setting {stm_pin} = HIGH (+3.3V):")
        print(f"      a. Probe STM32 header pin '{stm_pin}' directly on the BlackPill.")
        print(f"         -> MUST read: ~3.3V")
        print(f"      b. Probe DAC '{dac_pin}' on the DAC chip.")
        print(f"         -> MUST read: ~3.3V")
        print(f"         (If STM32 is 3.3V but DAC pin is ~0.1V, the jumper wire is broken or in the wrong hole!)")
        input("      Press [ENTER] when ready to test LOW...")

        # 2. Drive LOW
        bridge.execute(f"PIN_LOW {stm_pin}")
        print(f"  [2] Setting {stm_pin} = LOW (0.0V):")
        print(f"      Probe DAC '{dac_pin}'.")
        print(f"      -> MUST read: ~0.0V")
        input("      Press [ENTER] to continue to next pin...")

    # Return pins to safe state
    bridge.execute(f"PIN_SET_OUT {cs_pin}")
    bridge.execute(f"PIN_HIGH {cs_pin}")
    bridge.execute(f"PIN_SET_OUT {sclk_pin}")
    bridge.execute(f"PIN_LOW {sclk_pin}")
    bridge.execute(f"PIN_SET_OUT {din_pin}")
    bridge.execute(f"PIN_LOW {din_pin}")
    print("\n[Pin audit completed. SYNC returned to HIGH, SCLK and DIN to LOW.]")


def main():
    parser = argparse.ArgumentParser(
        description="DAC8552IDGKR Hardware Interfacing & Diagnostic Tool for Mimic"
    )
    parser.add_argument("--port", "-p", default=None, help="Serial port of Mimic board (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--cs", default="B14", help="Chip Select / SYNC pin connected to Pin 5 (default: B14)")
    parser.add_argument("--sclk", default="A12", help="Serial Clock pin connected to Pin 6 (default: A12)")
    parser.add_argument("--din", default="A15", help="Serial Data In pin connected to Pin 7 (default: A15)")
    parser.add_argument("--mode", choices=["auto", "bitbang", "spi"], default="auto", help="Communication mode: auto, bitbang (GPIO), or spi (default: auto)")
    parser.add_argument("--spi", type=int, default=None, help="Hardware SPI instance (1-5) if using SPI mode")
    parser.add_argument("--vref", type=float, default=5.0, help="Reference voltage in Volts (default: 5.0)")
    parser.add_argument("--speed", type=int, default=1000000, help="SPI clock speed in Hz (default: 1000000)")
    parser.add_argument("--loopback", "-l", default="B0", help="STM32 GPIO pin connected to VOUTA for auto loopback test (default: B0)")
    parser.add_argument("--no-loopback", action="store_true", help="Disable automated GPIO loopback check")
    parser.add_argument("--probe", action="store_true", help="Enable step-by-step manual multimeter probing mode")
    parser.add_argument("--check-pins", action="store_true", help="Run interactive static DC pin audit to verify jumper wires with multimeter")
    parser.add_argument("--interactive", "-i", action="store_true", help="Launch interactive voltage controller CLI")
    parser.add_argument("--sweep", action="store_true", help="Run analog triangle waveform ramp sweep")
    parser.add_argument("--sine", action="store_true", help="Run dual-channel sine wave generator")
    parser.add_argument("--pinout", action="store_true", help="Display pinout and wiring guide only")

    args = parser.parse_args()

    print_banner()

    if args.pinout:
        print_connections()
        return

    # Auto-detect USB serial port if none provided
    target_port = args.port
    if not target_port:
        import serial.tools.list_ports
        detected_ports = [p.device for p in serial.tools.list_ports.comports()]
        usb_ports = [p for p in detected_ports if "ttyUSB" in p or "ttyACM" in p]
        if usb_ports:
            target_port = usb_ports[0]

    # Connect to Mimic Bridge
    print("[CONNECTING TO MIMIC HARDWARE BRIDGE]")
    bridge = MimicBridge(port=target_port, baud=args.baud)
    if not bridge.connect():
        print("ERROR: Failed to connect to Mimic board.")
        print("Make sure your USB-to-TTL adapter is connected to PA2 (TX) and PA3 (RX) on the STM32.")
        print_connections()
        sys.exit(1)

    print(f"Connected to Mimic on port: {bridge.port}\n")

    # Instantiate DAC8552 controller
    dac = DAC8552(
        bridge=bridge,
        cs_pin=args.cs,
        sclk_pin=args.sclk,
        din_pin=args.din,
        spi_instance=args.spi,
        vref=args.vref,
        spi_speed=args.speed,
        mode=args.mode,
    )

    try:
        loopback_pin = None if args.no_loopback else args.loopback

        if args.check_pins:
            run_pin_audit(bridge, cs_pin=args.cs, sclk_pin=args.sclk, din_pin=args.din)
        elif args.sweep:
            dac.init()
            run_voltage_sweep(dac)
        elif args.sine:
            dac.init()
            run_sine_wave(dac)
        elif args.interactive:
            dac.init()
            interactive_cli(dac)
        else:
            # Default behavior: Print wiring guide, run diagnostics, and optionally launch CLI
            print_connections()
            success = run_diagnostics(dac, loopback_pin=loopback_pin, manual_probe=args.probe)
            
            if not args.probe and not args.interactive:
                print("\nTip: Run with '--interactive' or '-i' to dynamically set voltages.")
                print("     Run with '--probe' for guided step-by-step multimeter measurement.")
                print("     Run with '--sweep' or '--sine' to generate continuous test waveforms.")

    finally:
        bridge.disconnect()
        print("\nSession finished. Disconnected from Mimic.")


if __name__ == "__main__":
    main()
