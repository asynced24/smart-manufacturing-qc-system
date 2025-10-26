#!/usr/bin/env python3
"""
====================================================================
SMART BLOCK DETECTION & REMOVAL SYSTEM  (Edge AI + IIoT + Cloud)
====================================================================

High-level, modular implementation for an Industry 4.0 quality-control
node running on a Raspberry Pi.  The system fuses vision, proximity,
and environmental sensing for intelligent defect detection and
actuation, streaming structured telemetry to a cloud broker (PubNub).

Subsystems
----------
🧠  Vision Module:        HuskyLens AI camera for on-device inference
🌐  Cloud Telemetry:      PubNub with Auth-Key security
⚙️  Actuation Layer:      Servo ejector, RGB LED status, buzzer alerts
📡  Sensors:              Ultrasonic (distance) + MQ-2 (gas/anomaly)
💾  Edge Runtime:         Median filtering, debounce logic, event cache
====================================================================
"""

from __future__ import annotations
import os, time, statistics, asyncio, contextlib
from dataclasses import dataclass
from dotenv import load_dotenv
import RPi.GPIO as GPIO
from huskylib import HuskyLensLibrary
from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub

# ------------------------------------------------------------------
# Configuration Layer
# ------------------------------------------------------------------
load_dotenv()  # load secure keys from .env file

@dataclass(frozen=True)
class Pins:
    GREEN_LED: int = 22
    YELLOW_LED: int = 27
    RED_LED: int = 17
    SERVO: int = 16
    ULTRA_TRIG: int = 23
    ULTRA_ECHO: int = 25
    MQ2: int = 5
    BUZZER: int = 6


# ------------------------------------------------------------------
# Utility: PubNub Client
# ------------------------------------------------------------------
class CloudBridge:
    """Secure PubNub publisher with basic sync publishing."""
    def __init__(self):
        cfg = PNConfiguration()
        cfg.publish_key   = os.getenv("PUB_KEY")
        cfg.subscribe_key = os.getenv("SUB_KEY")
        cfg.uuid          = os.getenv("UUID")
        cfg.auth_key      = os.getenv("AUTH_TOKEN")
        self._pubnub = PubNub(cfg)

    def publish(self, event: str, **payload):
        msg = {"event": event, **payload}
        with contextlib.suppress(Exception):
            self._pubnub.publish().channel("block_channel").message(msg).sync()


# ------------------------------------------------------------------
# Hardware Abstraction Layer
# ------------------------------------------------------------------
class IOController:
    """Wraps GPIO primitives into expressive methods."""
    def __init__(self, pins: Pins):
        self.p = pins
        GPIO.setmode(GPIO.BCM)

        # Output devices
        for led in (self.p.GREEN_LED, self.p.YELLOW_LED, self.p.RED_LED):
            GPIO.setup(led, GPIO.OUT)
        GPIO.setup(self.p.SERVO, GPIO.OUT)
        GPIO.setup(self.p.BUZZER, GPIO.OUT)

        # Sensors
        GPIO.setup(self.p.ULTRA_TRIG, GPIO.OUT)
        GPIO.setup(self.p.ULTRA_ECHO, GPIO.IN)
        GPIO.setup(self.p.MQ2, GPIO.IN)

        # Servo init
        self._servo = GPIO.PWM(self.p.SERVO, 50)
        self._servo.start(7.5)
        print("[INIT] Hardware initialized.")

    # ------------- Actuators -------------
    def leds(self, *, green=False, yellow=False, red=False):
        GPIO.output(self.p.GREEN_LED, green)
        GPIO.output(self.p.YELLOW_LED, yellow)
        GPIO.output(self.p.RED_LED, red)

    def buzz(self, duration=0.8):
        GPIO.output(self.p.BUZZER, True)
        time.sleep(duration)
        GPIO.output(self.p.BUZZER, False)

    def servo_sweep(self):
        print("[ACTION] Servo sweep executing.")
        for duty in (7.5, 2.5, 7.5, 0):
            self._servo.ChangeDutyCycle(duty)
            time.sleep(0.3)

    # ------------- Sensors -------------
    def distance_cm(self, samples=5) -> float:
        """Median-filtered ultrasonic distance."""
        vals = []
        for _ in range(samples):
            GPIO.output(self.p.ULTRA_TRIG, False)
            time.sleep(0.0002)
            GPIO.output(self.p.ULTRA_TRIG, True)
            time.sleep(0.00001)
            GPIO.output(self.p.ULTRA_TRIG, False)

            start, end = None, None
            timeout = time.time() + 0.04
            while GPIO.input(self.p.ULTRA_ECHO) == 0 and time.time() < timeout:
                start = time.time()
            while GPIO.input(self.p.ULTRA_ECHO) == 1 and time.time() < timeout:
                end = time.time()
            if start and end:
                vals.append((end - start) * 17150)
        return statistics.median(vals) if vals else 999.0

    def gas_triggered(self, threshold=0.5) -> bool:
        """Simple threshold on MQ2 digital signal."""
        readings = [GPIO.input(self.p.MQ2) for _ in range(3)]
        return sum(readings)/len(readings) > threshold

    # ------------- Cleanup -------------
    def shutdown(self):
        self._servo.ChangeDutyCycle(0)
        self._servo.stop()
        GPIO.cleanup()
        print("[SYSTEM] GPIO cleanup complete.")


# ------------------------------------------------------------------
# Core Intelligence Layer
# ------------------------------------------------------------------
class SmartBlockSystem:
    """High-level orchestration of sensing, vision, and actuation."""

    def __init__(self):
        self.hw = IOController(Pins())
        self.cloud = CloudBridge()
        self.husky = self._init_huskylens()
        self.gas_counter = 0
        self.last_sweep = 0.0
        self.cooldown = 2.0

    @staticmethod
    def _init_huskylens():
        try:
            cam = HuskyLensLibrary("I2C", "", address=0x32)
            print("[INIT] HuskyLens connected.")
            return cam
        except Exception as e:
            print(f"[WARN] HuskyLens unavailable: {e}")
            return None

    # ------------- Sensor + Vision Fusion -------------
    def read_environment(self) -> dict:
        """Collect multi-sensor snapshot."""
        dist = self.hw.distance_cm()
        gas  = self.hw.gas_triggered()
        return {"distance_cm": round(dist, 1), "gas_alert": gas}

    def classify_block(self):
        """Obtain block ID using onboard AI vision."""
        if not self.husky:
            return None
        with contextlib.suppress(Exception):
            self.husky.requestAll()
            blocks = self.husky.learnedBlocks()
            if isinstance(blocks, list) and blocks:
                return blocks[0].ID
            elif blocks:
                return blocks.ID
        return None

    # ------------- Decision Logic -------------
    def handle_gas(self, gas_alert: bool):
        if gas_alert:
            self.gas_counter += 1
            self.hw.leds(red=True)
            self.hw.buzz(0.6)
            self.cloud.publish("Gas Detected", level=self.gas_counter)
            print(f"[ALERT] Gas anomaly detected ({self.gas_counter}/3).")
            if self.gas_counter >= 3:
                self.cloud.publish("System Shutdown", reason="Gas threshold exceeded")
                raise SystemExit("[CRITICAL] Gas threshold reached.")
        else:
            self.gas_counter = max(0, self.gas_counter - 1)

    def handle_block(self, block_id):
        """Decision path for block classification."""
        if block_id == 1:
            self.hw.leds(yellow=True)
            print(f"[DETECT] Defective block (ID {block_id}).")
            self.cloud.publish("Defective Block", block_id=block_id)
            if time.time() - self.last_sweep > self.cooldown:
                self.hw.servo_sweep()
                self.last_sweep = time.time()
        elif block_id is not None:
            self.hw.leds(green=True)
            print(f"[DETECT] Valid block (ID {block_id}).")
            self.cloud.publish("Valid Block", block_id=block_id)
        else:
            self.hw.leds(green=True)
            self.cloud.publish("No Block")

    # ------------- Main Loop -------------
    async def run(self):
        print("[SYSTEM] Smart Block Detection Node Active.")
        try:
            while True:
                env = self.read_environment()
                self.cloud.publish("Distance Reading", distance_cm=env["distance_cm"])
                self.handle_gas(env["gas_alert"])

                if env["distance_cm"] < 15:
                    self.hw.leds(red=True)
                    self.cloud.publish("Proximity Warning", distance=env["distance_cm"])
                    print("[WARN] Object too close. Pausing servo.")
                else:
                    block_id = self.classify_block()
                    self.handle_block(block_id)

                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("[SYSTEM] Interrupted by user.")
        finally:
            self.hw.shutdown()


# ------------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------------
if __name__ == "__main__":
    asyncio.run(SmartBlockSystem().run())
