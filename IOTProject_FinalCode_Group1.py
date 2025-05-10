#!/usr/bin/env python3
"""
Smart Block Detection and Removal System

This Python script integrates key IoT components to automate defect detection in blocks
moving on a conveyor system. It combines real-time vision, proximity sensing, gas detection,
and cloud-based monitoring to ensure safety and quality control.

Components:
 - HuskyLens AI camera for visual classification
 - Ultrasonic sensor for proximity detection
 - MQ2 gas sensor for anomaly detection
 - Servo motor to eject defective blocks
 - RGB LEDs and buzzer for local alerts
 - PubNub for secure remote event publishing (with auth key)
"""

import RPi.GPIO as GPIO
import time
from huskylib import HuskyLensLibrary
from dotenv import load_dotenv
import os
from pubnub.pnconfiguration import PNConfiguration
from pubnub.pubnub import PubNub

# ----------------- Load Environment Variables -----------------
load_dotenv()

# ----------------- PubNub Configuration -----------------
pnconfig = PNConfiguration()
pnconfig.publish_key = os.getenv("PUB_KEY")
pnconfig.subscribe_key = os.getenv("SUB_KEY")
pnconfig.uuid = os.getenv("UUID")
pnconfig.auth_key = os.getenv("AUTH_TOKEN")  # Include auth token for secure access

pubnub = PubNub(pnconfig)

# Publishes a structured event to the block_channel
def publish_event(event_type, data=None):
    message = {"event": event_type}
    if data:
        message.update(data)
    try:
        pubnub.publish().channel("block_channel").message(message).sync()
    except Exception as e:
        print("[PUBNUB ERROR]", e)

# ----------------- GPIO Setup -----------------
GPIO.setmode(GPIO.BCM)

# LED Pins
GREEN_LED = 22
YELLOW_LED = 27
RED_LED = 17
GPIO.setup(GREEN_LED, GPIO.OUT)
GPIO.setup(YELLOW_LED, GPIO.OUT)
GPIO.setup(RED_LED, GPIO.OUT)

# Servo Configuration
SERVO_PIN = 16  # GPIO16 (Physical Pin 36)
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(7.5)  # Neutral starting point
print("[INIT] Servo initialized to neutral position")
time.sleep(1.0)

# Ultrasonic Sensor Pins
ULTRA_TRIG = 23
ULTRA_ECHO = 25
GPIO.setup(ULTRA_TRIG, GPIO.OUT)
GPIO.setup(ULTRA_ECHO, GPIO.IN)

# MQ2 Gas Sensor and Buzzer
MQ2_PIN = 5
BUZZER_PIN = 6
GPIO.setup(MQ2_PIN, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# HuskyLens Initialization
try:
    huskyLens = HuskyLensLibrary("I2C", "", address=0x32)
except Exception as e:
    print("[ERROR] HuskyLens initialization failed:", e)
    huskyLens = None

# ----------------- Helper Functions -----------------

def set_leds(green=False, yellow=False, red=False):
    GPIO.output(GREEN_LED, green)
    GPIO.output(YELLOW_LED, yellow)
    GPIO.output(RED_LED, red)

# Returns a single distance measurement from ultrasonic sensor
def measure_distance_once():
    GPIO.output(ULTRA_TRIG, False)
    time.sleep(0.0002)
    GPIO.output(ULTRA_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(ULTRA_TRIG, False)
    pulse_start, pulse_end = None, None
    timeout = time.time() + 0.04
    while GPIO.input(ULTRA_ECHO) == 0 and time.time() < timeout:
        pulse_start = time.time()
    if pulse_start is None:
        return 999
    timeout = time.time() + 0.04
    while GPIO.input(ULTRA_ECHO) == 1 and time.time() < timeout:
        pulse_end = time.time()
    if pulse_end is None:
        return 999
    return (pulse_end - pulse_start) * 17150

# Averages multiple ultrasonic readings for accuracy
def measure_distance():
    distances = sorted([measure_distance_once() for _ in range(7)])
    return distances[len(distances)//2]

# Executes a servo sweep to remove defective blocks
def sweep_bad_block_servo():
    print("[ACTION] Executing servo sweep...")
    servo_pwm.ChangeDutyCycle(7.5)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(2.5)
    time.sleep(0.5)
    servo_pwm.ChangeDutyCycle(7.5)
    time.sleep(0.3)
    servo_pwm.ChangeDutyCycle(0)
    print("[ACTION] Sweep complete.")

# Gas detection with debounce logic and alerting

gas_count = 0
no_gas_counter = 0
shutdown_requested = False

def check_gas_and_alert():
    global gas_count, no_gas_counter, shutdown_requested
    gas_readings = [GPIO.input(MQ2_PIN) for _ in range(3)]
    avg_gas_value = sum(gas_readings) / len(gas_readings)
    if avg_gas_value > 0.5:
        gas_count += 1
        no_gas_counter = 0
        print(f"[ALERT] Gas detected ({gas_count}/3) | MQ2 Reading: {avg_gas_value:.2f}")
        publish_event("Gas Detected", {"level": gas_count})
        GPIO.output(BUZZER_PIN, GPIO.HIGH)
        GPIO.output(RED_LED, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(BUZZER_PIN, GPIO.LOW)
        GPIO.output(RED_LED, GPIO.LOW)
        if gas_count >= 3:
            publish_event("System Shutdown", {"reason": "Gas threshold exceeded"})
            shutdown_requested = True
    else:
        no_gas_counter += 1
        if no_gas_counter >= 3:
            gas_count = 0

# ----------------- Main Control Loop -----------------

last_servo_time = 0
servo_cooldown = 2

try:
    while True:
        if shutdown_requested:
            print("[SYSTEM] Shutting down due to repeated gas detection.")
            break

        distance = measure_distance()
        print(f"[INFO] Ultrasonic Distance: {distance:.1f} cm")
        publish_event("Distance Reading", {"distance_cm": round(distance, 1)})
        check_gas_and_alert()

        if distance < 15:
            print("[WARNING] Object too close. System paused.")
            set_leds(red=True, green=False, yellow=False)
            publish_event("Proximity Warning", {"distance": distance})
        else:
            current_block = None
            if huskyLens:
                try:
                    huskyLens.requestAll()
                    time.sleep(0.2)
                    blocks = huskyLens.learnedBlocks()
                    if isinstance(blocks, list) and len(blocks) > 0:
                        current_block = blocks[0]
                    elif blocks is not None:
                        current_block = blocks
                except Exception:
                    pass

            if current_block:
                block_id = current_block.ID
                if block_id == 1:
                    set_leds(yellow=True)
                    print(f"[DETECT] Defective block detected (ID {block_id}).")
                    publish_event("Defective Block", {"block_id": block_id})
                    if time.time() - last_servo_time > servo_cooldown:
                        print("[ACTION] Initiating servo action.")
                        sweep_bad_block_servo()
                        last_servo_time = time.time()
                else:
                    set_leds(green=True)
                    print(f"[DETECT] Valid block detected (ID {block_id}).")
                    publish_event("Valid Block", {"block_id": block_id})
            else:
                set_leds(green=True)
                print("[INFO] No block detected in frame.")
                publish_event("No Block")

        time.sleep(1)

except KeyboardInterrupt:
    print("[SYSTEM] Interrupted by user.")

finally:
    servo_pwm.ChangeDutyCycle(0)
    servo_pwm.stop()
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.cleanup()
    print("[SYSTEM] GPIO cleanup complete. Program terminated.")
