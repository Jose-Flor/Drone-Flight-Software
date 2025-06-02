from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import cv2
import numpy as np
import imutils

# === Parameters ===
TARGET_ALTITUDE = 10  # meters
WAYPOINTS = [
    (0.0002, 0.0002),
    (0.0004, 0.0001)
]
AVOID_RADIUS = 100
CRITICAL_BATTERY = 20  # percent

# === Connect to Vehicle ===
print("Connecting to vehicle...")
vehicle = connect('COM3', baud=57600, wait_ready=True)  # make sure to check my port

# === OpenCV Camera Feed ===
cap = cv2.VideoCapture(0)

def detect_obstacle(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    for c in cnts:
        if cv2.contourArea(c) > 1000:
            (x, y, w, h) = cv2.boundingRect(c)
            center_x = x + w // 2
            if abs(center_x - frame.shape[1] // 2) < AVOID_RADIUS:
                print("🚨 Obstacle detected!")
                return True
    return False

def wait_for_gps_and_arm():
    print("Waiting for GPS lock and arming...")
    while not vehicle.is_armable:
        print("  Waiting for vehicle to become armable...")
        time.sleep(2)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("  Switching to GUIDED...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("  Waiting for arming...")
        time.sleep(1)
    print("✅ Armed.")

def takeoff(alt):
    print(f"Taking off to {alt} meters...")
    vehicle.simple_takeoff(alt)
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_alt:.2f} m")
        if current_alt >= alt * 0.95:
            print("✅ Reached target altitude.")
            break
        time.sleep(1)

def go_to_location(lat_offset, lon_offset):
    home = vehicle.location.global_relative_frame
    target = LocationGlobalRelative(
        home.lat + lat_offset,
        home.lon + lon_offset,
        TARGET_ALTITUDE
    )
    print(f"Flying to waypoint: {target.lat}, {target.lon}")
    vehicle.simple_goto(target)

    for _ in range(30):
        ret, frame = cap.read()
        if not ret:
            continue
        if detect_obstacle(frame):
            print("⛔ Obstacle! Hovering...")
            vehicle.mode = VehicleMode("LOITER")
            time.sleep(3)
            vehicle.mode = VehicleMode("GUIDED")
        if vehicle.battery.level and vehicle.battery.level < CRITICAL_BATTERY:
            print("⚠️ Low battery! Returning...")
            vehicle.mode = VehicleMode("RTL")
            return False
        time.sleep(1)
    return True

# === Mission Start ===
wait_for_gps_and_arm()
takeoff(TARGET_ALTITUDE)

for i, (lat_off, lon_off) in enumerate(WAYPOINTS):
    print(f"\n📍 Navigating to Waypoint {i+1}...")
    if not go_to_location(lat_off, lon_off):
        break

print("🏁 Mission complete. Returning to launch...")
vehicle.mode = VehicleMode("RTL")

cap.release()
vehicle.close()
print("✅ Drone landed and disconnected.")
