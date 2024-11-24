import board
import busio
import digitalio
import time
import adafruit_vl53l4cd
from adafruit_ht16k33 import segments
from adafruit_max1704x import MAX17048

i2c = busio.I2C(board.SCL, board.SDA)
display = segments.BigSeg7x4(i2c)
sensor = adafruit_vl53l4cd.VL53L4CD(i2c)
battery = MAX17048(i2c)

display.brightness = 0.8
sensor.timing_budget = 100
sensor.inter_measurement = 100
sensor.signal_threshold = 1000

previous_display_value = -999
previous_actual_value = -999
last_time_without_error = 0  # time.monotonic() from the last reading that had a zero status code
reading_change_threshold = 1.0  # how much the reading must change to be considered a changed value
last_time_reading_changed = 0  # time.monotonic() from the last reading that changed more than the threshold
last_reading_time = 0  # time.monotonic() taken just before last reading
next_reading_time = 0  # time.monotonic() before which no new reading should be taken
ranging_pause_duration = 2.5  # how long to pause ranging for when there's been no change for a while
dim_display_after_time_without_change = 5.0  # dim display after this many seconds without change greater than threshold
switch_off_display_after_time_without_change = 30.0  # switch off display after this many seconds without change greater than threshold

# Switch of TFT screen to save energy. Press D0 to switch it on, D1 to switch it off.
board.DISPLAY.brightness = 0
button_d0 = digitalio.DigitalInOut(board.BUTTON)
button_d0.switch_to_input(pull=digitalio.Pull.UP)
button_d1 = digitalio.DigitalInOut(board.D1)
button_d1.switch_to_input(pull=digitalio.Pull.DOWN)

# Helper functions for avoiding unnecessary comms with display.
display_is_clear = True
def clear_display():
    global display_is_clear
    if not display_is_clear:
        display_is_clear = True
        display.fill(0)
        
def write_display(value):
    global display_is_clear
    display_is_clear = False
    display.print(f"{value:04d}")

# Helper functions for switching the distance sensor on and off.
ranging_is_paused = True
def start_ranging():
    global ranging_is_paused
    if not ranging_is_paused:
        return
    ranging_is_paused = False
    sensor.start_ranging()
    print("start ranging")
    
def stop_ranging():
    global ranging_is_paused
    if ranging_is_paused:
        return
    ranging_is_paused = True
    sensor.stop_ranging()
    print("stop ranging")

start_ranging()

while True:
    time.sleep(0.05)

    # when D0 button is pressed, switch on display
    if not button_d0.value:
        board.DISPLAY.brightness = 0.5

    # when D1 button is pressed, switch it off again
    if button_d1.value:
        board.DISPLAY.brightness = 0

    now = time.monotonic()

    if now < next_reading_time:
        continue

    # Time to take a reading but sensor is paused? Fire it up!
    if ranging_is_paused:
        start_ranging()

    if not sensor.data_ready:
        continue

    # There is data, a reading will be taken. Record timings.
    last_reading_time = now
    next_reading_time = now + 0.05  # default: read every 50ms

    # Read the sensor data.
    distance = sensor.distance  # in cm
    sigma = sensor.sigma  # noise in cm
    status = sensor.range_status
    print(f"{status:02} {distance:.1f} {sigma:.2f} {battery.cell_voltage:.1f}V {battery.cell_percent:.1f}% {previous_actual_value}")
    # Clear the interrupt to be ready for next measurement
    sensor.clear_interrupt()

    if status != 0 or sigma > 0.75:
        clear_display()
        if now - last_time_without_error > 5.0:
            next_reading_time = now + ranging_pause_duration
            stop_ranging()
        print(f"bad reading for {now - last_time_without_error}s")
        # every bad reading counts as a changed reading
        last_time_reading_changed = now
        previous_actual_value = -999
        continue

    last_time_without_error = now

    if abs(distance - previous_actual_value) < reading_change_threshold:
        time_since_last_reading = now - last_time_reading_changed
        if time_since_last_reading > dim_display_after_time_without_change:
            display.brightness = 0.2
            next_reading_time = now + ranging_pause_duration
            stop_ranging()
        print(f"same reading for {time_since_last_reading}s")
        if time_since_last_reading > switch_off_display_after_time_without_change:
            clear_display()
            continue
    else:
        last_time_reading_changed = now
        previous_actual_value = distance
        
    display_value = int(distance)
    if previous_display_value != display_value:
        previous_display_value = display_value
        write_display(display_value)
        display.brightness = 0.8

# status codes:
#
# 0 None Returned distance is valid
# 1 Warning Sigma is above the defined threshold (see Section 3.3 Sigma and signal thresholds)
# 2 Warning Signal is below the defined threshold (see Section 3.3 Sigma and signal thresholds)
# 3 Error Measured distance is below detection threshold
# 4 Error Phase out of valid limit
# 5 Error Hardware fail
# 6 Warning Phase valid but no wrap around check performed
# 7 Error Wrapped target, phase does not match
# 8 Error Processing fail
# 9 Error Crosstalk signal fail
# 10 Error Interrupt error
# 11 Error Merged target
# 12 Error Signal is too low
# 255 Error Other error (for example, boot error)

