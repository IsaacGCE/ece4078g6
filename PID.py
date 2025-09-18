import socket
import struct
import io
import threading
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB  = 25
LEFT_MOTOR_IN3  = 23
LEFT_MOTOR_IN4  = 24
LEFT_ENCODER    = 26
RIGHT_ENCODER   = 16

# PID Constants (default values, will be overridden by client)
use_PID = 0
KP, KI, KD = 0.0, 0.0, 0.0          # <-- use uppercase KI consistently
MAX_CORRECTION = 30                 # Maximum PWM correction value (fixed clamp)

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE = 250                     # PWM units per second (adjust this value to tune ramp speed)
MIN_RAMP_THRESHOLD = 15             # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 15
current_movement, prev_movement = 'stop', 'stop'

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB,  GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3,  GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4,  GPIO.OUT)

    # Prevent slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB,  GPIO.LOW)

    # Encoders (single-channel) + edge detect
    GPIO.setup(LEFT_ENCODER,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER,  GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)

    # PWM at 100 Hz
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm  = GPIO.PWM(LEFT_MOTOR_ENB,  100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    if (prev_left_state is not None and current_state != prev_left_state):
        left_count += 1
        prev_left_state = current_state
    elif prev_left_state is None:
        prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state
    current_state = GPIO.input(RIGHT_ENCODER)
    if (prev_right_state is not None and current_state != prev_right_state):
        right_count += 1
        prev_right_state = current_state
    elif prev_right_state is None:
        prev_right_state = current_state

def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement

    # Pre-Start Kick (Motor Priming) for forward/backward only
    if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
        if current_movement == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH); GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3,  GPIO.HIGH); GPIO.output(LEFT_MOTOR_IN4,  GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW);  GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3,  GPIO.LOW);  GPIO.output(LEFT_MOTOR_IN4,  GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    # Right motor
    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH); GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW);  GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # Active Braking when pwm == 0
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH); GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)

    # Left motor
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH); GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW);  GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH); GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)

def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0
    elif abs(pwm_value) < min_threshold:
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value

def pid_control():
    # Only applies for forward/backward, not turning
    global left_pwm, right_pwm, left_count, right_count, use_PID, KP, KI, KD, prev_movement, current_movement

    # --- Straight-line PID state ---
    integral = 0.0
    last_error = 0.0
    last_time = monotonic()

    # --- Turning PID state (separate memory) ---
    turn_integral = 0.0
    turn_last_error = 0.0

    # --- For counts/sec estimation ---
    prev_left_count = 0
    prev_right_count = 0

    # --- EMA smoothing for turning speeds ---
    ema_speed_L = 0.0
    ema_speed_R = 0.0
    EMA_ALPHA = 0.3  # 0..1; higher = less smoothing

    # Ramping variables & params
    ramp_left_pwm = 0.0
    ramp_right_pwm = 0.0
    previous_left_target = 0.0
    previous_right_target = 0.0

    # Track which side is "outer" across loops (for gentle integral resets)
    last_outer_is_right = None

    while running:
        current_time = monotonic()
        dt = current_time - last_time
        last_time = current_time
        if dt <= 0:
            dt = 1e-3  # avoid divide-by-zero if clock hiccups

        prev_movement = current_movement
        if (left_pwm > 0 and right_pwm > 0):
            current_movement = 'forward'
        elif (left_pwm < 0 and right_pwm < 0):
            current_movement = 'backward'
        elif (left_pwm == 0 and right_pwm == 0):
            current_movement = 'stop'
        else:
            current_movement = 'turn'

        if not use_PID:
            target_left_pwm = left_pwm
            target_right_pwm = right_pwm

        else:
            if current_movement == 'forward' or current_movement == 'backward':
                # ------- YOUR ORIGINAL STRAIGHT PID (unchanged) -------
                error = left_count - right_count
                proportional = KP * error
                integral += KI * error * dt
                integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
                derivative = KD * (error - last_error) / dt if dt > 0 else 0.0
                correction = proportional + integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                last_error = error

                if current_movement == 'backward':
                    correction = -correction

                target_left_pwm  = left_pwm  - correction
                target_right_pwm = right_pwm + correction

            elif current_movement == 'turn':
                # ------- TURNING PID (ratio match: outer / inner) -------
                # Reset straight-line accumulators while turning
                integral = 0.0
                last_error = 0.0

                # Base targets start at commands (we adjust around them)
                target_left_pwm = left_pwm
                target_right_pwm = right_pwm

                # Per-loop encoder deltas
                d_left  = left_count  - prev_left_count
                d_right = right_count - prev_right_count
                prev_left_count  = left_count
                prev_right_count = right_count

                # Raw "speeds" (ticks/sec) -> smooth with EMA
                speed_left_raw  = d_left  / dt
                speed_right_raw = d_right / dt
                ema_speed_L = (1.0 - EMA_ALPHA) * ema_speed_L + EMA_ALPHA * speed_left_raw
                ema_speed_R = (1.0 - EMA_ALPHA) * ema_speed_R + EMA_ALPHA * speed_right_raw
                speed_left  = ema_speed_L
                speed_right = ema_speed_R

                # Decide inner vs outer by commanded magnitudes
                left_pwm_abs  = abs(left_pwm)
                right_pwm_abs = abs(right_pwm)

                if right_pwm_abs >= left_pwm_abs:
                    # Right is OUTER, Left is INNER
                    outer_cmd, inner_cmd = right_pwm_abs, left_pwm_abs
                    outer_speed, inner_speed = speed_right, speed_left
                    outer_is_right = True
                else:
                    # Left is OUTER, Right is INNER
                    outer_cmd, inner_cmd = left_pwm_abs, right_pwm_abs
                    outer_speed, inner_speed = speed_left, speed_right
                    outer_is_right = False

                # Gentle integral reset if outer side flips, or near-zero commands
                if (last_outer_is_right is None) or (outer_is_right != last_outer_is_right) \
                   or (outer_cmd < 5.0 or inner_cmd < 5.0):
                    turn_integral = 0.0
                last_outer_is_right = outer_is_right

                # Desired ratio from commands (clamped to sane range)
                eps = 1e-6
                turning_ratio = outer_cmd / (inner_cmd + eps)
                turning_ratio = max(0.5, min(turning_ratio, 3.0))

                # Normalized ratio error: bounded & scale-invariant
                num = outer_speed - turning_ratio * inner_speed
                den = outer_speed + turning_ratio * inner_speed + eps
                turn_error = num / den   # roughly in [-1, 1]

                # PID on turn_error (separate memory)
                turn_proportional = KP * turn_error
                turn_integral += KI * turn_error * dt
                turn_integral = max(-MAX_CORRECTION, min(turn_integral, MAX_CORRECTION))  # Anti-windup
                turn_derivative = KD * (turn_error - turn_last_error) / dt if dt > 0 else 0.0
                turn_correction = turn_proportional + turn_integral + turn_derivative
                turn_correction = max(-MAX_CORRECTION, min(turn_correction, MAX_CORRECTION))
                turn_last_error = turn_error

                # Helpers to adjust magnitude without flipping sign
                def inc_mag(pwm, d): return pwm + d if pwm >= 0 else pwm - d
                def dec_mag(pwm, d): return pwm - d if pwm >= 0 else pwm + d

                # Apply correction: if outer too fast (error>0) slow outer / boost inner; else the reverse.
                if outer_is_right:
                    if turn_correction >= 0:
                        target_right_pwm = dec_mag(right_pwm, turn_correction)
                        target_left_pwm  = inc_mag(left_pwm,  turn_correction)
                    else:
                        c = -turn_correction
                        target_right_pwm = inc_mag(right_pwm, c)
                        target_left_pwm  = dec_mag(left_pwm,  c)
                else:
                    if turn_correction >= 0:
                        target_left_pwm  = dec_mag(left_pwm,  turn_correction)
                        target_right_pwm = inc_mag(right_pwm, turn_correction)
                    else:
                        c = -turn_correction
                        target_left_pwm  = inc_mag(left_pwm,  c)
                        target_right_pwm = dec_mag(right_pwm, c)

            else:
                # Reset when stopped
                integral = 0.0
                last_error = 0.0
                reset_encoder()
                target_left_pwm  = left_pwm
                target_right_pwm = right_pwm

        # ---------- RAMPING (unchanged) ----------
        if use_ramping and use_PID:
            max_change_per_cycle = RAMP_RATE * dt

            left_diff  = target_left_pwm  - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm

            left_needs_ramp  = abs(left_diff)  > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD

            left_direction_change  = (target_left_pwm  * previous_left_target  < 0) and target_left_pwm  != 0 and previous_left_target  != 0
            right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0

            if left_direction_change:
                ramp_left_pwm = target_left_pwm
            if right_direction_change:
                ramp_right_pwm = target_right_pwm

            if not left_direction_change and not right_direction_change:
                if left_needs_ramp or right_needs_ramp:
                    # Left
                    if abs(left_diff) <= max_change_per_cycle:
                        ramp_left_pwm = target_left_pwm
                    else:
                        ramp_left_pwm += max_change_per_cycle if left_diff > 0 else -max_change_per_cycle
                    # Right
                    if abs(right_diff) <= max_change_per_cycle:
                        ramp_right_pwm = target_right_pwm
                    else:
                        ramp_right_pwm += max_change_per_cycle if right_diff > 0 else -max_change_per_cycle
                else:
                    ramp_left_pwm  = target_left_pwm
                    ramp_right_pwm = target_right_pwm

            previous_left_target  = target_left_pwm
            previous_right_target = target_right_pwm

        else:
            ramp_left_pwm  = target_left_pwm
            ramp_right_pwm = target_right_pwm

        final_left_pwm  = apply_min_threshold(ramp_left_pwm,  MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)

        # Debug print
        if ramp_left_pwm != 0 or ramp_right_pwm != 0:
            print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")

        time.sleep(0.01)

def camera_stream_server():
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
    picam2.configure(camera_config)
    picam2.start()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print("Camera stream client connected")
            while running:
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break
                time.sleep(0.01)
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")
        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()
    picam2.stop()

def pid_config_server():
    global use_PID, KP, KI, KD

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print("PID config client connected")
            try:
                # Receive PID constants (4 floats)
                data = client_socket.recv(16)
                if data and len(data) == 16:
                    use_PID, KP, KI, KD = struct.unpack("!ffff", data)  # Keep KI uppercase
                    if use_PID:
                        print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
                    else:
                        print("The robot is not using PID.")
                    response = struct.pack("!i", 1)
                else:
                    response = struct.pack("!i", 0)
                client_socket.sendall(response)
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    client_socket.sendall(struct.pack("!i", 0))
                except:
                    pass
            client_socket.close()
        except Exception as e:
            print(f"PID config server error: {str(e)}")

    server_socket.close()

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")

    while running:
        try:
            client_socket, _ = server_socket.accept()
            print("Wheel client connected")
            while running:
                try:
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Wheel client sending speed error")
                        break
                    left_speed, right_speed = struct.unpack("!ff", data)
                    left_pwm, right_pwm = left_speed*100, right_speed*100
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                except Exception as e:
                    print("Wheel client disconnected")
                    break
        except Exception as e:
            print(f"Wheel server error: {str(e)}")
        if 'client_socket' in locals() and client_socket:
            client_socket.close()

    server_socket.close()

def main():
    try:
        setup_gpio()

        pid_thread = threading.Thread(target=pid_control, daemon=True)
        pid_thread.start()

        camera_thread = threading.Thread(target=camera_stream_server, daemon=True)
        camera_thread.start()

        pid_config_thread = threading.Thread(target=pid_config_server, daemon=True)
        pid_config_thread.start()

        wheel_server()

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")

if __name__ == "__main__":
    main()
