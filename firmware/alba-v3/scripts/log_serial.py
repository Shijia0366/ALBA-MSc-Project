import csv
import queue
import serial
import serial.tools.list_ports
import threading
import time
from pathlib import Path

BAUD = 115200
CSV_TOGGLE_COMMAND = b"L"
CSV_HEADER_PREFIX = "time_s,encoder_count,motor_pos_rev"
EXPECTED_CSV_COLUMNS = 12

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data"
DATA_DIR.mkdir(exist_ok=True)

filename = DATA_DIR / time.strftime("trial_%Y%m%d_%H%M%S.csv")

COMMAND_HELP = """
Commands you can type here, then press Enter:
  e  enable motor
  d  disable motor
  f  forward
  b  backward
  s  stop motor / disable position control (use h to exit flow-speed control)
  +  faster
  -  slower
  z  zero encoder
  r  reset flow pulse count
  p  toggle position control
  0  target 0 rev
  1  target 1 rev
  2  target 2 rev
  v  valve ON
  c  valve OFF
  x  toggle valve
  a  start/restart relative-volume recording only
  g  start flow-speed controller / open valve
  h  stop flow-speed controller / close valve
  L  toggle CSV mode on ESP32
  SET_VOLUME 300  set target void volume in mL (> 0)
  SET_SPEED 200   set exact motor speed command in steps/s
  SET_POS_STOP 0.5  auto-stop at +0.5 encoder rev
  SET_POS_STOP OFF  disable motor-position auto-stop
  STOP_VOID       stop voiding, motor command, and valve
  STATUS          query voiding status
  q  quit logger
  ?  show this help
"""


def choose_port():
    ports = list(serial.tools.list_ports.comports())

    if not ports:
        raise RuntimeError("No serial ports found. Is the ESP32 plugged in?")

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")

    index = input("Select ESP32 port number: ")
    return ports[int(index)].device


def looks_like_csv_data(line):
    if line.startswith(CSV_HEADER_PREFIX):
        return True

    try:
        row = next(csv.reader([line]))
    except csv.Error:
        return False

    # Firmware emits one fixed-width CSV schema while CSV mode is enabled.
    if len(row) != EXPECTED_CSV_COLUMNS:
        return False

    try:
        float(row[0])
    except ValueError:
        return False

    return True


def print_live_status(line):
    try:
        row = next(csv.reader([line]))
    except csv.Error:
        return

    if not row or not row[0] or not row[0][0].isdigit():
        return

    # main CSV format:
    # time_s,encoder_count,motor_pos_rev,motor_vel_rev_s,
    # motor_cmd_steps_s,valve_state,void_state,flow_pulses,
    # flow_pulse_interval_us,flow_mL_s,flow_mL_s_filtered,
    # voided_volume_mL
    if len(row) != EXPECTED_CSV_COLUMNS:
        return

    time_s = row[0]
    encoder_count = row[1]
    pos_rev = row[2]
    vel_rev_s = row[3]
    cmd_steps_s = row[4]
    valve_state = row[5]
    void_state = row[6]
    flow_pulses = row[7]
    pulse_interval_us = row[8]
    flow_ml_s = row[9]
    filtered_flow_ml_s = row[10]
    voided_ml = row[11]

    status = (
        f"t={time_s}s | enc={encoder_count} | pos={pos_rev} rev | "
        f"vel={vel_rev_s} rev/s | cmd={cmd_steps_s} steps/s | "
        f"valve={valve_state} | state={void_state} | pulses={flow_pulses} | "
        f"interval={pulse_interval_us} us | Q={flow_ml_s} mL/s | "
        f"Q_filtered={filtered_flow_ml_s} mL/s | voided={voided_ml} mL"
    )

    print("\r" + status[:240] + " " * 40, end="", flush=True)


def keyboard_worker(command_queue, stop_event):
    while not stop_event.is_set():
        try:
            text = input()
        except EOFError:
            stop_event.set()
            break

        text = text.strip()

        if not text:
            continue

        command_queue.put(text)

        if text == "q":
            stop_event.set()
            break


def send_command(ser, command_text, stop_event):
    if command_text == "?":
        print("\n" + COMMAND_HELP)
        return

    if command_text == "q":
        print("\nQuit requested.")
        stop_event.set()
        return

    # Validate the ENTIRE command before writing any bytes. Silently dropping
    # a Unicode minus would turn a negative motor command into a positive one.
    wire_text = command_text if len(command_text) == 1 else command_text + "\n"
    try:
        payload = wire_text.encode("ascii", errors="strict")
    except UnicodeEncodeError:
        print("\nERROR: command not sent. Use ASCII characters and the normal '-' minus sign.")
        return
    ser.write(payload)
    ser.flush()

    print(f"\nSent command: {command_text}")


def main():
    port = choose_port()
    command_queue = queue.Queue()
    stop_event = threading.Event()

    print(f"Opening {port} at {BAUD} baud")
    print(f"Logging CSV data to {filename}")
    print("This script is your live monitor and command console while logging.")
    print("Do not open PlatformIO Serial Monitor at the same time.")
    print(COMMAND_HELP)
    print("Starting logger. Press Ctrl+C or type q + Enter to stop.\n")

    keyboard_thread = threading.Thread(
        target=keyboard_worker,
        args=(command_queue, stop_event),
        daemon=True,
    )
    keyboard_thread.start()

    with serial.Serial(port, BAUD, timeout=0.05) as ser, open(filename, "w", newline="") as f:
        time.sleep(2.0)
        ser.reset_input_buffer()

        # Ask ESP32 to enter CSV mode. This is the same as typing capital L.
        ser.write(CSV_TOGGLE_COMMAND)
        ser.flush()

        try:
            while not stop_event.is_set():
                while not command_queue.empty():
                    command_text = command_queue.get()
                    send_command(ser, command_text, stop_event)

                line = ser.readline().decode(errors="ignore").strip()

                # ESP32 live-monitor lines use carriage return "\r" instead of newline.
                # If Python reads several of those live updates in one chunk, keep only
                # the most recent segment so the terminal does not print a giant messy row.
                if "\r" in line:
                    line = line.split("\r")[-1].strip()

                if not line:
                    continue

                if looks_like_csv_data(line):
                    f.write(line + "\n")
                    f.flush()

                    if not line.startswith(CSV_HEADER_PREFIX):
                        print_live_status(line)
                else:
                    # Before the logger switches the ESP32 into CSV mode, the ESP32 may
                    # already be printing live status lines that use carriage returns.
                    # Those lines are useful in PlatformIO Serial Monitor, but they make
                    # the Python terminal look messy, so ignore them here.
                    if line.startswith("t_ms="):
                        continue
                    print("\n" + line)

        except KeyboardInterrupt:
            stop_event.set()

        finally:
            # Try to stop the hardware safely before closing serial.
            try:
                ser.write(b"h")
                ser.write(b"s")
                ser.flush()
            except serial.SerialException:
                pass

            print(f"\n\nStopped logging. Saved to {filename}")


if __name__ == "__main__":
    main()
