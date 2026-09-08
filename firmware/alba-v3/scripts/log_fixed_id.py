"""Record fixed-weight, open-loop flow identification trials from the ESP32.

This logger treats load markers and repetitions as host-side metadata.  It does
not reset the ESP32 pulse counter and never sends a motor-control command.
"""

import csv
import math
import queue
import re
import threading
import time
from collections import deque
from decimal import Decimal, InvalidOperation
from pathlib import Path

import serial
import serial.tools.list_ports


BAUD = 115200
CSV_TOGGLE_COMMAND = b"L"
FLOW_PULSES_PER_LITER = 1380.0
UINT32_MASK = (1 << 32) - 1
FLUSH_INTERVAL_S = 1.0

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = PROJECT_ROOT / "data"
DATA_DIR.mkdir(parents=True, exist_ok=True)

ESP32_FIELDS = (
    "time_s",
    "encoder_count",
    "motor_pos_rev",
    "motor_vel_rev_s",
    "motor_cmd_steps_s",
    "valve_state",
    "void_state",
    "flow_pulses",
    "flow_pulse_interval_us",
    "flow_mL_s",
    "flow_mL_s_filtered",
    "voided_volume_mL",
)

FIXED_ID_FIELDS = (
    "mass_g",
    "nominal_load_N",
    "repeat_id",
    "initial_volume_mL",
    "device_time_ms",
    "trial_time_s",
    "load_state",
    "event_code",
    "flow_pulses_total",
    "flow_pulses_trial",
    "flow_pulse_interval_us",
    "flow_mL_s",
    "flow_mL_s_filtered",
    "voided_volume_trial_mL",
    "valve_state",
    "void_state",
)

SAFE_FORWARD_COMMANDS = {"a", "v", "h", "c"}

COMMAND_HELP = """
Fixed-weight ID commands (type a command, then press Enter):
  <mass>  start a trial, for example 50 or 62.5 (grams)
  l       mark weight loaded: event_code=1, load_state=1
  u       mark weight unloaded: event_code=2, load_state=0
  r       save this trial and immediately start another repeat at the same mass
  e       save this trial and return to WAITING_FOR_MASS
  q       safely save, close the serial port, and quit
  a/v/h/c optionally forward the matching existing command to the ESP32
  ?       show this help

The script automatically sends only capital L to enable ESP32 CSV output.
It does not send g or any motor-control command.
"""


def choose_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Is the ESP32 plugged in?")

    if len(ports) == 1:
        print(f"Automatically selected the only serial port: {ports[0].device}")
        return ports[0].device

    esp32_keywords = (
        "CP210",
        "SILICON LABS",
        "CH340",
        "CH341",
        "CH910",
        "USB SERIAL",
        "USB-SERIAL",
        "UART",
        "ESP32",
        "FTDI",
    )
    esp32_candidates = []

    for port in ports:
        identity = " ".join(
            str(value or "")
            for value in (
                port.description,
                port.manufacturer,
                port.product,
                port.hwid,
            )
        ).upper()

        if "BLUETOOTH" in identity or "BTHENUM" in identity:
            continue

        if any(keyword in identity for keyword in esp32_keywords):
            esp32_candidates.append(port)

    if len(esp32_candidates) == 1:
        selected = esp32_candidates[0]
        print(
            f"Automatically selected ESP32 serial port: {selected.device} "
            f"- {selected.description}"
        )
        return selected.device

    selectable_ports = esp32_candidates if esp32_candidates else ports
    if len(esp32_candidates) > 1:
        print("Multiple possible ESP32 serial ports were found.")
    else:
        print("The ESP32 serial port could not be identified automatically.")

    print("Available serial ports:")
    for index, port in enumerate(selectable_ports):
        print(f"[{index}] {port.device} - {port.description}")

    selected = int(input("Select ESP32 port number: ").strip())
    return selectable_ports[selected].device


def prompt_positive_float(prompt):
    while True:
        text = input(prompt).strip()
        try:
            value = float(text)
        except ValueError:
            print("Enter a positive number.")
            continue
        if value > 0.0 and value < float("inf"):
            return value
        print("Enter a finite value greater than zero.")


def parse_mass(text):
    try:
        mass = Decimal(text)
    except InvalidOperation:
        return None
    if not mass.is_finite() or mass <= 0:
        return None
    return mass


def mass_filename_token(mass):
    normalized = format(mass.normalize(), "f")
    if "." in normalized:
        normalized = normalized.rstrip("0").rstrip(".")
    return normalized.replace(".", "p")


def next_trial_path(mass):
    token = mass_filename_token(mass)
    pattern = re.compile(rf"^{re.escape(token)}_(\d{{3}})\.csv$")
    highest_repeat = 0
    for path in DATA_DIR.glob(f"{token}_*.csv"):
        match = pattern.match(path.name)
        if match:
            highest_repeat = max(highest_repeat, int(match.group(1)))

    repeat_id = highest_repeat + 1
    while True:
        path = DATA_DIR / f"{token}_{repeat_id:03d}.csv"
        if not path.exists():
            return path, repeat_id
        repeat_id += 1


class TrialWriter:
    def __init__(self, mass, initial_volume_ml):
        self.mass = mass
        self.mass_g = float(mass)
        self.nominal_load_n = self.mass_g * 9.81 / 1000.0
        self.initial_volume_ml = initial_volume_ml
        self.path, self.repeat_id = next_trial_path(mass)
        self.file = open(self.path, "x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=FIXED_ID_FIELDS)
        self.writer.writeheader()

        self.start_time_ms = None
        self.start_pulses = None
        self.load_state = 0
        self.pending_events = deque()
        self.row_count = 0
        self.last_flush_time = time.monotonic()
        self.last_status_time = 0.0
        self.closed = False

        print(
            f"TRIAL_ACTIVE: mass={self.mass_g:g} g, repeat={self.repeat_id:03d}, "
            f"file={self.path.resolve()}"
        )

    def mark_loaded(self):
        self.load_state = 1
        self.pending_events.append((1, 1))
        print("Load marker queued: loaded (event_code=1).")

    def mark_unloaded(self):
        self.load_state = 0
        self.pending_events.append((2, 0))
        print("Load marker queued: unloaded (event_code=2).")

    def write_esp32_row(self, source):
        try:
            device_time_s = float(source["time_s"])
            device_time_ms = int(round(device_time_s * 1000.0)) & UINT32_MASK
            pulse_total = int(source["flow_pulses"]) & UINT32_MASK
            pulse_interval_us = int(source["flow_pulse_interval_us"]) & UINT32_MASK
            flow_ml_s = float(source["flow_mL_s"])
            filtered_ml_s = float(source["flow_mL_s_filtered"])
            valve_state = int(source["valve_state"])
        except (KeyError, TypeError, ValueError):
            return False

        if not all(
            math.isfinite(value)
            for value in (device_time_s, flow_ml_s, filtered_ml_s)
        ):
            return False

        if self.start_time_ms is None:
            self.start_time_ms = device_time_ms
            self.start_pulses = pulse_total

        elapsed_ms = (device_time_ms - self.start_time_ms) & UINT32_MASK
        trial_pulses = (pulse_total - self.start_pulses) & UINT32_MASK
        trial_volume_ml = trial_pulses * 1000.0 / FLOW_PULSES_PER_LITER

        if self.pending_events:
            event_code, row_load_state = self.pending_events.popleft()
        else:
            event_code = 0
            row_load_state = self.load_state

        self.writer.writerow(
            {
                "mass_g": f"{self.mass_g:g}",
                "nominal_load_N": f"{self.nominal_load_n:.6f}",
                "repeat_id": self.repeat_id,
                "initial_volume_mL": f"{self.initial_volume_ml:g}",
                "device_time_ms": device_time_ms,
                "trial_time_s": f"{elapsed_ms / 1000.0:.3f}",
                "load_state": row_load_state,
                "event_code": event_code,
                "flow_pulses_total": pulse_total,
                "flow_pulses_trial": trial_pulses,
                "flow_pulse_interval_us": pulse_interval_us,
                "flow_mL_s": f"{flow_ml_s:.6f}",
                "flow_mL_s_filtered": f"{filtered_ml_s:.6f}",
                "voided_volume_trial_mL": f"{trial_volume_ml:.6f}",
                "valve_state": valve_state,
                "void_state": source["void_state"],
            }
        )
        self.row_count += 1

        now = time.monotonic()
        if event_code or now - self.last_flush_time >= FLUSH_INTERVAL_S:
            self.file.flush()
            self.last_flush_time = now

        if now - self.last_status_time >= 1.0:
            print(
                f"\rmass={self.mass_g:g} g | repeat={self.repeat_id:03d} | "
                f"t={elapsed_ms / 1000.0:.1f} s | Q={filtered_ml_s:.3f} mL/s | "
                f"pulses={trial_pulses} | volume={trial_volume_ml:.2f} mL",
                end="",
                flush=True,
            )
            self.last_status_time = now

        return True

    def close(self):
        if self.closed:
            return
        self.file.flush()
        self.file.close()
        self.closed = True
        print(
            f"\nSaved {self.path.resolve()} ({self.row_count} valid data rows)."
        )


def keyboard_worker(command_queue):
    while True:
        try:
            command_queue.put(input().strip())
        except EOFError:
            command_queue.put("q")
            return


def parse_esp32_line(line, header_index):
    try:
        row = next(csv.reader([line]))
    except csv.Error:
        return header_index, None

    if tuple(row) == ESP32_FIELDS:
        return {name: index for index, name in enumerate(row)}, None

    if header_index is None or len(row) != len(ESP32_FIELDS):
        return header_index, None

    try:
        float(row[header_index["time_s"]])
    except (KeyError, ValueError):
        return header_index, None

    return header_index, {
        name: row[index] for name, index in header_index.items()
    }


def process_command(text, ser, trial, initial_volume_ml):
    command = text.strip().lower()
    if not command:
        return trial, False

    if command == "?":
        print("\n" + COMMAND_HELP)
        return trial, False

    if command == "q":
        return trial, True

    if command == "l":
        if trial is None:
            print("Enter a mass first; state is WAITING_FOR_MASS.")
        else:
            trial.mark_loaded()
        return trial, False

    if command == "u":
        if trial is None:
            print("Enter a mass first; state is WAITING_FOR_MASS.")
        else:
            trial.mark_unloaded()
        return trial, False

    if command == "r":
        if trial is None:
            print("No active trial to repeat; enter a mass first.")
            return trial, False
        mass = trial.mass
        trial.close()
        return TrialWriter(mass, initial_volume_ml), False

    if command == "e":
        if trial is not None:
            trial.close()
        print("WAITING_FOR_MASS: enter the next total suspended mass in grams.")
        return None, False

    if command in SAFE_FORWARD_COMMANDS:
        ser.write(command.encode("ascii"))
        ser.flush()
        print(f"Forwarded optional ESP32 command: {command}")
        return trial, False

    mass = parse_mass(command)
    if mass is not None:
        if trial is not None:
            print("A trial is already active. Type e before entering a new mass.")
            return trial, False
        return TrialWriter(mass, initial_volume_ml), False

    if command == "g":
        print("Command g is disabled in the fixed-weight logger (no motor/PID control).")
    else:
        print("Unknown command. Type ? for fixed-weight logger help.")
    return trial, False


def main():
    initial_volume_ml = prompt_positive_float("Initial bladder volume (mL): ")
    port = choose_port()
    command_queue = queue.Queue()
    stop_requested = False
    trial = None
    header_index = None

    print(f"Opening {port} at {BAUD} baud")
    print(f"Data directory: {DATA_DIR.resolve()}")
    print("Do not open PlatformIO Serial Monitor on the same port.")
    print(COMMAND_HELP)
    print("WAITING_FOR_MASS: enter the total suspended mass in grams.")

    keyboard_thread = threading.Thread(
        target=keyboard_worker, args=(command_queue,), daemon=True
    )
    keyboard_thread.start()

    try:
        with serial.Serial(port, BAUD, timeout=0.05) as ser:
            time.sleep(2.0)
            ser.reset_input_buffer()

            # This is the only command sent automatically.  CSV mode is off
            ser.write(CSV_TOGGLE_COMMAND)
            ser.flush()

            while not stop_requested:
                while not command_queue.empty():
                    trial, stop_requested = process_command(
                        command_queue.get(), ser, trial, initial_volume_ml
                    )
                    if stop_requested:
                        break

                if stop_requested:
                    break

                line = ser.readline().decode(errors="ignore").strip()
                if "\r" in line:
                    line = line.split("\r")[-1].strip()
                if not line:
                    continue

                header_index, source = parse_esp32_line(line, header_index)
                if source is not None and trial is not None:
                    trial.write_esp32_row(source)

    except KeyboardInterrupt:
        print("\nCtrl+C received; closing the active trial safely.")
    except serial.SerialException as error:
        print(f"\nSerial error: {error}")
    finally:
        if trial is not None:
            trial.close()
        print("Logger stopped; serial port closed.")


if __name__ == "__main__":
    main()
