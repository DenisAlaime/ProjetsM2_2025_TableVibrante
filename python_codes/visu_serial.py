import sys
import threading
import time
import io
import csv
import signal
import serial
from serial.tools import list_ports
import numpy as np
# import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt

#!/usr/bin/env python3
"""
visu_serial.py

Simple serial passthrough with a GRAPH mode:
- In passthrough mode, serial lines are printed to the terminal and you can type to send to the device.
- If a serial line starts with "GRAPH", the program enters graph mode:
    - Next line is expected to be a CSV header.
    - Subsequent lines are CSV data rows.
    - When a line containing "END" is received, the buffered CSV is plotted and the program returns to passthrough mode.

Dependencies:
    pip install pyserial matplotlib numpy
"""

# Image output parameters (pixels and DPI)
PLOT_WIDTH_PX = 4800
PLOT_HEIGHT_PX = 2400
PLOT_DPI = 300

RUNNING = True

def choose_port():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Enter port name manually (e.g. COM3 or /dev/ttyUSB0):")
        return input("> ").strip()
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    choice = input("Choose port index or enter port name (default 0): ").strip()
    if choice == "":
        return ports[0].device
    if choice.isdigit():
        idx = int(choice)
        if 0 <= idx < len(ports):
            return ports[idx].device
    return choice

def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        # small delay for some Arduinos
        time.sleep(0.1)
        print(f"Opened {port} at {baud} baud.")
        return ser
    except Exception as e:
        print(f"Failed to open serial port {port}: {e}")
        sys.exit(1)

class SerialHandler:
    def __init__(self, ser):
        self.ser = ser
        self.mode = "PASSTHROUGH"  # or "GRAPH"
        self.buffer = []  # buffered CSV lines (strings)
        self.header = None
        self.lock = threading.Lock()

    def start_reader(self):
        t = threading.Thread(target=self._reader_loop, daemon=True)
        t.start()
        return t

    def _reader_loop(self):
        # print the current thread
        print(f"Reader thread started: {threading.current_thread().name}")
        global RUNNING
        while RUNNING:
            try:
                raw = self.ser.readline()
            except Exception:
                continue
            if not raw:
                continue
            try:
                line = raw.decode('utf-8', errors='replace').rstrip('\r\n')
            except Exception:
                line = raw.decode('latin-1', errors='replace').rstrip('\r\n')
            if not line:
                continue
            with self.lock:
                if self.mode == "PASSTHROUGH":
                    # detect GRAPH command
                    if line.startswith("GRAPH"):
                        self.mode = "GRAPH"
                        self.buffer = []
                        self.header = None
                        print("\n--- GRAPH MODE STARTED --- (buffering CSV lines, will plot on 'END')\n")
                    else:
                        print(line)
                elif self.mode == "GRAPH":
                    # if header not set, first non-empty line is header
                    if self.header is None:
                        if line.strip() == "":
                            continue
                        self.header = line
                        # store header but do not print
                    else:
                        # if line contains END, finalize
                        if "END" in line:
                            # process buffer in separate thread to avoid blocking reader
                            buf_copy = list(self.buffer)
                            header_copy = self.header
                            self.mode = "PASSTHROUGH"
                            print("\n--- GRAPH MODE END: plotting ---\n")
                            self._process_and_plot(header_copy, buf_copy)
                        else:
                            # append CSV data line
                            self.buffer.append(line)
                else:
                    # fallback
                    print(line)


    def _process_and_plot(self, header_line, data_lines):
        # Build CSV text
        print("Processing CSV data...")
        try:
            csv_text = "\n".join([header_line] + data_lines)
            f = io.StringIO(csv_text)
            reader = csv.reader(f)
            rows = list(reader)
            if not rows or len(rows) < 2:
                print("No data to plot.")
                return
            header = rows[0]
            data_rows = rows[1:]
            # Convert to numeric array where possible, ignore rows with parsing errors
            numeric = []
            for r in data_rows:
                # strip possible empty columns to length of header
                if len(r) < len(header):
                    # pad with empty strings
                    r = r + [''] * (len(header) - len(r))
                try:
                    nums = [float(x) for x in r[:len(header)]]
                    numeric.append(nums)
                except Exception:
                    # skip non-numeric rows
                    continue
            if not numeric:
                print("No numeric data rows found to plot.")
                return
            arr = np.array(numeric)  # shape (n, m)
            m = arr.shape[1]

            # Create a new figure for each plot (off-screen via Agg) using configured image size
            figsize = (PLOT_WIDTH_PX / PLOT_DPI, PLOT_HEIGHT_PX / PLOT_DPI)
            fig = plt.figure(figsize=figsize, dpi=PLOT_DPI)
            ax = fig.add_subplot(111)

            if m == 1:
                x = np.arange(arr.shape[0])
                ax.plot(x, arr[:,0], label=header[0] if header else "value")
                ax.set_xlabel("index")
                ax.legend()
                ax.set_title("Arduino CSV Plot")
            else:
                # use first column as x
                x = (arr[:,0]-arr[0,0])*(0.001)  # offset x to start at 0
                # compute average sample frequency and display in title
                total_time = (x[-1] - x[0]) / 1000.0  # in ms
                avg_freq = arr.shape[0] / total_time if total_time > 0 else 0
                ax.set_title(f"Arduino CSV Plot (Avg. Freq: {avg_freq:.2f} Hz)")
                for i in range(1, m):
                    lbl = header[i] if i < len(header) else f"col{i}"
                    ax.plot(x, arr[:, i], label=lbl, linewidth=1)
                ax.set_xlabel(header[0] if header else "x")
                ax.legend()
            ax.grid(True)
            fig.tight_layout()

            # Show plot interactively
            plt.show()

            # # Save figure to file (unique by timestamp)
            # filename = f"arduino_plot.png"
            # try:
            #     fig.savefig(filename, dpi=PLOT_DPI)
            #     print(f"Saved plot to {filename}")
            # except Exception as e:
            #     print(f"Failed to save plot to file: {e}")
            # finally:
            #     plt.close(fig)

            print("\n--- PLOTTING DONE. Returning to PASSTHROUGH ---\n")
        except Exception as e:
            print(f"Error while processing/plotting data: {e}")


def stdin_sender(ser):
    # send lines typed by user to serial
    try:
        while RUNNING:
            try:
                to_send = input()
            except EOFError:
                break
            if to_send is None:
                break
            # send with newline
            try:
                ser.write((to_send + "\n").encode('utf-8'))
            except Exception as e:
                print(f"Failed to send to serial: {e}")
                break
    except KeyboardInterrupt:
        pass

def signal_handler(sig, frame):
    global RUNNING
    RUNNING = False
    print("\nExiting...")

# def main():
#     signal.signal(signal.SIGINT, signal_handler)
#     port = choose_port()
#     baud_input = input("Baud rate (default 115200): ").strip()
#     baud = int(baud_input) if baud_input.isdigit() else 115200
#     ser = open_serial(port, baud)
#     handler = SerialHandler(ser)
#     handler.start_reader()
#     print("Type to send lines to the device. Ctrl-C to exit.")
#     try:
#         stdin_sender(ser)
#     finally:
#         ser.close()
#         print("Serial closed.")

def main():
    signal.signal(signal.SIGINT, signal_handler)
    port = choose_port()
    baud_input = input("Baud rate (default 115200): ").strip()
    baud = int(baud_input) if baud_input.isdigit() else 115200
    ser = open_serial(port, baud)
    handler = SerialHandler(ser)

    # Start writer (stdin sender) in a background thread, run reader on the main thread
    writer_thread = threading.Thread(target=stdin_sender, args=(ser,), daemon=True)
    writer_thread.start()
    print("Type to send lines to the device. Ctrl-C to exit. (stdin handled in background thread)")

    try:
        # Run reader loop on the main thread so serial reading uses the main thread
        handler._reader_loop()
    finally:
        ser.close()
        print("Serial closed.")


if __name__ == "__main__":
    main()