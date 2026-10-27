import socket
import struct
import sys
import select
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

HOST = '192.168.1.59'
PORT = 10001

class FloatClientApp:
    def __init__(self, master):
        self.master = master
        self.closing = threading.Event()
        self.received_values = queue.Queue()
        self.last_send_time = None
        master.protocol("WM_DELETE_WINDOW", self.close)
        master.title("TCP Float Client GUI")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((HOST, PORT))
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            self.close()
            return

        # Input Fields
        self.send_entries = []
        for i in range(8):
            label = ttk.Label(master, text=f"SendValue{i+1}")
            label.grid(row=i, column=0)
            entry = ttk.Entry(master, width=10)
            entry.insert(0, "0")
            entry.grid(row=i, column=1)
            self.send_entries.append(entry)

        self.send_button = ttk.Button(master, text="Send", command=self.send_floats)
        self.send_button.grid(row=8, column=0, columnspan=2, pady=5)
        self.keyboard_dialog = None
        self.keyboard_button = ttk.Button(
            master, text="Keyboard Control", command=self.open_keyboard_dialog)
        self.keyboard_button.grid(row=8, column=2, padx=5, pady=5)

        # Received Values
        self.recv_vars = []
        for i in range(8):
            label = ttk.Label(master, text=f"RecvValue{i+1}")
            label.grid(row=i, column=3)
            var = tk.StringVar(value="0")
            entry = ttk.Entry(master, textvariable=var, width=10, state='readonly')
            entry.grid(row=i, column=4)
            self.recv_vars.append(var)

        # Plot Area
        self.fig = Figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Received Point History")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_autoscale_on(False)
        self.scatter_data = []
        self.canvas = FigureCanvasTkAgg(self.fig, master)
        self.canvas.get_tk_widget().grid(row=0, column=5, rowspan=9, padx=10, pady=10)

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50)

        # Threaded Receiver
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()

    def close(self):
        if self.closing.is_set():
            return
        self.closing.set()
        if hasattr(self, 'ani') and self.ani.event_source is not None:
            self.ani.event_source.stop()
        # The receiver checks the closing flag every 0.2 seconds.
        if hasattr(self, 'recv_thread'):
            self.recv_thread.join(timeout=1)
        # Reset the connection immediately to avoid a lingering graceful close
        # on the ESP32. Pending outgoing data may be discarded.
        try:
            if sys.platform == 'win32':
                linger_format = 'HH'
            else:
                linger_format = 'ii'
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                                 struct.pack(linger_format, 1, 0))
        except OSError as e:
            print(f"Could not enable immediate socket close: {e}")
        self.sock.close()
        self.master.destroy()

    def ready_to_send(self):
        # While Keyboard Control is open, allow at most one send every 50 ms.
        if self.keyboard_dialog is not None:
            if self.last_send_time is not None:
                elapsed_seconds = time.monotonic() - self.last_send_time
                if elapsed_seconds < 0.050:
                    return False
        return True

    def send_floats(self):
        if not self.ready_to_send():
            return
        try:
            # Read each entry and build a message with spaces between values.
            message = ""
            for i in range(8):
                entry_text = self.send_entries[i].get()
                value = float(entry_text)
                if i > 0:
                    message = message + " "
                message = message + f"{value:.3f}"

            # The ESP32 expects a start byte, the text, and an end byte.
            message_bytes = message.encode('ascii')
            data_str = b'\xFD' + message_bytes + b'\xFF'
            self.sock.sendall(data_str)
            self.last_send_time = time.monotonic()

        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def open_keyboard_dialog(self):
        if self.keyboard_dialog is not None and self.keyboard_dialog.winfo_exists():
            self.keyboard_dialog.deiconify()
            self.keyboard_dialog.lift()
            self.keyboard_dialog.focus_set()
            return

        dialog = tk.Toplevel(self.master)
        self.keyboard_dialog = dialog
        dialog.title("Keyboard Control")
        dialog.transient(self.master)
        dialog.geometry("360x260")
        instructions_label = ttk.Label(
            dialog,
            text=("Press keys:\n"
                  "Press a for Left\n"
                  "Press f for Right\n"
                  "Press s to Speed Up\n"
                  "Press d to Slow Down\n"
                  "Press x to Stop\n"
                  "Press w to Go Straight\n"
                  "Press q to Quit"),
            justify="left", anchor="nw"
        )
        instructions_label.pack(expand=True, fill="both", padx=20, pady=20)
        dialog.protocol("WM_DELETE_WINDOW", self.close_keyboard_dialog)
        dialog.bind("<Button-1>", self.focus_keyboard_dialog)
        dialog.bind("<KeyPress>", self.handle_control_key)
        dialog.after_idle(dialog.focus_set)

    def focus_keyboard_dialog(self, event):
        self.keyboard_dialog.focus_set()

    def close_keyboard_dialog(self):
        if self.keyboard_dialog is not None:
            self.keyboard_dialog.destroy()
            self.keyboard_dialog = None

    def handle_control_key(self, event):
        key = event.keysym.lower()
        if key == 'q':
            self.close_keyboard_dialog()
            return "break"
        # Ignore fast key repeats before changing any values.
        # This also limits alternating keys, not just a single held key.
        if not self.ready_to_send():
            return "break"
        # Python lists start at 0: entry 0 is SendValue1 (speed),
        # and entry 1 is SendValue2 (steering).
        try:
            if key == 'a':
                # Turn left by changing SendValue2.
                entry_text = self.send_entries[1].get()
                value = float(entry_text)
                value = value + 0.1
                self.send_entries[1].delete(0, tk.END)
                self.send_entries[1].insert(0, f"{value:.12g}")
                self.send_floats()
                return "break"

            if key == 'f':
                # Turn right by changing SendValue2.
                entry_text = self.send_entries[1].get()
                value = float(entry_text)
                value = value - 0.1
                self.send_entries[1].delete(0, tk.END)
                self.send_entries[1].insert(0, f"{value:.12g}")
                self.send_floats()
                return "break"

            if key == 's':
                # Speed up by changing SendValue1.
                entry_text = self.send_entries[0].get()
                value = float(entry_text)
                value = value + 0.1
                self.send_entries[0].delete(0, tk.END)
                self.send_entries[0].insert(0, f"{value:.12g}")
                self.send_floats()
                return "break"

            if key == 'd':
                # Slow down by changing SendValue1.
                entry_text = self.send_entries[0].get()
                value = float(entry_text)
                value = value - 0.1
                self.send_entries[0].delete(0, tk.END)
                self.send_entries[0].insert(0, f"{value:.12g}")
                self.send_floats()
                return "break"

            if key == 'x':
                # Stop: set both speed and steering to zero.
                self.send_entries[0].delete(0, tk.END)
                self.send_entries[0].insert(0, "0.0")
                self.send_entries[1].delete(0, tk.END)
                self.send_entries[1].insert(0, "0.0")
                self.send_floats()
                return "break"

            if key == 'w':
                # Go straight: set speed to 0.5 and steering to zero.
                self.send_entries[0].delete(0, tk.END)
                self.send_entries[0].insert(0, "0.5")
                self.send_entries[1].delete(0, tk.END)
                self.send_entries[1].insert(0, "0.0")
                self.send_floats()
                return "break"

        except ValueError:
            messagebox.showerror(
                "Send Error", "The SendValue being changed must be a number.",
                parent=self.keyboard_dialog)
            return "break"

    def receive_loop(self):
        try:
            buffer = b''
            while not self.closing.is_set():
                readable, _, _ = select.select([self.sock], [], [], 0.2)
                if self.closing.is_set():
                    break
                if not readable:
                    continue
                data = self.sock.recv(1024)
                if not data:
                    break
                buffer += data
                while b'\r\n' in buffer and not self.closing.is_set():
                    line, buffer = buffer.split(b'\r\n', 1)
                    try:
                        line_text = line.decode()
                        value_strings = line_text.split()
                        values = []
                        for value_string in value_strings:
                            value = float(value_string)
                            values.append(value)
                        if len(values) == 8:
                            self.received_values.put(values)
                    except (ValueError, UnicodeDecodeError):
                        continue
        except Exception as e:
            if not self.closing.is_set():
                print(f"Receive loop error: {e}")

    def update_display(self, values):
        for i in range(len(values)):
            value = values[i]
            self.recv_vars[i].set(f"{value:.2f}")
        self.scatter_data.append((values[0], values[1]))  # X, Y only
        if len(self.scatter_data) > 100:
            self.scatter_data.pop(0)

    def update_plot(self, frame):
        if self.closing.is_set():
            return
        while True:
            try:
                values = self.received_values.get_nowait()
            except queue.Empty:
                break
            self.update_display(values)
        self.ax.clear()
        self.ax.set_title("Received Point History")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_autoscale_on(False)
        if self.scatter_data:
            xs = []
            ys = []
            for point in self.scatter_data:
                xs.append(point[0])
                ys.append(point[1])
            self.ax.plot(xs, ys, linestyle='', marker='o')
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = FloatClientApp(root)
    try:
        root.mainloop()
    finally:
        app.close()
