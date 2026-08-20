import socket
import struct
import threading
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

HOST = '192.168.1.58'
PORT = 10001

class FloatClientApp:
    def __init__(self, master):
        self.master = master
        master.title("TCP Float Client GUI")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((HOST, PORT))
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            master.destroy()
            return

        # Input Fields
        self.send_entries = []
        for i in range(8):
            ttk.Label(master, text=f"SendValue{i+1}").grid(row=i, column=0)
            entry = ttk.Entry(master, width=10)
            entry.insert(0, "0")
            entry.grid(row=i, column=1)
            self.send_entries.append(entry)

        self.send_button = ttk.Button(master, text="Send", command=self.send_floats)
        self.send_button.grid(row=8, column=0, columnspan=2, pady=5)

        # Received Values
        self.recv_vars = []
        for i in range(8):
            ttk.Label(master, text=f"RecvValue{i+1}").grid(row=i, column=3)
            var = tk.StringVar(value="0")
            entry = ttk.Entry(master, textvariable=var, width=10, state='readonly')
            entry.grid(row=i, column=4)
            self.recv_vars.append(var)

        # Plot Area
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Received Point History")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.scatter_data = []
        self.canvas = FigureCanvasTkAgg(self.fig, master)
        self.canvas.get_tk_widget().grid(row=0, column=5, rowspan=9, padx=10, pady=10)

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        self.fig.canvas.mpl_connect('key_press_event', self.press)
        self.master.bind('<Key>', self.press)
        self.master.protocol('WM_DELETE_WINDOW', self.on_close)

        # Threaded Receiver
        self.recv_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.recv_thread.start()

    def send_floats(self):
        try:
            values = [float(entry.get()) for entry in self.send_entries]
            num = " ".join(f"{v:.3f}" for v in values) 
            data_str = b'\xFD' + num.encode('ascii') + b'\xFF'
            self.sock.sendall(data_str)

        except Exception as e:
            messagebox.showerror("Send Error", str(e))

    def receive_loop(self):
        try:
            buffer = b''
            while True:
                buffer += self.sock.recv(1024)
                while b'\r\n' in buffer:
                    line, buffer = buffer.split(b'\r\n', 1)
                    try:
                        values = [float(v) for v in line.decode().split()]
                        if len(values) == 8:
                            self.master.after(0, self.update_display, values)
                    except:
                        continue
        except Exception as e:
            print(f"Receive loop error: {e}")

    def update_display(self, values):
        for i, val in enumerate(values):
            self.recv_vars[i].set(f"{val:.2f}")
        self.scatter_data.append((values[0], values[1]))  # X, Y only
        if len(self.scatter_data) > 100:
            self.scatter_data.pop(0)

    def update_plot(self, frame):
        self.ax.clear()
        self.ax.set_title("Received Point History")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        if self.scatter_data:
            xs, ys = zip(*self.scatter_data)
            self.ax.plot(xs, ys, linestyle='', marker='o')

        self.canvas.draw()

    def on_close(self):
        try:
            self.sock.close()
        except Exception:
            pass
        plt.close('all')
        self.master.destroy()

    def press(self, event):
        key = getattr(event, 'key', None)
        if key is None:
            key = getattr(event, 'char', None)
        if key == 'q':
            self.on_close()

if __name__ == "__main__":
    root = tk.Tk()
    app = FloatClientApp(root)
    root.mainloop()