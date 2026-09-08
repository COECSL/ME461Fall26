import tkinter as tk
from tkinter import messagebox
import serial

# Change this to the COM port used by your RedBoard.
SERIAL_PORT = "COM20"
BAUD_RATE = 115200


class LEDControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RedBoard LED Control")

        # ---------------------------------------------------------
        # LED states sent FROM Python TO RedBoard
        # ---------------------------------------------------------
        self.led1 = 0
        self.led2 = 0
        self.led3 = 0
        self.led4 = 0
        self.led5 = 0
        self.led6 = 0
        self.led7 = 0
        self.led8 = 0
        self.led9 = 0
        self.led10 = 0
        self.led11 = 0
        self.led12 = 0
        self.led13 = 0
        self.led14 = 0
        self.led15 = 0
        self.led16 = 0

        # ---------------------------------------------------------
        # Push-button states received FROM RedBoard TO Python
        # ---------------------------------------------------------
        self.button1 = 0
        self.button2 = 0
        self.button3 = 0
        self.button4 = 0

        self.ser = None

        # ---------------------------------------------------------
        # Connect to RedBoard
        # ---------------------------------------------------------
        try:
            self.ser = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=0
            )

            connection_text = (
                f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud"
            )

        except serial.SerialException as e:
            connection_text = f"Not connected to {SERIAL_PORT}"

            messagebox.showerror(
                "Serial Connection Error",
                f"Could not open {SERIAL_PORT}.\n\n"
                f"Change SERIAL_PORT at the top of the file.\n\n{e}"
            )

        # ---------------------------------------------------------
        # Connection status
        # ---------------------------------------------------------
        self.status_label = tk.Label(
            root,
            text=connection_text
        )
        self.status_label.pack(padx=20, pady=(15, 10))

        # =========================================================
        # PYTHON -> REDBOARD
        # =========================================================

        tk.Label(
            root,
            text="LED Control",
            font=("Arial", 14, "bold")
        ).pack(pady=(10, 5))

        # LED 1 button
        self.led1_button = tk.Button(
            root,
            text="LED 1: OFF",
            width=20,
            command=self.toggle_led1
        )
        self.led1_button.pack(padx=20, pady=5)

        # LED 2 button
        self.led2_button = tk.Button(
            root,
            text="LED 2: OFF",
            width=20,
            command=self.toggle_led2
        )
        self.led2_button.pack(padx=20, pady=5)

        # Show what was last sent
        self.last_sent_label = tk.Label(
            root,
            text="Last sent: 0 0"
        )
        self.last_sent_label.pack(padx=20, pady=(10, 15))

        # =========================================================
        # REDBOARD -> PYTHON
        # =========================================================

        tk.Label(
            root,
            text="Push Button States",
            font=("Arial", 14, "bold")
        ).pack(pady=(10, 5))

        # Canvas used to draw the button-state LEDs
        self.button_canvas = tk.Canvas(
            root,
            width=300,
            height=100
        )
        self.button_canvas.pack(padx=20, pady=10)

        # Push Button 1 indicator
        self.button1_indicator = self.button_canvas.create_oval(
            50,
            20,
            90,
            60,
            fill="gray"
        )

        self.button_canvas.create_text(
            70,
            80,
            text="Button 1"
        )

        # Push Button 2 indicator
        self.button2_indicator = self.button_canvas.create_oval(
            210,
            20,
            250,
            60,
            fill="gray"
        )

        self.button_canvas.create_text(
            230,
            80,
            text="Button 2"
        )

        # Show raw received values
        self.received_label = tk.Label(
            root,
            text="Received: -- --"
        )
        self.received_label.pack(padx=20, pady=(0, 15))

        # Close handler
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # Start checking serial data
        # self.read_serial()
        # Start sending and receiving data every 50 ms
        self.update_serial()

    # =============================================================
    # Send LED states to RedBoard
    # =============================================================

    def toggle_led1(self):
        self.led1 = 1 - self.led1

        self.led1_button.config(
            text=f"LED 1: {'ON' if self.led1 else 'OFF'}"
        )

        self.send_led_values()

    def toggle_led2(self):
        self.led2 = 1 - self.led2

        self.led2_button.config(
            text=f"LED 2: {'ON' if self.led2 else 'OFF'}"
        )

        self.send_led_values()

    def send_led_values(self):
        if self.ser is None or not self.ser.is_open:
            return

        # Sends:
        #
        # 16bit number\r\n
        LED16bits = 0
        # << is left shift | is bitwise or
        LED16bits |= (self.led1  & 1) << 0
        LED16bits |= (self.led2  & 1) << 1
        LED16bits |= (self.led3  & 1) << 2
        LED16bits |= (self.led4  & 1) << 3
        LED16bits |= (self.led5  & 1) << 4
        LED16bits |= (self.led6  & 1) << 5
        LED16bits |= (self.led7  & 1) << 6
        LED16bits |= (self.led8  & 1) << 7
        LED16bits |= (self.led9  & 1) << 8
        LED16bits |= (self.led10 & 1) << 9
        LED16bits |= (self.led11 & 1) << 10
        LED16bits |= (self.led12 & 1) << 11
        LED16bits |= (self.led13 & 1) << 12
        LED16bits |= (self.led14 & 1) << 13
        LED16bits |= (self.led15 & 1) << 14
        LED16bits |= (self.led16 & 1) << 15

        message = f"{LED16bits}\r\n"

        self.ser.write(
            message.encode("ascii")
        )

        self.last_sent_label.config(
            text=f"Last sent: {LED16bits}"
        )

        print(f"Sent: {message.strip()}")

    # =============================================================
    # Read push-button states from RedBoard
    # =============================================================

    def read_serial(self):

        if self.ser is not None and self.ser.is_open:

            try:
                # Read all complete lines currently waiting
                while self.ser.in_waiting > 0:

                    line = self.ser.readline().decode(
                        "ascii",
                        errors="ignore"
                    ).strip()

                    if line:
                        print(f"Received: {line}")

                        values = line.split()

                        # We expect exactly:
                        # button4bit number
                        #
                        # Example:
                        # 6   Middle two buttons pressed

                        if len(values) == 1:

                            try:

                                # only use last 4 bits of received value.
                                button4bits = int(values[0]) & 0x000F

                                self.button1  = (button4bits >> 0)  & 1
                                self.button2  = (button4bits >> 1)  & 1
                                # self.button3 = ?
                                # self.button4 = ?

                                self.update_button_indicators()

                            except ValueError:
                                # Ignore malformed serial lines
                                pass

            except serial.SerialException:
                pass


    def update_serial(self):

        self.read_serial()
        # Could maybe put send_led_values here if you want to send a new value every 50ms.  but there may be a better way for your task
        # self.send_led_values()

        # Run this function again in 50 ms
        self.root.after(
            50,
            self.update_serial
        )
    # =============================================================
    # Update graphical push-button LEDs
    # =============================================================

    def update_button_indicators(self):

        # Button 1
        if self.button1 == 1:
            self.button_canvas.itemconfig(
                self.button1_indicator,
                fill="green"
            )
        else:
            self.button_canvas.itemconfig(
                self.button1_indicator,
                fill="gray"
            )

        # Button 2
        if self.button2 == 1:
            self.button_canvas.itemconfig(
                self.button2_indicator,
                fill="green"
            )
        else:
            self.button_canvas.itemconfig(
                self.button2_indicator,
                fill="gray"
            )

        self.received_label.config(
            text=f"Received: {self.button1} {self.button2}"
        )

    # =============================================================
    # Close program
    # =============================================================

    def close(self):

        if self.ser is not None and self.ser.is_open:
            self.ser.close()

        self.root.destroy()


# =============================================================
# Main
# =============================================================

if __name__ == "__main__":

    root = tk.Tk()

    app = LEDControlGUI(root)

    root.mainloop()