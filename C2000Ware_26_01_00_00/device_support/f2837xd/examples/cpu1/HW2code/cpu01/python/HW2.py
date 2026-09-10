"""
ME461 -- Intro Python UI that talks to a TI RedBoard microcontroller
===================================================================

WHAT THIS PROGRAM DOES
----------------------
This is a small desktop application (a "GUI" -- graphical user interface)
built with Python's built-in ``tkinter`` library.  It does two things at once:

  1. PYTHON -> REDBOARD:  When you click a button in the window, Python sends
     the current state of 16 LEDs to the RedBoard over a USB serial port.
     All 16 on/off values are packed into a single 16-bit number and sent as
     text, e.g.  "5\\r\\n"  (5 in binary is 0000000000000101, so LED 1 and
     LED 3 are ON, everything else OFF).

  2. REDBOARD -> PYTHON:  Every 50 milliseconds Python checks the serial port
     for a line of text coming back from the RedBoard.  That line is a single
     number whose lowest 4 bits are the states of 4 push buttons.  Python
     unpacks those bits and recolors on-screen indicator circles.

HOW TO READ THIS FILE IF YOU ARE NEW TO PYTHON
---------------------------------------------
Jump to the very bottom first (the ``if __name__ == "__main__":`` block) --
that is where the program actually starts.  Then come back up to the
``LEDControlGUI`` class and read ``__init__`` from top to bottom, because that
method runs once at startup and builds the entire window.

Comments that start with "PYTHON:" point out a general language feature that is
worth learning and reusing elsewhere.  Other comments explain what this
specific program is doing.

WHAT YOU NEED INSTALLED
-----------------------
* Python 3 (``tkinter`` comes with it).
* pyserial:  run  ``pip install pyserial``  once in a terminal.
"""

# PYTHON: `import` brings in code written by other people so we can use it.
# `import X as Y` gives the module a short nickname (`Y`) for the rest of this
# file, so we can type `tk.Button` instead of `tkinter.Button`.
import tkinter as tk

# PYTHON: `from MODULE import NAME` imports just one piece of a module, so we
# can write `messagebox` instead of `tkinter.messagebox`.
from tkinter import messagebox

# `serial` (the pyserial package) is NOT built into Python -- install it with
# `pip install pyserial`.  It lets Python read and write a serial ("COM") port,
# which is how the RedBoard appears to the computer when plugged in over USB.
import serial


# PYTHON: names written in ALL_CAPS at the top of a file are a convention for
# "constants" -- values you set once here and never change while the program
# runs.  Keeping them together makes them easy to find and edit.
#
# On Windows a port looks like "COM20".  On macOS/Linux it looks like
# "/dev/tty.usbserial-XXXX" or "/dev/ttyUSB0".  Use the Arduino IDE's port
# menu or your OS device manager to find the right name, then edit this line.
SERIAL_PORT = "COM20"

# Baud rate = bits per second on the serial wire.  This number MUST match the
# value the RedBoard firmware uses (e.g. `Serial.begin(115200);` in the C code),
# or the two sides will exchange garbage.
BAUD_RATE = 115200


# PYTHON: `class` defines a new kind of object -- a bundle of data
# ("attributes") plus functions that work on that data ("methods").  We create
# exactly one object of this class at the bottom of the file.  By convention,
# class names use CapWords.
class LEDControlGUI:
    # PYTHON: `__init__` is the "constructor".  It runs automatically once, the
    # moment you create the object, and is where you set up the object's
    # starting state.  `self` is the object being built; EVERY method receives
    # it as the first parameter.  Anything you want to keep after the method
    # returns must be stored on `self` (e.g. `self.led1`), not in a plain local
    # variable.  `root` is the main window; it is handed in from the code at
    # the bottom of the file.
    def __init__(self, root):
        self.root = root
        self.root.title("RedBoard LED Control")  # text shown in the title bar

        # ---------------------------------------------------------
        # LED states sent FROM Python TO RedBoard.
        # Each value is 0 (off) or 1 (on).  There are 16 LEDs.
        #
        # These are written out one per line so the mapping to hardware stays
        # obvious.  Once you are comfortable with lists, you could replace all
        # 16 lines with:   self.leds = [0] * 16
        # and then use self.leds[0] ... self.leds[15].
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
        # Push-button states received FROM RedBoard TO Python.
        # Same idea: 0 = not pressed, 1 = pressed.
        # ---------------------------------------------------------
        self.button1 = 0
        self.button2 = 0
        self.button3 = 0
        self.button4 = 0

        # PYTHON: `None` is Python's built-in "nothing here yet" value.  We
        # create the attribute now so it always exists, then try to replace it
        # with a real serial connection just below.  Checking `is None` later
        # tells us whether the connection succeeded.
        self.ser = None

        # ---------------------------------------------------------
        # Connect to the RedBoard.
        #
        # PYTHON: `try` / `except` is error handling.  Python runs the code in
        # the `try` block; if an error ("exception") of the listed type occurs,
        # it jumps to the `except` block instead of crashing the program.
        # Opening a serial port often fails (wrong port name, board unplugged,
        # port already in use), so we protect it here.
        # ---------------------------------------------------------
        try:
            self.ser = serial.Serial(
                SERIAL_PORT,
                BAUD_RATE,
                timeout=0  # timeout=0 -> read calls return immediately ("non-blocking")
            )

            # PYTHON: an f-string  f"...{expr}..."  builds a string with the
            # values of variables/expressions dropped in where the {curly
            # braces} are.
            connection_text = (
                f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud"
            )

        # PYTHON: `except SomeError as e` catches that error and stores the
        # error object in the variable `e` so we can display its details.
        except serial.SerialException as e:
            connection_text = f"Not connected to {SERIAL_PORT}"

            # Pop up an error dialog box.  Note the program does NOT stop -- the
            # window still opens, it just cannot send or receive anything.
            # In an f-string, \n is a newline character.
            messagebox.showerror(
                "Serial Connection Error",
                f"Could not open {SERIAL_PORT}.\n\n"
                f"Change SERIAL_PORT at the top of the file.\n\n{e}"
            )

        # ---------------------------------------------------------
        # Connection status label
        #
        # PYTHON / TKINTER: a "widget" is one on-screen element (a label, a
        # button, a drawing canvas...).  You create it with
        #     tk.SomeWidget(parent, option=value, option=value, ...)
        # The first argument is the parent it lives inside (here, `root`).
        #
        # Creating a widget does NOT make it appear.  A "geometry manager" must
        # place it.  `.pack()` is the simplest one: it stacks widgets top to
        # bottom in the order you pack them.  `padx` / `pady` add empty space
        # (in pixels) around the widget; `pady=(15, 10)` means 15 above and 10
        # below.
        #
        # We store this label on `self` because we might want to change its
        # text later.  (Static labels below are NOT stored -- see the note there.)
        # ---------------------------------------------------------
        self.status_label = tk.Label(
            root,
            text=connection_text
        )
        self.status_label.pack(padx=20, pady=(15, 10))

        # =========================================================
        # PYTHON -> REDBOARD  (the "send" half of the UI)
        # =========================================================

        # A plain heading label.  Here we do NOT keep a reference to it: we
        # create it, immediately call `.pack()` on it, and then forget it,
        # because we never need to touch it again.  `font` is a tuple of
        # (family, size, style).
        tk.Label(
            root,
            text="LED Control",
            font=("Arial", 14, "bold")
        ).pack(pady=(10, 5))

        # LED 1 button.
        #
        # PYTHON: `command=self.toggle_led1` passes the FUNCTION ITSELF, with no
        # parentheses.  Writing `self.toggle_led1` means "here is the function
        # to call later, when the button is clicked".  Writing
        # `self.toggle_led1()` would call it right now (wrong!) and hand tkinter
        # the return value instead.
        #
        # `width=20` is width in text characters, not pixels (that is how
        # Button sizing works).
        self.led1_button = tk.Button(
            root,
            text="LED 1: OFF",
            width=20,
            command=self.toggle_led1
        )
        self.led1_button.pack(padx=20, pady=5)

        # LED 2 button.  Only LEDs 1 and 2 have buttons wired up in this starter
        # code.  To control more LEDs, copy this pattern: add a toggle_ledN
        # method and another tk.Button here.
        self.led2_button = tk.Button(
            root,
            text="LED 2: OFF",
            width=20,
            command=self.toggle_led2
        )
        self.led2_button.pack(padx=20, pady=5)

        # A label that shows the last number we transmitted.  Kept on `self`
        # because `send_led_values` updates its text every time we send.
        self.last_sent_label = tk.Label(
            root,
            text="Last sent: 0 0"
        )
        self.last_sent_label.pack(padx=20, pady=(10, 15))

        # =========================================================
        # REDBOARD -> PYTHON  (the "receive" half of the UI)
        # =========================================================

        tk.Label(
            root,
            text="Push Button States",
            font=("Arial", 14, "bold")
        ).pack(pady=(10, 5))

        # A Canvas is a blank rectangle you can draw shapes and text on.
        # Width and height here ARE in pixels.
        self.button_canvas = tk.Canvas(
            root,
            width=300,
            height=100
        )
        self.button_canvas.pack(padx=20, pady=10)

        # Push Button 1 indicator.
        #
        # `create_oval(x0, y0, x1, y1, ...)` draws an ellipse inside the box
        # with top-left corner (x0, y0) and bottom-right corner (x1, y1).
        # On a Canvas the origin (0, 0) is the TOP-LEFT and y increases
        # DOWNWARD.  `create_oval` returns an integer ID; we save it so we can
        # recolor this exact circle later with `itemconfig`.
        self.button1_indicator = self.button_canvas.create_oval(
            50,
            20,
            90,
            60,
            fill="gray"
        )

        # A text label drawn on the canvas, centered at (70, 80).  We do not
        # need to change it, so we do not keep its ID.
        self.button_canvas.create_text(
            70,
            80,
            text="Button 1"
        )

        # Push Button 2 indicator (same idea, shifted to the right).
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

        # Label showing the raw button values we last received.
        self.received_label = tk.Label(
            root,
            text="Received: -- --"
        )
        self.received_label.pack(padx=20, pady=(0, 15))

        # PYTHON / TKINTER: `protocol("WM_DELETE_WINDOW", func)` overrides what
        # happens when the user clicks the window's [X] close box.  Instead of
        # tkinter closing the window directly, it calls our `close` method so we
        # can shut the serial port down cleanly first.
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # Start checking serial data.
        # self.read_serial()   # <- left as a hint; update_serial() below calls it for us
        # Start sending and receiving data every 50 ms
        self.update_serial()

    # =============================================================
    # Send LED states to RedBoard
    # =============================================================

    # PYTHON: `def name(self, ...):` inside a class defines a METHOD -- a
    # function attached to the object.  You call it as `self.toggle_led1()`
    # from other methods, or tkinter calls it for us when LED 1's button is
    # clicked (because we passed it as `command=` above).
    def toggle_led1(self):
        # PYTHON: a neat trick to flip a 0/1 value:
        #   1 - 0 == 1   and   1 - 1 == 0
        self.led1 = 1 - self.led1

        # Update the button's caption to match the new state.
        # PYTHON: `A if CONDITION else B` is a "conditional expression": it
        # evaluates to A when CONDITION is true, otherwise B.  Here CONDITION is
        # `self.led1`, and any non-zero number counts as true.
        self.led1_button.config(
            text=f"LED 1: {'ON' if self.led1 else 'OFF'}"
        )

        # Push the updated 16-LED state out over serial.
        self.send_led_values()

    def toggle_led2(self):
        self.led2 = 1 - self.led2

        self.led2_button.config(
            text=f"LED 2: {'ON' if self.led2 else 'OFF'}"
        )

        self.send_led_values()

    def send_led_values(self):
        # PYTHON: a "guard clause" -- bail out early if there is nothing to do.
        # If the serial port never opened (`self.ser is None`) or has since
        # closed, `return` leaves the method immediately and the rest is
        # skipped.  `not X` flips a true/false value.
        if self.ser is None or not self.ser.is_open:
            return

        # Pack the 16 individual LED variables into ONE 16-bit integer, one bit
        # per LED, then send that number as text followed by \r\n.
        #
        # PYTHON bit operators used below:
        #   x & 1    keep only the lowest bit of x (forces it to 0 or 1)
        #   x << n   "left shift": move x's bits n places up  (x << 3 == x * 8)
        #   a |= b   "or-assign": same as  a = a | b ; sets the bits that are on in b
        #
        # So `(self.led3 & 1) << 2` puts LED 3's state into bit position 2,
        # and `|=` merges it into LED16bits without disturbing the other bits.
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

        # Build the exact text line to transmit.  \r\n (carriage return +
        # line feed) is the line ending the RedBoard firmware watches for.
        message = f"{LED16bits}\r\n"

        # PYTHON: a serial port sends raw BYTES, not Python `str` text.
        # `.encode("ascii")` converts the string "5\r\n" into the bytes
        # b"5\r\n".  (The reverse is `.decode(...)`, used in read_serial.)
        self.ser.write(
            message.encode("ascii")
        )

        # Reflect what we just sent in the GUI.
        self.last_sent_label.config(
            text=f"Last sent: {LED16bits}"
        )

        # PYTHON: `print(...)` writes to the terminal/console you launched the
        # program from -- very handy for debugging.  `.strip()` here removes the
        # trailing \r\n so the console line stays tidy.
        print(f"Sent: {message.strip()}")

    # =============================================================
    # Read push-button states from RedBoard
    # =============================================================

    def read_serial(self):

        # Only try to read if we actually have an open connection.
        if self.ser is not None and self.ser.is_open:

            try:
                # PYTHON: `while CONDITION:` repeats the indented block as long
                # as CONDITION stays true.  `self.ser.in_waiting` is how many
                # bytes have arrived and are sitting in the buffer unread.  We
                # loop until we have drained them all, so the GUI never falls
                # behind the RedBoard.
                while self.ser.in_waiting > 0:

                    # `readline()` reads bytes up to and including the next \n
                    # and returns them as a `bytes` object.
                    # `.decode("ascii", errors="ignore")` turns those bytes
                    # back into a `str`, silently dropping any byte that is not
                    # valid ASCII.  `.strip()` removes surrounding whitespace,
                    # including the trailing \r\n.
                    line = self.ser.readline().decode(
                        "ascii",
                        errors="ignore"
                    ).strip()

                    # PYTHON: an empty string is "falsy", so `if line:` is true
                    # only when we actually got some text.  This skips blank
                    # lines.
                    if line:
                        print(f"Received: {line}")

                        # PYTHON: `str.split()` with no arguments splits on any
                        # run of whitespace and returns a LIST of the pieces.
                        # "6" -> ["6"];  "6 extra" -> ["6", "extra"].
                        values = line.split()

                        # We expect exactly:
                        # button4bit number
                        #
                        # Example:
                        # 6   Middle two buttons pressed

                        # PYTHON: `len(x)` is the number of items in a list (or
                        # characters in a string).  Only proceed if we got
                        # exactly one token.
                        if len(values) == 1:

                            # Inner try/except: `int("abc")` raises ValueError.
                            # If a corrupted line sneaks through, we ignore it
                            # instead of crashing.
                            try:

                                # `int(values[0])` converts the text (list item
                                # 0) into a whole number.
                                # `& 0x000F` keeps only the low 4 bits:
                                # 0x000F is hexadecimal for 15 == binary 1111.
                                # only use last 4 bits of received value.
                                button4bits = int(values[0]) & 0x000F

                                # Pull out one bit at a time:
                                #   x >> n   "right shift": move bit n down to
                                #            position 0
                                #   & 1      then keep just that lowest bit
                                self.button1  = (button4bits >> 0)  & 1
                                self.button2  = (button4bits >> 1)  & 1
                                # self.button3 = ?   <- exercise: add bits 2 and 3
                                # self.button4 = ?

                                # Redraw the on-screen indicators.
                                self.update_button_indicators()

                            except ValueError:
                                # Ignore malformed serial lines
                                pass

            except serial.SerialException:
                # e.g. the board was unplugged mid-read.  Ignore and try again
                # on the next tick.
                pass


    def update_serial(self):

        # Do one round of reading whatever the RedBoard has sent.
        self.read_serial()
        # Could maybe put send_led_values here if you want to send a new value every 50ms.  but there may be a better way for your task
        # self.send_led_values()

        # PYTHON / TKINTER: `root.after(ms, func)` tells tkinter "call `func`
        # once, about `ms` milliseconds from now".  Because `update_serial`
        # schedules ITSELF again here, it keeps running every ~50 ms for as long
        # as the program is open -- a repeating timer.
        #
        # This is the correct way to do periodic work in a GUI.  Do NOT use
        # `time.sleep()` or a `while True:` loop in a tkinter program -- either
        # one freezes the whole window because tkinter never gets a turn to
        # redraw or respond to clicks.
        self.root.after(
            50,
            self.update_serial
        )
    # =============================================================
    # Update graphical push-button LEDs
    # =============================================================

    def update_button_indicators(self):

        # PYTHON / TKINTER: `canvas.itemconfig(item_id, option=value)` changes
        # an option of a shape already drawn on the canvas.  We saved
        # `self.button1_indicator` earlier -- it is the ID of that oval.

        # Button 1: green when pressed, gray when not.
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

        # Also show the raw numbers as text.
        self.received_label.config(
            text=f"Received: {self.button1} {self.button2}"
        )

    # =============================================================
    # Close program
    # =============================================================

    def close(self):

        # Release the serial port so other programs (and the next run of this
        # one) can use it.  Always close hardware resources you opened.
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

        # Tear down the window.  This ends `mainloop()` at the bottom of the
        # file, so the program exits.
        self.root.destroy()


# =============================================================
# Main
# =============================================================

# PYTHON: every file has a built-in variable `__name__`.  When you RUN this
# file directly (`python LEDdan.py`) Python sets `__name__` to "__main__", so
# this block executes.  When another file does `import LEDdan` instead,
# `__name__` is "LEDdan" and this block is skipped.  This is the standard place
# to put "start the program" code.
if __name__ == "__main__":

    # Create the main application window.
    root = tk.Tk()

    # Build our GUI inside that window.  This line runs LEDControlGUI.__init__,
    # which creates every widget and starts the 50 ms serial timer.
    app = LEDControlGUI(root)

    # Hand control to tkinter.  `mainloop()` runs an endless loop that waits for
    # events -- button clicks, `after` timers, the window closing -- and calls
    # the matching functions.  Execution stays on this line until the window is
    # destroyed; nothing after it runs until then.
    root.mainloop()
