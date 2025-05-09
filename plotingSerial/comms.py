import threading
import queue
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Create a thread-safe queue to share data between threads
data_queue = queue.Queue()

# Serial reading function to run in a separate thread
def read_serial(port='COM19', baudrate=9600):
    ser = serial.Serial(port, baudrate)
    while True:
        try:
            line = ser.read() #.decode('utf-8')
            value = int.from_bytes(line) #, byteorder='big', signed=True)  # adjust this if your data format is different
            data_queue.put(value)
        except Exception as e:
            print("Error:", e)

# Matplotlib plotting setup
x_data = [0]
y_data = [0]

def animate(i):
    # Pull data from the queue
    while not data_queue.empty():
        value = data_queue.get()
        print("[1] %d" % (value))
        x_data.append(x_data[-1] + 1)  # Simple x: count of values
        y_data.append(value)

        # Optional: limit data length for performance
        if len(x_data) > 100:
            x_data.pop(0)
            y_data.pop(0)

    ax.clear()
    ax.plot(x_data, y_data)
    ax.set_title("Real-time Serial Data")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Value")

# Start serial reading thread
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

# Create plot
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, animate, interval=100)  # update every 100ms
plt.show()

