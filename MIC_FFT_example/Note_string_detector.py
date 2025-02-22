import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation 
import serial
import struct
from threading import Thread
from collections import deque

# uart comport
uart_mcu = serial.Serial('COM3', 115200, parity=serial.PARITY_NONE, 
    stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

# number of frequencies
N = 32

# defining plot parameters
fig = plt.figure(figsize=(12,6))
ax1 = plt.subplot(111)
x = np.linspace(0, 14000, N)  # x-axis in Hz
fft_values = np.zeros(N)
fmt = "%df"% (N)

# Deque to store last 5 maximum frequency points
max_freq_points = deque(maxlen=5)

# creating thread to read the uart
def read_uart():
    global fft_values
    print("running thread")
    while True:
        data_uart = uart_mcu.read_until(b'\xf9\xf8xN')
        if data_uart.__len__() == N * 4 + 4:
            fft_values = struct.unpack(fmt, np.flip(data_uart[0:(N * 4)]))

# animation function to update the plot
def plot_animation(i):
    global fft_values, max_freq_points
    ax1.clear()
    
    # Find frequency of maximum value
    max_index = np.argmax(fft_values)
    max_freq = x[max_index]
    max_freq_points.append(max_freq)
    freq_avg = sum(max_freq_points) / len(max_freq_points)
    
    # Plot setup
    ax1.set_xlabel('Frequency (Hz)')
    ax1.set_ylabel('Magnitude')
    ax1.set_ylim(0, 10000)
    ax1.grid(True)
    
    # Plot line with markers
    line = ax1.plot(x, fft_values, 'b-o', markersize=4, 
                    label=f'FFT (Avg Max Freq: {freq_avg:.1f} Hz)')
    
    # Highlight the current maximum frequency point
    ax1.plot(max_freq, fft_values[max_index], 'ro', markersize=8)
    
    # Add legend
    ax1.legend(loc='upper right')

ani = FuncAnimation(fig, plot_animation, frames=100, interval=10, blit=False)

if __name__ == "__main__":
    t1 = Thread(target=read_uart, daemon=True)
    t1.start()
    plt.show()