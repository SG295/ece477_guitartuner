import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import serial
import struct
from threading import Thread
from collections import deque

class FFTPlotter:
    def __init__(self, port='COM3', baud=115200):
        # UART configuration
        self.uart = serial.Serial(port, baud,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS,
                                timeout=1)
        
        # FFT parameters
        self.N = 64  # WAV_WRITE_SAMPLE_COUNT/4 from firmware
        self.sample_rate = 32000  # 32kHz sampling
        self.freq_range = np.linspace(0, self.sample_rate/2, self.N//2)  # Frequency axis
        
        # Plot setup
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.fft_values = np.zeros(self.N//2)
        self.max_freq_points = deque(maxlen=10)  # Store last 10 peak frequencies
        
        # Data reading
        self.buffer = bytearray()
        self.START_MARKER = 0xAA
        self.END_MARKER = 0x55
        self.expected_data_size = (self.N//2) * 4  # Size in bytes of float32 array
        
        # Start reading thread
        self.running = True
        self.thread = Thread(target=self._read_uart, daemon=True)
        self.thread.start()

    def _read_uart(self):
        SEARCHING_START = 0
        READING_DATA = 1
        state = SEARCHING_START
        
        while self.running:
            try:
                if self.uart.in_waiting > 0:
                    if state == SEARCHING_START:
                        # Look for start marker
                        byte = self.uart.read(1)[0]
                        if byte == self.START_MARKER:
                            self.buffer = bytearray()
                            state = READING_DATA
                    
                    elif state == READING_DATA:
                        # Read expected amount of data
                        remaining = self.expected_data_size - len(self.buffer)
                        if remaining > 0:
                            data = self.uart.read(min(remaining, self.uart.in_waiting))
                            self.buffer.extend(data)
                            
                            if len(self.buffer) == self.expected_data_size:
                                # Check end marker
                                end_byte = self.uart.read(1)
                                if end_byte and end_byte[0] == self.END_MARKER:
                                    # Parse the data
                                    try:
                                        fmt = f"<{self.N//2}f"
                                        self.fft_values = struct.unpack(fmt, self.buffer)
                                    except struct.error:
                                        pass
                                state = SEARCHING_START
                                
            except Exception as e:
                print(f"Error reading UART: {e}")
                state = SEARCHING_START

    def _update_plot(self, frame):
        self.ax.clear()
        
        # Find peak frequency
        max_idx = np.argmax(self.fft_values)
        max_freq = self.freq_range[max_idx]
        self.max_freq_points.append(max_freq)
        freq_avg = np.mean(self.max_freq_points)
        
        # Plot setup
        self.ax.set_title('Real-time FFT Analysis')
        self.ax.set_xlabel('Frequency (Hz)')
        self.ax.set_ylabel('Magnitude')
        self.ax.grid(True)
        
        # Plot FFT data
        self.ax.plot(self.freq_range, self.fft_values, 'b-', alpha=0.7, label='FFT')
        
        # Highlight peak
        self.ax.plot(max_freq, self.fft_values[max_idx], 'ro', markersize=8)
        self.ax.annotate(f'Peak: {max_freq:.1f} Hz',
                        xy=(max_freq, self.fft_values[max_idx]),
                        xytext=(max_freq + 500, self.fft_values[max_idx] * 0.9),
                        arrowprops=dict(facecolor='black', shrink=0.05))
        
        # Show average peak frequency
        self.ax.axvline(x=freq_avg, color='r', linestyle='--', alpha=0.5,
                       label=f'Avg Peak: {freq_avg:.1f} Hz')
        
        self.ax.legend()
        self.ax.set_ylim(0, max(1000, max(self.fft_values) * 1.2))

    def run(self):
        ani = FuncAnimation(self.fig, self._update_plot, 
                          cache_frame_data=False,  # Disable frame caching
                          interval=50)             # Faster updates
        plt.show()
        
    def close(self):
        self.running = False
        self.uart.close()
        plt.close()

if __name__ == "__main__":
    try:
        plotter = FFTPlotter()
        plotter.run()
    except KeyboardInterrupt:
        plotter.close()