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
        try:
            self.uart = serial.Serial(port, baud,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS,
                                    timeout=1)
        except serial.SerialException as e:
            print(f"Failed to open serial port {port}: {e}")
            raise
        
        # FFT parameters
        self.sample_rate = 16000  # Match with I2S config
        self.N = 64  # Number of FFT bins we're receiving (0-1000Hz)
        self.freq_per_bin = self.sample_rate / 1024  # ~15.625 Hz per bin
        self.freq_range = np.linspace(0, self.N * self.freq_per_bin, self.N)  # Frequency axis up to 1000Hz
        
        # Plot setup
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.fft_values = np.zeros(self.N)
        self.max_freq_points = deque(maxlen=10)  # Store last 10 peak frequencies
        
        # Data reading
        self.buffer = bytearray()
        self.START_MARKER = 0xAA
        self.END_MARKER = 0x55
        self.expected_data_size = self.N * 4  # Size in bytes of float32 array (64 * 4 = 256 bytes)
        
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
                        byte = self.uart.read(1)[0]
                        if byte == self.START_MARKER:
                            self.buffer = bytearray()
                            state = READING_DATA
                    
                    elif state == READING_DATA:
                        remaining = self.expected_data_size - len(self.buffer)
                        if remaining > 0:
                            data = self.uart.read(min(remaining, self.uart.in_waiting))
                            self.buffer.extend(data)
                            
                            if len(self.buffer) == self.expected_data_size:
                                end_byte = self.uart.read(1)
                                if end_byte and end_byte[0] == self.END_MARKER:
                                    try:
                                        fmt = f"<{self.N}f"
                                        self.fft_values = struct.unpack(fmt, self.buffer)
                                    except struct.error as e:
                                        print(f"Error unpacking data: {e}")
                                state = SEARCHING_START
                                
            except Exception as e:
                print(f"Error reading UART: {e}")
                state = SEARCHING_START

    def _update_plot(self, frame):
        self.ax.clear()
        
        # Scale FFT values using log10 for better visualization
        scaled_fft = np.log10(np.array(self.fft_values) + 1)  # Add 1 to avoid log(0)
        
        # Find peak frequency
        max_idx = np.argmax(scaled_fft)
        max_freq = self.freq_range[max_idx]
        self.max_freq_points.append(max_freq)
        freq_avg = np.mean(self.max_freq_points)
        
        # Plot setup
        self.ax.set_title('Real-time FFT Analysis (0-1000 Hz)', color='white', pad=20)
        self.ax.set_xlabel('Frequency (Hz)', color='white')
        self.ax.set_ylabel('Magnitude (log scale)', color='white')
        self.ax.grid(True, alpha=0.3)
        
        # Set axis limits
        self.ax.set_xlim(0, 1000)  # Only show up to 1000 Hz
        self.ax.set_ylim(0, max(1, max(scaled_fft) * 1.2))
        
        # Plot FFT data
        self.ax.plot(self.freq_range, scaled_fft, 'c-', alpha=0.7, label='FFT')
        
        # Highlight peak
        self.ax.plot(max_freq, scaled_fft[max_idx], 'ro', markersize=8)
        self.ax.annotate(f'Peak: {max_freq:.1f} Hz\nMag: {self.fft_values[max_idx]:.1f}',
                        xy=(max_freq, scaled_fft[max_idx]),
                        xytext=(max_freq + 100, scaled_fft[max_idx] * 0.9),
                        color='white',
                        arrowprops=dict(facecolor='white', shrink=0.05))
        
        # Show average peak frequency
        self.ax.axvline(x=freq_avg, color='r', linestyle='--', alpha=0.5,
                       label=f'Avg Peak: {freq_avg:.1f} Hz')
        
        self.ax.legend(loc='upper right')
        
        # Add frequency markers every 100 Hz
        for freq in range(0, 1001, 100):
            self.ax.axvline(x=freq, color='gray', linestyle=':', alpha=0.2)
            if freq % 200 == 0:  # Label every 200 Hz for clarity
                self.ax.text(freq, 0, f'{freq}Hz', alpha=0.5, color='white')

    def run(self):
        ani = FuncAnimation(self.fig, self._update_plot, 
                          cache_frame_data=False,
                          interval=50)  # Update every 50ms
        plt.show()
        
    def close(self):
        self.running = False
        if hasattr(self, 'uart') and self.uart.is_open:
            self.uart.close()
        plt.close()

if __name__ == "__main__":
    try:
        plotter = FFTPlotter()
        plotter.run()
    except KeyboardInterrupt:
        print("\nClosing application...")
        plotter.close()
    except Exception as e:
        print(f"Error: {e}")
        try:
            plotter.close()
        except:
            pass