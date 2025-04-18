import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.fft import fft, fftfreq
import struct
import time
import threading

# Configuration
PORT = 'COM3'  # Change to your serial port
BAUD_RATE = 9600
BUFFER_SIZE = 64
SAMPLE_RATE = 32000  # 32 kHz
BYTES_PER_SAMPLE = 4  # 4 bytes per sample (32-bit)
FRAME_MARKER = 0xF0   # Frame marker for packet synchronization

# Headers
UNFILTERED_HEADER = 0xAA
FILTERED_HEADER = 0xBB

class AudioVisualizer:
    def __init__(self):
        # Set up serial port with larger timeout for debugging
        self.ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {PORT} at {BAUD_RATE} baud")
        
        # Data buffers (store more samples for better visualization)
        self.buffer_length = 1024  # Adjust based on your needs
        self.raw_data = np.zeros(self.buffer_length)
        self.filtered_data = np.zeros(self.buffer_length)
        
        # Debug variables
        self.last_rx_count = 0
        self.total_packets = 0
        self.debug_counter = 0
        
        # State variables for parsing
        self.frame_sync = False
        self.temp_buffer = bytearray()
        
        # Create figures for time and frequency domain
        plt.ion()  # Interactive mode
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.setup_plots()
        
        # Create thread for data reading
        self.running = True
        self.data_thread = threading.Thread(target=self.read_data_thread)
        self.data_thread.daemon = True
        self.data_thread.start()
    
    def setup_plots(self):
        # Time domain plots
        self.axes[0, 0].set_title('Raw Signal (Time Domain)')
        self.axes[0, 0].set_xlabel('Time (s)')
        self.axes[0, 0].set_ylabel('Amplitude')
        self.raw_time_plot, = self.axes[0, 0].plot([], [], 'b-')
        
        self.axes[0, 1].set_title('Filtered Signal (Time Domain)')
        self.axes[0, 1].set_xlabel('Time (s)')
        self.axes[0, 1].set_ylabel('Amplitude')
        self.filtered_time_plot, = self.axes[0, 1].plot([], [], 'g-')
        
        # Frequency domain plots
        self.axes[1, 0].set_title('Raw Signal (Frequency Domain)')
        self.axes[1, 0].set_xlabel('Frequency (Hz)')
        self.axes[1, 0].set_ylabel('Magnitude (dB)')
        self.raw_freq_plot, = self.axes[1, 0].plot([], [], 'b-')
        
        self.axes[1, 1].set_title('Filtered Signal (Frequency Domain)')
        self.axes[1, 1].set_xlabel('Frequency (Hz)')
        self.axes[1, 1].set_ylabel('Magnitude (dB)')
        self.filtered_freq_plot, = self.axes[1, 1].plot([], [], 'g-')
        
        # Set initial axis limits for better visualization
        for ax in self.axes.flat:
            ax.grid(True)
        
        plt.tight_layout()
    
    def find_frame_markers(self, data):
        """Find all frame marker positions in buffer"""
        markers = []
        for i in range(len(data) - 1):
            if data[i] == FRAME_MARKER and data[i+1] == FRAME_MARKER:
                markers.append(i)
        return markers
    
    def read_data_thread(self):
        """Background thread to continuously read data"""
        while self.running:
            try:
                # Read available data
                if self.ser.in_waiting > 0:
                    new_data = self.ser.read(self.ser.in_waiting)
                    self.temp_buffer.extend(new_data)
                    
                    # Process the buffer if it's getting large
                    if len(self.temp_buffer) > 1024:
                        self.process_buffer()
                
                # Small delay to prevent CPU hogging
                time.sleep(0.001)
            except Exception as e:
                print(f"Error in read thread: {e}")
                time.sleep(0.1)
    
    def process_buffer(self):
        """Process the current buffer looking for frames"""
        # Find all frame markers in the buffer
        markers = self.find_frame_markers(self.temp_buffer)
        
        # Need at least 2 markers (start and end) to have a complete frame
        if len(markers) >= 2:
            # Process each frame
            for i in range(len(markers) - 1):
                start_idx = markers[i]
                end_idx = markers[i+1]
                
                # Check if this looks like a valid frame
                frame_size = end_idx - start_idx
                if 100 < frame_size < 1000:  # Reasonable size for a frame
                    # Extract frame data (skip the markers)
                    frame_data = self.temp_buffer[start_idx+2:end_idx]
                    self.process_frame(frame_data)
                    self.total_packets += 1
        
            # Keep only data after the last marker
            if markers:
                self.temp_buffer = self.temp_buffer[markers[-1]+2:]
            
            # Print debug info occasionally
            self.debug_counter += 1
            if self.debug_counter >= 100:
                self.debug_counter = 0
                print(f"Processed {self.total_packets} packets, buffer size: {len(self.temp_buffer)}")
                
                # Print sample values for debugging
                if len(self.raw_data) > 0:
                    print(f"Raw data sample stats: min={np.min(self.raw_data)}, max={np.max(self.raw_data)}, mean={np.mean(self.raw_data)}")
                if len(self.filtered_data) > 0:
                    print(f"Filtered data sample stats: min={np.min(self.filtered_data)}, max={np.max(self.filtered_data)}, mean={np.mean(self.filtered_data)}")
    
    def process_frame(self, data):
        """Process a complete data frame"""
        unfiltered_samples = []
        filtered_samples = []
        
        # Process data in 4-byte chunks
        for i in range(0, len(data), BYTES_PER_SAMPLE):
            if i + BYTES_PER_SAMPLE <= len(data):
                # Extract 4 bytes
                sample_bytes = data[i:i+BYTES_PER_SAMPLE]
                
                # Unpack as little-endian 32-bit unsigned int
                value = struct.unpack('<I', sample_bytes)[0]
                
                # Extract header (MSB byte)
                header = (value >> 24) & 0xFF
                
                # Extract 24-bit sample (bottom 3 bytes)
                sample = value & 0xFFFFFF
                
                # Sign extension for 24-bit data in 2's complement
                if sample & 0x800000:
                    sample |= 0xFF000000  # Extend the sign bit
                
                # Convert to signed integer (now a 32-bit int with sign extension)
                sample = np.int32(sample)
                
                # Handle based on header
                if header == UNFILTERED_HEADER:
                    unfiltered_samples.append(sample)
                elif header == FILTERED_HEADER:
                    filtered_samples.append(sample)
        
        # Debug
        if unfiltered_samples and self.debug_counter == 0:
            print(f"Received {len(unfiltered_samples)} unfiltered samples, {len(filtered_samples)} filtered samples")
        
        # Update data buffers if we have samples
        if unfiltered_samples:
            # Roll buffer and add new samples
            self.raw_data = np.roll(self.raw_data, -len(unfiltered_samples))
            self.raw_data[-len(unfiltered_samples):] = unfiltered_samples
        
        if filtered_samples:
            # Roll buffer and add new samples
            self.filtered_data = np.roll(self.filtered_data, -len(filtered_samples))
            self.filtered_data[-len(filtered_samples):] = filtered_samples
    
    def update_plots(self):
        """Update all plots with current data"""
        # Only update if we have data
        if np.any(self.raw_data) or np.any(self.filtered_data):
            # Time domain
            time_axis = np.arange(len(self.raw_data)) / SAMPLE_RATE
            
            self.raw_time_plot.set_data(time_axis, self.raw_data)
            self.filtered_time_plot.set_data(time_axis, self.filtered_data)
            
            # Adjust time domain axes limits
            for ax, data in zip([self.axes[0, 0], self.axes[0, 1]], 
                               [self.raw_data, self.filtered_data]):
                if np.any(data):
                    margin = max(1, (np.max(data) - np.min(data)) * 0.1)
                    ax.set_ylim(np.min(data) - margin, np.max(data) + margin)
                ax.set_xlim(0, time_axis[-1])
            
            # FFT
            N = len(self.raw_data)
            if N > 0:
                # Apply window function to reduce spectral leakage
                window = np.hanning(N)
                
                # Compute FFT for both signals if they contain data
                if np.any(self.raw_data):
                    raw_windowed = self.raw_data * window
                    raw_fft = np.abs(fft(raw_windowed))
                    # Convert to dB with protection against log(0)
                    raw_fft = 20 * np.log10(np.maximum(raw_fft[:N//2] / N, 1e-10))
                    
                    # Frequency axis
                    freq_axis = fftfreq(N, 1/SAMPLE_RATE)[:N//2]
                    self.raw_freq_plot.set_data(freq_axis, raw_fft)
                
                if np.any(self.filtered_data):
                    filtered_windowed = self.filtered_data * window
                    filtered_fft = np.abs(fft(filtered_windowed))
                    # Convert to dB with protection against log(0)
                    filtered_fft = 20 * np.log10(np.maximum(filtered_fft[:N//2] / N, 1e-10))
                    
                    # Frequency axis (reuse from above)
                    if 'freq_axis' not in locals():
                        freq_axis = fftfreq(N, 1/SAMPLE_RATE)[:N//2]
                    self.filtered_freq_plot.set_data(freq_axis, filtered_fft)
                
                # Set fixed y-axis for dB plots and x-axis to show up to Nyquist frequency
                for ax in [self.axes[1, 0], self.axes[1, 1]]:
                    ax.set_ylim([-120, 0])
                    ax.set_xlim([0, SAMPLE_RATE/2])
        
        # Update plots
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def run(self):
        """Main processing loop"""
        try:
            update_counter = 0
            while self.running:
                # Update plots periodically (not on every data arrival)
                update_counter += 1
                if update_counter >= 10:
                    update_counter = 0
                    self.update_plots()
                
                # Small pause to allow GUI to update and prevent CPU hogging
                plt.pause(0.01)
        except KeyboardInterrupt:
            print("Stopping visualization...")
        finally:
            self.running = False
            if self.data_thread.is_alive():
                self.data_thread.join(timeout=1.0)
            self.ser.close()
            plt.close()


if __name__ == "__main__":
    print(f"Starting audio visualization with sampling rate: {SAMPLE_RATE} Hz")
    print(f"Connect to serial port {PORT} at {BAUD_RATE} baud")
    print("Press Ctrl+C to stop")
    
    try:
        visualizer = AudioVisualizer()
        visualizer.run()
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print(f"Make sure the device is connected to {PORT} and not in use by another application.")
    except Exception as e:
        print(f"Unexpected error: {e}")