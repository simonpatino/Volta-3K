import serial
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from datetime import datetime
import time

#pip install pyserial matplotlib pandas numpy

class ArduinoDataVisualizer:
    def __init__(self, port='COM3', baudrate=115200):
        # Serial connection
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        
        # Data columns
        self.columns = ['Cycle', 'Time', 'Temp_C', 'Press_hPa', 'Alt_m', 'humidity', 
                       'MaxAlt_m', 'AccelX', 'AccelY', 'AccelZ', 'gyroX', 'gyroY', 
                       'gyroZ', 'EulerX', 'EulerY', 'EulerZ', 'Stage', 'GPS_Lat', 
                       'GPS_Lon', 'GPS_Valid', 'GPS_Sats']
        
        self.df = None
        
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=2)
            print(f"Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)  # Allow connection to stabilize
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def read_complete_data(self):
        """Read the complete data file from Arduino serial"""
        print("Reading data from Arduino...")
        print("Waiting for data transmission to start...")
        
        all_lines = []
        start_time = time.time()
        last_data_time = time.time()
        timeout_duration = 5  # seconds without data before assuming transmission is complete
        
        # Skip any initial garbage or setup messages
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and ',' in line and line.count(',') >= 10:  # Looks like data
                    all_lines.append(line)
                    last_data_time = time.time()
                    if len(all_lines) == 1:
                        print("Data transmission started...")
                    if len(all_lines) % 50 == 0:
                        print(f"Received {len(all_lines)} lines...")
                elif line:
                    print(f"Skipping: {line}")
            
            # Check if we haven't received data for a while
            if time.time() - last_data_time > timeout_duration and all_lines:
                print(f"No new data for {timeout_duration} seconds. Transmission complete.")
                break
            
            # Overall timeout
            if time.time() - start_time > 300:  # 5 minutes max
                print("Timeout reached. Stopping data collection.")
                break
        
        print(f"Collected {len(all_lines)} lines of data")
        
        if not all_lines:
            print("No valid data received!")
            return False
        
        # Parse the data
        parsed_data = []
        for line in all_lines:
            try:
                values = line.split(',')
                if len(values) == len(self.columns):
                    row = {}
                    for i, col in enumerate(self.columns):
                        try:
                            if col == 'GPS_Valid':
                                row[col] = bool(int(values[i])) if values[i].strip() else False
                            else:
                                row[col] = float(values[i]) if values[i].strip() else 0.0
                        except ValueError:
                            row[col] = 0.0
                    parsed_data.append(row)
            except Exception as e:
                print(f"Error parsing line: {line[:50]}... - {e}")
        
        if parsed_data:
            self.df = pd.DataFrame(parsed_data)
            print(f"Successfully parsed {len(self.df)} data points")
            return True
        else:
            print("Failed to parse any data!")
            return False
    
    def create_visualizations(self):
        """Create comprehensive static visualizations"""
        if self.df is None or len(self.df) == 0:
            print("No data to visualize!")
            return
        
        # Create figure with subplots
        plt.style.use('dark_background')
        fig = plt.figure(figsize=(20, 16))
        fig.suptitle(f'Arduino Flight Data Analysis - {len(self.df)} Data Points', 
                     fontsize=18, fontweight='bold', color='white')
        
        # Define subplot layout (4x3 grid)
        gs = fig.add_gridspec(4, 3, hspace=0.4, wspace=0.3)
        
        time_data = self.df['Time']
        
        # 1. Environmental Sensors
        ax1 = fig.add_subplot(gs[0, 0])
        ax1_twin = ax1.twinx()
        ax1.plot(time_data, self.df['Temp_C'], 'r-', linewidth=2, label='Temperature (°C)')
        ax1_twin.plot(time_data, self.df['humidity'], 'b-', linewidth=2, label='Humidity (%)')
        ax1.set_title('Environmental Sensors', fontweight='bold', color='white')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Temperature (°C)', color='red')
        ax1_twin.set_ylabel('Humidity (%)', color='blue')
        ax1.grid(True, alpha=0.3)
        ax1.tick_params(colors='white')
        ax1_twin.tick_params(colors='white')
        
        # 2. Altitude Profile
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(time_data, self.df['Alt_m'], 'g-', linewidth=2, label='Current Altitude')
        ax2.plot(time_data, self.df['MaxAlt_m'], 'y--', linewidth=2, label='Max Altitude')
        ax2.set_title('Altitude Profile', fontweight='bold', color='white')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Altitude (m)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.tick_params(colors='white')
        
        # 3. Pressure
        ax3 = fig.add_subplot(gs[0, 2])
        ax3.plot(time_data, self.df['Press_hPa'], 'c-', linewidth=2)
        ax3.set_title('Atmospheric Pressure', fontweight='bold', color='white')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Pressure (hPa)')
        ax3.grid(True, alpha=0.3)
        ax3.tick_params(colors='white')
        
        # 4. Acceleration
        ax4 = fig.add_subplot(gs[1, 0])
        ax4.plot(time_data, self.df['AccelX'], 'r-', linewidth=1.5, label='X-axis', alpha=0.8)
        ax4.plot(time_data, self.df['AccelY'], 'g-', linewidth=1.5, label='Y-axis', alpha=0.8)
        ax4.plot(time_data, self.df['AccelZ'], 'b-', linewidth=1.5, label='Z-axis', alpha=0.8)
        # Calculate total acceleration
        total_accel = np.sqrt(self.df['AccelX']**2 + self.df['AccelY']**2 + self.df['AccelZ']**2)
        ax4.plot(time_data, total_accel, 'white', linewidth=2, label='Total', alpha=0.9)
        ax4.set_title('Acceleration', fontweight='bold', color='white')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Acceleration (g)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.tick_params(colors='white')
        
        # 5. Gyroscope
        ax5 = fig.add_subplot(gs[1, 1])
        ax5.plot(time_data, self.df['gyroX'], 'r-', linewidth=1.5, label='X-axis', alpha=0.8)
        ax5.plot(time_data, self.df['gyroY'], 'g-', linewidth=1.5, label='Y-axis', alpha=0.8)
        ax5.plot(time_data, self.df['gyroZ'], 'b-', linewidth=1.5, label='Z-axis', alpha=0.8)
        ax5.set_title('Angular Velocity', fontweight='bold', color='white')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Angular Velocity (°/s)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.tick_params(colors='white')
        
        # 6. Euler Angles
        ax6 = fig.add_subplot(gs[1, 2])
        ax6.plot(time_data, self.df['EulerX'], 'r-', linewidth=1.5, label='Roll', alpha=0.8)
        ax6.plot(time_data, self.df['EulerY'], 'g-', linewidth=1.5, label='Pitch', alpha=0.8)
        ax6.plot(time_data, self.df['EulerZ'], 'b-', linewidth=1.5, label='Yaw', alpha=0.8)
        ax6.set_title('Euler Angles', fontweight='bold', color='white')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Angle (°)')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        ax6.tick_params(colors='white')
        
        # 7. GPS Trajectory
        ax7 = fig.add_subplot(gs[2, 0])
        valid_gps = self.df[self.df['GPS_Valid'] == True]
        if len(valid_gps) > 0:
            ax7.scatter(valid_gps['GPS_Lon'], valid_gps['GPS_Lat'], 
                       c=valid_gps['Alt_m'], cmap='viridis', s=20, alpha=0.7)
            if len(valid_gps) > 1:
                ax7.plot(valid_gps['GPS_Lon'], valid_gps['GPS_Lat'], 'r-', alpha=0.5, linewidth=1)
            cbar = plt.colorbar(ax7.collections[0], ax=ax7)
            cbar.set_label('Altitude (m)', color='white')
            cbar.ax.tick_params(colors='white')
        ax7.set_title('GPS Trajectory', fontweight='bold', color='white')
        ax7.set_xlabel('Longitude')
        ax7.set_ylabel('Latitude')
        ax7.grid(True, alpha=0.3)
        ax7.tick_params(colors='white')
        
        # 8. Flight Stages
        ax8 = fig.add_subplot(gs[2, 1])
        ax8.plot(time_data, self.df['Stage'], 'orange', linewidth=3, marker='o', markersize=3)
        ax8.set_title('Flight Stages', fontweight='bold', color='white')
        ax8.set_xlabel('Time (s)')
        ax8.set_ylabel('Stage')
        ax8.grid(True, alpha=0.3)
        ax8.tick_params(colors='white')
        
        # 9. GPS Satellites
        ax9 = fig.add_subplot(gs[2, 2])
        ax9.plot(time_data, self.df['GPS_Sats'], 'lime', linewidth=2)
        ax9.fill_between(time_data, self.df['GPS_Sats'], alpha=0.3, color='lime')
        ax9.set_title('GPS Satellites', fontweight='bold', color='white')
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('Number of Satellites')
        ax9.grid(True, alpha=0.3)
        ax9.tick_params(colors='white')
        
        # 10. Flight Statistics
        ax10 = fig.add_subplot(gs[3, :])
        ax10.axis('off')
        
        # Calculate statistics
        max_alt = self.df['Alt_m'].max()
        max_temp = self.df['Temp_C'].max()
        min_temp = self.df['Temp_C'].min()
        max_accel = total_accel.max()
        flight_duration = self.df['Time'].max() - self.df['Time'].min()
        
        stats_text = f"""
FLIGHT STATISTICS:
═══════════════════════════════════════════════════════════════════════════════
Maximum Altitude: {max_alt:.1f} m                    Flight Duration: {flight_duration:.1f} s
Temperature Range: {min_temp:.1f}°C to {max_temp:.1f}°C           Maximum Acceleration: {max_accel:.1f} g
Total Data Points: {len(self.df)}                     GPS Points: {len(valid_gps)} valid
Final Stage: {int(self.df['Stage'].iloc[-1])}                            Average GPS Satellites: {self.df['GPS_Sats'].mean():.1f}
        """
        
        ax10.text(0.5, 0.5, stats_text, transform=ax10.transAxes,
                 fontsize=14, verticalalignment='center', horizontalalignment='center',
                 color='white', fontfamily='monospace',
                 bbox=dict(boxstyle='round,pad=1', facecolor='black', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
        
        # Save the plot
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        fig.savefig(f'flight_data_analysis_{timestamp}.png', dpi=300, bbox_inches='tight',
                   facecolor='black', edgecolor='none')
        print(f"Visualization saved as flight_data_analysis_{timestamp}.png")
    
    def save_data(self, filename=None):
        """Save the collected data to CSV"""
        if self.df is None:
            print("No data to save!")
            return
        
        if not filename:
            filename = f"arduino_flight_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        self.df.to_csv(filename, index=False)
        print(f"Data saved to {filename}")
    
    def run_analysis(self):
        """Main function to run the complete analysis"""
        print("Arduino Flight Data Analyzer")
        print("=" * 40)
        
        if not self.connect_serial():
            return
        
        try:
            # Read all data from Arduino
            if self.read_complete_data():
                # Save raw data
                self.save_data()
                
                # Create visualizations
                self.create_visualizations()
                
                print("\nAnalysis complete!")
                print(f"Data points collected: {len(self.df)}")
                print("Visualization displayed and saved.")
            else:
                print("Failed to collect data from Arduino.")
                
        except KeyboardInterrupt:
            print("\nAnalysis interrupted by user.")
        finally:
            if self.ser:
                self.ser.close()
                print("Serial connection closed.")

if __name__ == "__main__":
    # Change port as needed: 'COM3' (Windows), '/dev/ttyUSB0' (Linux), '/dev/cu.usbmodem...' (Mac)
    analyzer = ArduinoDataVisualizer(port='/dev/cu.usbmodem141101', baudrate=115200)
    analyzer.run_analysis()
