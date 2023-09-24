import math
import time
import csv
from datetime import datetime
import serial

class PhotogrammetryController:
    
    def __init__(self):
        # Initializing the essential parameters.
        self.setup_hardware_parameters()
        self.setup_output_parameters()
        self.setup_serial()

    def setup_hardware_parameters(self):
        """Configure the hardware limits and parameters."""
        # Adjust these as per your specific hardware setup.
        self.x_max_limit = 50
        self.z_max_limit = 90
        self.e_max_limit = 360
        self.camera_distance = 60  # Optimal distance for capturing images.
        self.servo_start_position = 0
        self.servo_end_position = 180
        self.servo_delay = 1  # Short pause for the servo.
        self.speed_X = 1000
        self.speed_Y = 1000
        self.speed_Z = 1000
        self.speed_E = 1000
        self.combined_speed = 1500

    def setup_output_parameters(self):
        """Designate the output file with a unique timestamp."""
        # This ensures no overwrite issues.
        self.file_name = f"camera_positions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    def setup_serial(self):
        """Prepare the serial connection settings."""
        # Ensure the correct port and baudrate for the device.
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.serial_conn = self.init_serial_connection()

    def init_serial_connection(self):
        """Attempt to establish a serial connection."""
        try:
            connection = serial.Serial(self.port, self.baudrate, timeout=1)
            connection.flush()
            return connection
        except Exception as e:
            print(f"Error establishing serial connection: {e}")
            return None

    def close_serial_connection(self):
        """Close the active serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    def send_gcode_command(self, command):
        """Transmit the G-code command via serial."""
        if self.serial_conn:
            self.serial_conn.write((command + '\n').encode('utf-8'))
            response = self.serial_conn.readline().decode('utf-8').strip()
            print(f"Device response: {response}")
        else:
            print("No active serial connection.")

    def cartesian_coordinates_and_orientation(self, x, z, e_rotation, y_rotation):
        """Calculate cartesian coordinates based on given positions."""
        X = x * math.cos(math.radians(e_rotation))
        Y = x * math.sin(math.radians(e_rotation))
        roll = 0
        pitch = y_rotation
        yaw = e_rotation
        return X, Y, z, roll, pitch, yaw

    def move_camera(self, z_position, e_rotation):
        """Adjust camera to the desired position."""
        x_adjusted = math.sqrt(self.camera_distance ** 2 - (z_position - self.z_max_limit / 2) ** 2)
        gcode = f"G1 Z{z_position} E{e_rotation} X{x_adjusted} F{self.combined_speed}"
        self.send_gcode_command(gcode)

    def activate_servo(self):
        """Command the servo to trigger the camera."""
        self.send_gcode_command(f"M280 P0 S{self.servo_end_position}")
        time.sleep(self.servo_delay)
        self.send_gcode_command(f"M280 P0 S{self.servo_start_position}")

    def generate_positions(self, z_divs, e_divs):
        """Generate a list of positions for camera movements."""
        positions = []
        e_step = self.e_max_limit / e_divs
        z_step = self.camera_distance / z_divs if self.camera_distance < self.z_max_limit else self.z_max_limit / z_divs
        for i in range(z_divs):
            for j in range(e_divs):
                z_position = i * z_step
                e_rotation = j * e_step
                positions.append((self.camera_distance, z_position, e_rotation, 0))
        return positions

    def save_to_csv(self, positions):
        """Save the positions and orientations to a CSV file."""
        with open(self.file_name, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile, delimiter=',')
            csv_writer.writerow(['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'])
            for pos in positions:
                X, Y, Z, roll, pitch, yaw = self.cartesian_coordinates_and_orientation(*pos)
                csv_writer.writerow([X, Y, Z, roll, pitch, yaw])

    def execute_sequence(self, z_divs, e_divs):
        """Execute the sequence of movements and capture images."""
        positions = self.generate_positions(z_divs, e_divs)
        total_positions = len(positions)
        for idx, pos in enumerate(positions):
            x, z, e, y = pos
            self.move_camera(z, e)
            self.activate_servo()
            X, Y, Z, roll, pitch, yaw = self.cartesian_coordinates_and_orientation(x, z, e, y)
            print(f"Position: X:{X} Y:{Y} Z:{Z} E:{e}")
            print(f"Orientation: Roll:{roll} Pitch:{pitch} Yaw:{yaw}")
            print(f"Progress: {idx + 1}/{total_positions}")
        self.save_to_csv(positions)
        self.close_serial_connection()

def main():
    """Main driver function to execute the program."""
    controller = PhotogrammetryController()
    z_divisions = int(input("Enter number of divisions for the Z axis: "))
    e_divisions = int(input("Enter number of divisions for the E axis: "))
    controller.execute_sequence(z_divisions, e_divisions)

if __name__ == "__main__":
    main()
