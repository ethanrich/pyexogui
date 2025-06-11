import serial
import time
import queue
import threading
from functools import wraps
import re

from mock_arduino import MockSerial

def timeout(timeout_sec):
    """Decorator to add timeout functionality to methods"""
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            result_queue = queue.Queue()
            
            def target():
                try:
                    result = func(*args, **kwargs)
                    result_queue.put(("success", result))
                except Exception as e:
                    result_queue.put(("error", e))
            
            thread = threading.Thread(target=target)
            thread.daemon = True
            thread.start()
            
            try:
                status, result = result_queue.get(timeout=timeout_sec)
                if status == "error":
                    raise result
                return result
            except queue.Empty:
                raise TimeoutError(f"Operation timed out after {timeout_sec} seconds")
        
        return wrapper
    return decorator


class CommandSender:
    """Class responsible for sending position commands to the exoskeleton"""
    
    def __init__(self, port=None, baudrate=115200, num_fingers=5, position_tolerance=5, use_mock=False):
        """
        Initialize the command sender
        
        Args:
            port: COM port string (e.g., 'COM3' or '/dev/ttyUSB0')
            baudrate: Serial communication baudrate (default: 115200)
            num_fingers: Number of fingers to control (default: 5)
            position_tolerance: Tolerance in mm for position checking (default: 5)
            use_mock: If True, use mock Arduino instead of real serial port
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.num_fingers = num_fingers
        self.position_tolerance = position_tolerance
        self.current_positions = [0] * num_fingers
        self.finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        self.is_connected = False
        self.position_lock = threading.Lock()
        self.position_update_interval = 0.1  # How often to request positions (seconds)
        self.last_position_request_time = 0
        self.use_mock = use_mock
    
    def parse_position_data(self, data_string):
        """
        Parse position data from the device
        Format: F1:xx.xx,F2:xx.xx,F3:xx.xx,F4:xx.xx,F5:xx.xx
        
        Args:
            data_string: Raw position data string from device
            
        Returns:
            List of positions in mm, or None if parsing fails
        """
        try:
            # Use regex to extract all position values
            pattern = r'F(\d):(\d+\.?\d*)'
            matches = re.findall(pattern, data_string)
            
            if len(matches) != self.num_fingers:
                return None
            
            positions = [0] * self.num_fingers
            for match in matches:
                finger_num = int(match[0]) - 1  # Convert to 0-based index
                position = float(match[1])
                if 0 <= finger_num < self.num_fingers:
                    positions[finger_num] = position
            
            return positions
        except Exception as e:
            print(f"Error parsing position data: {e}")
            return None
    
    def request_and_parse_positions(self, max_retries=3):
        """
        Request position data from the device and parse the response
        
        Returns:
            List of positions in mm, or None if failed
        """
        if not self.is_connected or not self.serial_port or not self.serial_port.is_open:
            return None
        
        for attempt in range(max_retries):
            try:
                # Clear any pending data
                self.serial_port.reset_input_buffer()
                
                # Send position request
                self.serial_port.write(b"GET_POS\n")
                self.serial_port.flush()
                
                # Wait for response (with timeout)
                start_time = time.time()
                while time.time() - start_time < 0.5:  # 500ms timeout
                    if self.serial_port.in_waiting:
                        line = self.serial_port.readline().decode().strip()
                        
                        # Skip acknowledgment messages
                        if line.startswith("ACK:") or line.startswith("ERR:") or line.startswith("STATUS:"):
                            continue
                        
                        # Try to parse position data
                        positions = self.parse_position_data(line)
                        if positions:
                            return positions
                    
                    time.sleep(0.01)
                
                # If we get here, attempt failed
                if attempt < max_retries - 1:
                    time.sleep(0.1)  # Wait a bit before retry
                    
            except Exception as e:
                if attempt == max_retries - 1:
                    print(f"Error requesting positions: {e}")
        
        return None
    
    def update_current_positions(self):
        """
        Update the current positions by requesting from device
        
        Returns:
            True if positions were updated successfully
        """
        positions = self.request_and_parse_positions()
        if positions:
            with self.position_lock:
                self.current_positions = positions
            return True
        return False
    
    @timeout(15)
    def attempt_connection(self, port=None, baudrate=None):
        """
        Attempt to connect to the device
        
        Args:
            port: COM port string (uses instance port if not provided)
            baudrate: Serial baudrate (uses instance baudrate if not provided)
            
        Returns:
            Serial connection object
            
        Raises:
            serial.SerialException: If connection fails
            TimeoutError: If connection times out
        """
        port = port or self.port
        baudrate = baudrate or self.baudrate
        
        if not port and not self.use_mock:
            raise ValueError("No port specified")
        
        # Create serial connection (real or mock)
        if self.use_mock:
            ser = MockSerial(port or "MOCK", baudrate)
        else:
            ser = serial.Serial(port, baudrate, timeout=1)
        
        # Give the device a moment to initialize
        time.sleep(2.5)
        
        # Clear any existing data in the buffer
        ser.reset_input_buffer()
        
        # Wait for Arduino to send READY signal
        ready_received = False
        start_time = time.time()
        while time.time() - start_time < 2:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                if line == "READY":
                    ready_received = True
                    print("Arduino ready signal received")
                    break
        
        if not ready_received:
            print("Warning: No READY signal received from Arduino")
        
        # Temporarily assign serial port to use request_and_parse_positions
        temp_serial = self.serial_port
        temp_connected = self.is_connected
        self.serial_port = ser
        self.is_connected = True
        
        try:
            # Try multiple times to get position data
            for attempt in range(5):
                if self.update_current_positions():
                    # Successfully received and parsed position data
                    return ser
                time.sleep(0.2)  # Wait between attempts
            
            # If we get here, all attempts failed
            ser.close()
            raise serial.SerialException("No valid position data received from device after multiple attempts")
            
        finally:
            # Restore original state if connection failed
            if not ser.is_open:
                self.serial_port = temp_serial
                self.is_connected = temp_connected
    
    def connect(self, port=None, baudrate=None):
        """
        Connect to the exoskeleton device
        
        Args:
            port: COM port string (uses instance port if not provided)
            baudrate: Serial baudrate (uses instance baudrate if not provided)
            
        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Update port and baudrate if provided
            if port:
                self.port = port
            if baudrate:
                self.baudrate = baudrate
            
            # Close existing connection if any
            self.disconnect()
            
            # Attempt new connection
            self.serial_port = self.attempt_connection(self.port, self.baudrate)
            self.is_connected = True
            
            if self.use_mock:
                print(f"Successfully connected to MOCK Arduino")
            else:
                print(f"Successfully connected to {self.port} at {self.baudrate} baud")
            return True
            
        except TimeoutError:
            print(f"Connection to {self.port} timed out")
            self.is_connected = False
            return False
        except serial.SerialException as e:
            print(f"Serial connection error: {e}")
            self.is_connected = False
            return False
        except Exception as e:
            print(f"Unexpected error during connection: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the device"""
        # Close serial port
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print("Disconnected from device")
            except Exception as e:
                print(f"Error during disconnect: {e}")
        
        self.serial_port = None
        self.is_connected = False
    
    def toggle_connection(self, port=None, baudrate=None):
        """
        Toggle connection state
        
        Args:
            port: COM port string (uses instance port if not provided)
            baudrate: Serial baudrate (uses instance baudrate if not provided)
            
        Returns:
            True if now connected, False if now disconnected
        """
        if self.is_connected:
            self.disconnect()
            return False
        else:
            return self.connect(port, baudrate)
    
    def calculate_command_from_position(self, finger_idx, position_percent):
        """
        Calculate PWM command from position percentage
        
        Args:
            finger_idx: Index of the finger (0-based)
            position_percent: Desired position as percentage (0-100)
            
        Returns:
            Command string in format "P<finger_number><pwm_value>\n"
        """
        # 0% position maps to 130 PWM, and 100% position maps to 47 PWM
        # This is an inverse relationship - as position increases, PWM decreases
        pwm_value = int(130 - (position_percent / 100.0) * (130 - 47))
        pwm_value = max(47, min(130, pwm_value))
        
        command = f"P{finger_idx + 1}{pwm_value}\n"
        print("Sending " + command.strip())
        return command
    
    def send_position_command(self, command):
        """
        Send a position command to the exoskeleton
        
        Args:
            command: Command string to send
            
        Returns:
            True if command was sent successfully and acknowledged
            
        Raises:
            serial.SerialException: If there's a communication error
        """
        if not self.is_connected or self.serial_port is None:
            raise serial.SerialException("Not connected to device")
        
        if not self.serial_port.is_open:
            raise serial.SerialException("Serial port is closed")
        
        # Clear input buffer before sending command
        self.serial_port.reset_input_buffer()
        
        # Send command
        self.serial_port.write(command.encode())
        self.serial_port.flush()
        
        # Wait for acknowledgment
        start_time = time.time()
        while time.time() - start_time < 0.5:  # 500ms timeout for ACK
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode().strip()
                if line.startswith("ACK:"):
                    return True
                elif line.startswith("ERR:"):
                    print(f"Error from device: {line}")
                    return False
            time.sleep(0.01)
        
        print("Warning: No acknowledgment received for command")
        return True  # Assume success if no ACK (for backward compatibility)
    
    def send_finger_position(self, finger_idx, position_percent):
        """
        Send position command for a specific finger
        
        Args:
            finger_idx: Index of the finger (0-based)
            position_percent: Desired position as percentage (0-100)
            
        Returns:
            True if command sent successfully
        """
        if finger_idx < 0 or finger_idx >= self.num_fingers:
            raise ValueError(f"Invalid finger index: {finger_idx}")
        
        if not (0 <= position_percent <= 100):
            raise ValueError(f"Position must be between 0 and 100, got: {position_percent}")
        
        command = self.calculate_command_from_position(finger_idx, position_percent)
        return self.send_position_command(command)
    
    def send_all_positions(self, positions):
        """
        Send position commands for all fingers
        
        Args:
            positions: List of position percentages for each finger
            
        Returns:
            List of success status for each command
        """
        if len(positions) != self.num_fingers:
            raise ValueError(f"Expected {self.num_fingers} positions, got {len(positions)}")
        
        results = []
        for finger_idx, position in enumerate(positions):
            try:
                success = self.send_finger_position(finger_idx, position)
                results.append(success)
                time.sleep(0.02)  # Small delay between commands
            except Exception as e:
                print(f"Error sending command to {self.finger_names[finger_idx]}: {e}")
                results.append(False)
        
        return results
    
    def get_current_positions(self):
        """
        Get the current positions of all fingers
        
        Returns:
            List of current positions in mm
        """
        # Update positions if enough time has passed since last request
        current_time = time.time()
        if current_time - self.last_position_request_time >= self.position_update_interval:
            self.update_current_positions()
            self.last_position_request_time = current_time
        
        with self.position_lock:
            return self.current_positions.copy()
    
    def get_current_position(self, finger_idx):
        """
        Get the current position of a specific finger
        
        Args:
            finger_idx: Index of the finger (0-based)
            
        Returns:
            Current position in mm
        """
        if 0 <= finger_idx < self.num_fingers:
            positions = self.get_current_positions()
            return positions[finger_idx]
        return 0
    
    def position_percent_to_mm(self, position_percent):
        """Convert position percentage to millimeters"""
        return (position_percent / 100.0) * 50.0
    
    def mm_to_position_percent(self, mm):
        """Convert millimeters to position percentage"""
        return (mm / 50.0) * 100.0
    
    def wait_for_position(self, finger_idx, target_position_mm, timeout=30,
                          check_callback=None):
        """
        Wait until the actuator reaches the target position within tolerance
        
        Args:
            finger_idx: Index of the finger (0-based)
            target_position_mm: Target position in millimeters
            timeout: Maximum time to wait in seconds
            check_callback: Optional callback function to check if we should stop waiting
            
        Returns:
            True if position reached, False if timeout or cancelled
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Force position update
            self.update_current_positions()
            
            current_pos = self.get_current_position(finger_idx)
            if abs(current_pos - target_position_mm) <= self.position_tolerance:
                return True
            
            # Check if we should stop waiting (e.g., if a stop command was issued)
            if check_callback and not check_callback():
                return False
            
            time.sleep(0.1)  # Check every 100ms
        
        return False
    
    def wait_for_position_percent(self, finger_idx, target_position_percent, timeout=30,
                                  check_callback=None):
        """
        Wait until the actuator reaches the target position percentage within tolerance
        
        Args:
            finger_idx: Index of the finger (0-based)
            target_position_percent: Target position as percentage (0-100)
            timeout: Maximum time to wait in seconds
            check_callback: Optional callback function to check if we should stop waiting
            
        Returns:
            True if position reached, False if timeout or cancelled
        """
        target_mm = self.position_percent_to_mm(target_position_percent)
        return self.wait_for_position(finger_idx, target_mm, timeout, check_callback)
    
    def get_available_ports(self):
        """
        Get list of available COM ports
        
        Returns:
            List of available port names
        """
        if self.use_mock:
            return ["MOCK"]
        
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def test_connection(self):
        """
        Test if the connection is working by requesting position data
        
        Returns:
            True if connection is working, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            # Try to get fresh position data
            return self.update_current_positions()
        except Exception:
            return False
    
    def estimate_movement_speed(self):
        """
        Estimate the movement speed of the exoskeleton by sending commands
        and progressively checking positions with shorter delays.
        
        Returns:
            Dictionary with movement rates for each finger in mm/s
        """
        if not self.is_connected:
            print("Not connected to device. Please connect first.")
            return None
        
        print("\n=== Starting Movement Speed Estimation ===")
        
        # Store results
        movement_rates = {}
        
        # Test parameters
        start_position = 0  # Start fully closed
        end_position = 100  # Move to fully open
        
        try:
            # First, move all fingers to starting position
            print(f"\nMoving all fingers to starting position ({start_position}%)...")
            self.send_all_positions([start_position] * self.num_fingers)
            
            # Wait for fingers to reach starting position
            print("Waiting for fingers to reach starting position...")
            for i in range(self.num_fingers):
                target_mm = self.position_percent_to_mm(start_position)
                if not self.wait_for_position(i, target_mm, timeout=30):
                    print(f"Warning: {self.finger_names[i]} finger didn't reach start position")
            
            time.sleep(2)  # Extra settling time
            
            # Get actual starting positions
            self.update_current_positions()
            initial_positions = self.get_current_positions()
            print(f"Starting positions (mm): {[f'{p:.1f}' for p in initial_positions]}")
            
            # Now test movement speed
            print(f"\nMoving all fingers to end position ({end_position}%)...")
            
            # Record the exact time when command is sent
            command_send_time = time.time()
            self.send_all_positions([end_position] * self.num_fingers)
            
            # Track when each finger reaches target
            finger_completion_times = [None] * self.num_fingers
            finger_reached_target = [False] * self.num_fingers
            
            check_count = 0
            
            print("\nMonitoring finger positions:")
            
            while not all(finger_reached_target) and time.time() - command_send_time < 60:
                # Update positions
                self.update_current_positions()
                current_positions = self.get_current_positions()
                
                check_time = time.time()
                elapsed_time = check_time - command_send_time
                
                # Check which fingers have reached target
                target_mm = self.position_percent_to_mm(end_position)
                
                if check_count % 10 == 0:  # Print every 10th check
                    print(f"\nCheck #{check_count + 1} (after {elapsed_time:.1f}s):")
                    print(f"Positions (mm): {[f'{p:.1f}' for p in current_positions]}")
                
                for i in range(self.num_fingers):
                    if not finger_reached_target[i]:
                        if abs(current_positions[i] - target_mm) <= self.position_tolerance:
                            finger_reached_target[i] = True
                            finger_completion_times[i] = elapsed_time
                            print(f"  {self.finger_names[i]} reached target after {elapsed_time:.1f}s!")
                
                check_count += 1
                time.sleep(0.1)  # Check every 100ms
            
            # Calculate movement rates
            print("\n=== Movement Speed Results ===")
            
            for i in range(self.num_fingers):
                if finger_completion_times[i] is not None:
                    # Calculate distance traveled in mm
                    start_mm = self.position_percent_to_mm(start_position)
                    end_mm = self.position_percent_to_mm(end_position)
                    distance_mm = abs(end_mm - start_mm)
                    
                    # Calculate rate
                    time_taken = finger_completion_times[i]
                    rate_mm_per_sec = distance_mm / time_taken if time_taken > 0 else 0
                    
                    movement_rates[self.finger_names[i]] = {
                        'time_seconds': time_taken,
                        'distance_mm': distance_mm,
                        'rate_mm_per_sec': rate_mm_per_sec,
                        'rate_percent_per_sec': (abs(end_position - start_position) / time_taken) if time_taken > 0 else 0
                    }
                    
                    print(f"\n{self.finger_names[i]}:")
                    print(f"  Time to move: {time_taken:.2f} seconds")
                    print(f"  Distance: {distance_mm:.1f} mm")
                    print(f"  Speed: {rate_mm_per_sec:.2f} mm/s ({movement_rates[self.finger_names[i]]['rate_percent_per_sec']:.1f}%/s)")
                else:
                    print(f"\n{self.finger_names[i]}: Did not reach target")
            
            # Calculate average speed
            if movement_rates:
                avg_speed_mm = sum(data['rate_mm_per_sec'] for data in movement_rates.values()) / len(movement_rates)
                avg_speed_percent = sum(data['rate_percent_per_sec'] for data in movement_rates.values()) / len(movement_rates)
                print(f"\nAverage movement speed: {avg_speed_mm:.2f} mm/s ({avg_speed_percent:.1f}%/s)")
            
            return movement_rates
            
        except Exception as e:
            print(f"Error during speed estimation: {e}")
            return None


if __name__ == "__main__":
    # Create command sender with mock Arduino
    command_sender = CommandSender(use_mock=True)
    
    # Get available ports
    available_ports = command_sender.get_available_ports()
    print(f"Available ports: {available_ports}")
    
    # Connect to mock device
    if command_sender.connect():
        print("Connected successfully!")
        
        try:
            # Test connection
            if command_sender.test_connection():
                print("Connection test passed!")
                
                # Display current positions
                positions = command_sender.get_current_positions()
                print(f"Current positions (mm): {[f'{p:.1f}' for p in positions]}")
                
                # Send all fingers to open position
                print("\nOpening hand...")
                command_sender.send_all_positions([100, 100, 100, 100, 100])
                
                # Wait and check positions
                time.sleep(5)
                positions = command_sender.get_current_positions()
                print(f"Positions after opening: {[f'{p:.1f}' for p in positions]} mm")
                
                # Send all fingers to OK sign position
                print("\nMaking OK sign...")
                results = command_sender.send_all_positions([30, 30, 80, 85, 90])
                print(f"Commands sent: {results}")
                
                # Monitor positions
                time.sleep(5)
                positions = command_sender.get_current_positions()
                print(f"Positions after OK sign: {[f'{p:.1f}' for p in positions]} mm")
                
                # Test movement speed estimation
                print("\n" + "="*50)
                input("Press Enter to start movement speed estimation...")
                command_sender.estimate_movement_speed()
                
        except Exception as e:
            print(f"Error during operation: {e}")
        
        finally:
            # Disconnect when done
            command_sender.disconnect()
    else:
        print("Failed to connect to device")