import threading
import time
import queue
from collections import deque

class ArduinoMock:
    """Class that mocks the behavior of an exoskeleton actuator controller"""
    def __init__(self):
        # Servo states
        self.servo_positions = [47, 47, 47, 47, 47]  # Current PWM values
        self.target_positions = [47, 47, 47, 47, 47]  # Target PWM values
        
        # Movement simulation parameters
        self.movement_speed = 50  # PWM units per second
        self.last_update_time = time.time()
        
        # ADC simulation parameters
        self.adc_min = 72
        self.adc_max = 651
        
        # Communication
        self.input_buffer = queue.Queue()
        self.output_buffer = queue.Queue()
        
        # Reporting control
        self.continuous_reporting = False
        self.last_report_time = 0
        self.report_interval = 0.1  # 100ms
        
        # Control flags
        self.running = True
        self._lock = threading.Lock()
        
        # Start the mock Arduino loop
        self.thread = threading.Thread(target=self._arduino_loop, daemon=True)
        self.thread.start()
        
        # Send initial READY message
        time.sleep(0.1)
        self._write_output("READY")
    
    def write(self, data):
        """Simulate serial write (from Python to Arduino)"""
        if isinstance(data, bytes):
            data = data.decode('utf-8')
        # Split by newlines and add each command to the input buffer
        for line in data.strip().split('\n'):
            if line:
                self.input_buffer.put(line)
    
    def readline(self):
        """Simulate serial readline (from Arduino to Python)"""
        try:
            # Wait for data with a timeout
            data = self.output_buffer.get(timeout=1.0)
            return (data + '\n').encode('utf-8')
        except queue.Empty:
            return b''
    
    def in_waiting(self):
        """Return number of bytes waiting to be read"""
        return self.output_buffer.qsize()
    
    def reset_input_buffer(self):
        """Clear the input buffer"""
        while not self.output_buffer.empty():
            try:
                self.output_buffer.get_nowait()
            except queue.Empty:
                break
    
    def reset_output_buffer(self):
        """Clear the output buffer"""
        while not self.input_buffer.empty():
            try:
                self.input_buffer.get_nowait()
            except queue.Empty:
                break
    
    def flush(self):
        """Simulate flush (no-op for mock)"""
        pass
    
    def close(self):
        """Stop the mock Arduino"""
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
    
    @property
    def is_open(self):
        """Check if mock is running"""
        return self.running and self.thread.is_alive()
    
    def _write_output(self, message):
        """Write a message to the output buffer"""
        self.output_buffer.put(message)
    
    def _update_servo_positions(self):
        """Simulate servo movement over time"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        with self._lock:
            for i in range(5):
                if self.servo_positions[i] != self.target_positions[i]:
                    # Calculate movement
                    max_movement = self.movement_speed * dt
                    diff = self.target_positions[i] - self.servo_positions[i]
                    
                    if abs(diff) <= max_movement:
                        self.servo_positions[i] = self.target_positions[i]
                    else:
                        self.servo_positions[i] += max_movement if diff > 0 else -max_movement
    
    def _pwm_to_mm(self, pwm):
        """Convert PWM value to position in mm"""
        # PWM 130 = 0mm (closed), PWM 47 = 50mm (open)
        return (130 - pwm) / (130 - 47) * 50
    
    def _pwm_to_adc(self, pwm):
        """Convert PWM value to simulated ADC reading"""
        # PWM 130 (closed) -> ADC 651
        # PWM 47 (open) -> ADC 72
        ratio = (pwm - 47) / (130 - 47)
        return int(self.adc_min + ratio * (self.adc_max - self.adc_min))
    
    def _report_positions(self):
        """Generate position report string"""
        with self._lock:
            positions_mm = []
            for i in range(5):
                # Simulate ADC reading based on current servo position
                adc_value = self._pwm_to_adc(self.servo_positions[i])
                # Map ADC to mm (inverse mapping as in Arduino)
                position_mm = self._map_value(adc_value, self.adc_max, self.adc_min, 0, 50)
                positions_mm.append(position_mm)
            
            report = f"F1:{positions_mm[0]:.2f},F2:{positions_mm[1]:.2f}," \
                    f"F3:{positions_mm[2]:.2f},F4:{positions_mm[3]:.2f}," \
                    f"F5:{positions_mm[4]:.2f}"
            return report
    
    def _map_value(self, x, in_min, in_max, out_min, out_max):
        """Arduino map function equivalent"""
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def _constrain(self, val, min_val, max_val):
        """Arduino constrain function equivalent"""
        return max(min_val, min(max_val, val))
    
    def _process_command(self, command):
        """Process a single command"""
        command = command.strip()
        
        # Position commands
        for i in range(1, 6):
            if command.startswith(f"P{i}"):
                try:
                    pwm = int(command[2:])
                    pwm = self._constrain(pwm, 47, 130)
                    with self._lock:
                        self.target_positions[i-1] = pwm
                    self._write_output(f"ACK:P{i}:{pwm}")
                    return
                except ValueError:
                    self._write_output(f"ERR:INVALID_PWM:{command}")
                    return
        
        # Other commands
        if command == "START_REPORT":
            self.continuous_reporting = True
            self._write_output("ACK:START_REPORT")
        elif command == "STOP_REPORT":
            self.continuous_reporting = False
            self._write_output("ACK:STOP_REPORT")
        elif command == "GET_POS":
            self._write_output(self._report_positions())
        elif command == "STATUS":
            with self._lock:
                status = f"STATUS:REPORTING={'ON' if self.continuous_reporting else 'OFF'}," \
                        f"T1={self.target_positions[0]},T2={self.target_positions[1]}," \
                        f"T3={self.target_positions[2]},T4={self.target_positions[3]}," \
                        f"T5={self.target_positions[4]}"
            self._write_output(status)
        else:
            self._write_output(f"ERR:UNKNOWN_CMD:{command}")
    
    def _arduino_loop(self):
        """Main Arduino loop simulation"""
        while self.running:
            # Update servo positions (simulate movement)
            self._update_servo_positions()
            
            # Process incoming commands
            try:
                command = self.input_buffer.get_nowait()
                self._process_command(command)
            except queue.Empty:
                pass
            
            # Handle continuous reporting
            if self.continuous_reporting:
                current_time = time.time()
                if current_time - self.last_report_time >= self.report_interval:
                    self._write_output(self._report_positions())
                    self.last_report_time = current_time
            
            # Small delay to prevent CPU spinning
            time.sleep(0.001)


class MockSerial:
    """Mock serial.Serial class that uses ArduinoMock"""
    
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._mock = ArduinoMock()
    
    def write(self, data):
        return self._mock.write(data)
    
    def readline(self):
        return self._mock.readline()
    
    def flush(self):
        return self._mock.flush()
    
    def reset_input_buffer(self):
        return self._mock.reset_input_buffer()
    
    def reset_output_buffer(self):
        return self._mock.reset_output_buffer()
    
    def close(self):
        return self._mock.close()
    
    @property
    def in_waiting(self):
        return self._mock.in_waiting()
    
    @property
    def is_open(self):
        return self._mock.is_open