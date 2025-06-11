import tkinter as tk
from tkinter import ttk
from comm_backend import CommandSender

class ExoControlGUI:
    """Handle GUI and sending commands to exoskeleton controller"""
    def __init__(self):
        # Create the main window
        self.window = tk.Tk()
        self.window.title("Exoskeleton Control")
        self.window.geometry("600x500")
        
        # Create the command sender (using mock Arduino)
        self.controller = CommandSender(use_mock=True)
        self.is_connected = False
        
        # Build the GUI
        self.create_gui()
        
    def create_gui(self):
        """Create all the GUI elements"""
        
        # Title Label
        title = tk.Label(self.window, text="Exoskeleton Finger Controller", 
                        font=("Arial", 16, "bold"))
        title.pack(pady=10)
        
        # Connection Section
        self.create_connection_section()
        
        # Position Display Section
        self.create_position_display()
        
        # Control Buttons Section
        self.create_control_buttons()
        
        # Status Label at the bottom
        self.status_label = tk.Label(self.window, text="Not connected", 
                                   bg="lightgray", relief=tk.SUNKEN)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)
        
    def create_connection_section(self):
        """Create the connection controls"""
        # Frame to hold connection controls
        conn_frame = tk.Frame(self.window, relief=tk.RAISED, borderwidth=2)
        conn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Label
        tk.Label(conn_frame, text="Connection:").pack(side=tk.LEFT, padx=10)
        
        # Port dropdown
        self.port_dropdown = ttk.Combobox(conn_frame, width=15)
        self.port_dropdown.pack(side=tk.LEFT, padx=5)
        
        # Get available ports and add to dropdown
        ports = self.controller.get_available_ports()
        self.port_dropdown['values'] = ports
        if ports:
            self.port_dropdown.set(ports[0])  # Select first port
            
        # Connect button
        self.connect_button = tk.Button(conn_frame, text="Connect", 
                                      command=self.connect_clicked,
                                      bg="green", fg="white", padx=20)
        self.connect_button.pack(side=tk.LEFT, padx=10, pady=5)
        
    def create_position_display(self):
        """Create labels to show finger positions"""
        # Frame for position display
        pos_frame = tk.LabelFrame(self.window, text="Actuator Positions (mm)")
        pos_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Create a label for each finger
        self.position_labels = []
        finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        
        for finger in finger_names:
            # Create a frame for each finger
            finger_frame = tk.Frame(pos_frame)
            finger_frame.pack(side=tk.LEFT, padx=10, pady=5)
            
            # Finger name
            tk.Label(finger_frame, text=finger).pack()
            
            # Position value
            pos_label = tk.Label(finger_frame, text="0.0", 
                               font=("Arial", 14), bg="white", width=6)
            pos_label.pack()
            
            # Store the label so we can update it later
            self.position_labels.append(pos_label)
            
    def create_control_buttons(self):
        """Create buttons to control the hand"""
        # Frame for control buttons
        control_frame = tk.LabelFrame(self.window, text="Hand Controls")
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Open hand button
        self.open_button = tk.Button(control_frame, text="Open Hand", 
                                   command=self.open_hand,
                                   bg="blue", fg="white", 
                                   font=("Arial", 12), padx=20, pady=10,
                                   state=tk.DISABLED)
        self.open_button.pack(side=tk.LEFT, padx=10, pady=10)
        
        # Close hand button
        self.close_button = tk.Button(control_frame, text="Close Hand", 
                                    command=self.close_hand,
                                    bg="red", fg="white",
                                    font=("Arial", 12), padx=20, pady=10,
                                    state=tk.DISABLED)
        self.close_button.pack(side=tk.LEFT, padx=10, pady=10)
        
        # OK sign button
        self.ok_button = tk.Button(control_frame, text="OK Sign", 
                                 command=self.make_ok_sign,
                                 bg="purple", fg="white",
                                 font=("Arial", 12), padx=20, pady=10,
                                 state=tk.DISABLED)
        self.ok_button.pack(side=tk.LEFT, padx=10, pady=10)
        
        # Update positions button
        self.update_button = tk.Button(control_frame, text="Update Positions", 
                                     command=self.update_positions,
                                     font=("Arial", 10), padx=10, pady=5,
                                     state=tk.DISABLED)
        self.update_button.pack(side=tk.RIGHT, padx=10, pady=10)
        
    def connect_clicked(self):
        """Handle the connect button click"""
        if not self.is_connected:
            # Try to connect
            port = self.port_dropdown.get()
            
            self.update_status("Connecting...")
            
            if self.controller.connect(port=port):
                # Connection successful
                self.is_connected = True
                self.connect_button.config(text="Disconnect", bg="red")
                self.enable_controls(True)
                self.update_status("Connected!")
                
                # Get initial positions
                self.update_positions()
            else:
                # Connection failed
                self.update_status("Connection failed!")
        else:
            # Disconnect
            self.controller.disconnect()
            self.is_connected = False
            self.connect_button.config(text="Connect", bg="green")
            self.enable_controls(False)
            self.update_status("Disconnected")
            
    def enable_controls(self, enabled):
        """Enable or disable the control buttons"""
        state = tk.NORMAL if enabled else tk.DISABLED
        
        self.open_button.config(state=state)
        self.close_button.config(state=state)
        self.ok_button.config(state=state)
        self.update_button.config(state=state)
        
    def update_status(self, message):
        """Update the status label"""
        self.status_label.config(text=message)
        self.window.update()  # Force GUI update
        
    def update_positions(self):
        """Update the position display"""
        if self.is_connected:
            # Get current positions from the controller
            positions = self.controller.get_current_positions()
            
            # Update each label
            for i, pos_label in enumerate(self.position_labels):
                pos_label.config(text=f"{positions[i]:.1f}")
                
            self.update_status("Positions updated")
            
    def open_hand(self):
        """Open all fingers to 100%"""
        self.update_status("Opening hand...")
        
        # Send command to open all fingers (100% = fully open)
        self.controller.send_all_positions([100, 100, 100, 100, 100])
        
        # Wait a bit for movement
        self.window.after(500, self.update_positions) 
        
    def close_hand(self):
        """Close all fingers to 0%"""
        self.update_status("Closing hand...")
        
        # Send command to close all fingers (0% = fully closed)
        self.controller.send_all_positions([0, 0, 0, 0, 0])
        
        # Wait a bit for movement
        self.window.after(500, self.update_positions) 
        
    def make_ok_sign(self):
        """Make the OK sign gesture"""
        self.update_status("Making OK sign...")
        
        # OK sign: thumb and index partially closed, others more open
        # [Thumb, Index, Middle, Ring, Pinky]
        self.controller.send_all_positions([30, 30, 80, 85, 90])
        
        # Wait a bit for movement
        self.window.after(500, self.update_positions) 
        
    def run(self):
        """Start the GUI"""
        self.window.mainloop()


# Main program
if __name__ == "__main__":
    gui = ExoControlGUI()
    gui.run()