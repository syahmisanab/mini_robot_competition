# gui_tkinter.py

import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import numpy as np
import logging
from multiprocessing import Process, Queue, Value
import time

logger = logging.getLogger(__name__)

class RobotTkinterGUI:
    # === PEMBETULAN: Tambah 'config' sebagai argumen ===
    def __init__(self, data_queue, robot_state_value, terminate_flag, config): 
        self.data_queue = data_queue
        self.robot_state_value = robot_state_value
        self.terminate_flag = terminate_flag
        self.config = config # Simpan config
        self.ROBOT_STATES = config['ROBOT_STATES'] # Dapatkan ROBOT_STATES dari config

        self.root = tk.Tk()
        self.root.title("Robot Control Panel")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        # Frame untuk video
        self.video_frame = ttk.Label(self.root)
        self.video_frame.grid(row=0, column=0, padx=10, pady=10, rowspan=6)

        # Frame untuk kawalan dan status
        self.control_frame = ttk.Frame(self.root, padding="10")
        self.control_frame.grid(row=0, column=1, padx=10, pady=10, sticky="n")

        # --- Elemen Status ---
        ttk.Label(self.control_frame, text="Robot Status:").grid(row=0, column=0, sticky="w")
        self.status_label = ttk.Label(self.control_frame, text="INITIALIZING", foreground="orange")
        self.status_label.grid(row=0, column=1, sticky="w")

        ttk.Label(self.control_frame, text="Current Mode:").grid(row=1, column=0, sticky="w")
        self.mode_label = ttk.Label(self.control_frame, text="N/A")
        self.mode_label.grid(row=1, column=1, sticky="w")

        ttk.Label(self.control_frame, text="Line Error:").grid(row=2, column=0, sticky="w")
        self.line_error_label = ttk.Label(self.control_frame, text="0.00")
        self.line_error_label.grid(row=2, column=1, sticky="w")

        ttk.Label(self.control_frame, text="Motor Speeds (L/R):").grid(row=3, column=0, sticky="w")
        self.motor_speed_label = ttk.Label(self.control_frame, text="0.00 / 0.00")
        self.motor_speed_label.grid(row=3, column=1, sticky="w")

        ttk.Label(self.control_frame, text="Victims Rescued:").grid(row=4, column=0, sticky="w")
        self.victims_rescued_label = ttk.Label(self.control_frame, text="0")
        self.victims_rescued_label.grid(row=4, column=1, sticky="w")

        # --- Butang Kawalan ---
        ttk.Button(self.control_frame, text="Start Line Following", command=self._start_line_following).grid(row=5, column=0, columnspan=2, pady=5)
        ttk.Button(self.control_frame, text="Initiate Victim Rescue", command=self._initiate_victim_rescue).grid(row=6, column=0, columnspan=2, pady=5)
        ttk.Button(self.control_frame, text="Initiate Zone Exit", command=self._initiate_zone_exit).grid(row=7, column=0, columnspan=2, pady=5)
        ttk.Button(self.control_frame, text="STOP ROBOT", command=self._stop_robot).grid(row=8, column=0, columnspan=2, pady=10, ipadx=20, ipady=10)


        # Mulakan kemas kini GUI
        self.root.after(100, self._update_gui)

    # === PEMBETULAN: Tambah 'config' sebagai argumen ===
    @staticmethod
    def run_gui(data_queue, robot_state_value, terminate_flag, config): 
        gui = RobotTkinterGUI(data_queue, robot_state_value, terminate_flag, config)
        gui.root.mainloop()

    def _update_gui(self):
        try:
            while not self.data_queue.empty():
                data = self.data_queue.get_nowait()
                
                if 'frame' in data and data['frame'] is not None:
                    img = Image.fromarray(cv2.cvtColor(data['frame'], cv2.COLOR_BGR2RGB))
                    img = ImageTk.PhotoImage(image=img)
                    self.video_frame.config(image=img)
                    self.video_frame.image = img
                
                if 'state' in data:
                    state = data['state']
                    if state == self.ROBOT_STATES['LINE_FOLLOWING']:
                        self.status_label.config(text="RUNNING", foreground="green")
                        self.mode_label.config(text="LINE FOLLOWING")
                    elif state == self.ROBOT_STATES['VICTIM_RESCUE']:
                        self.status_label.config(text="RUNNING", foreground="blue")
                        self.mode_label.config(text="VICTIM RESCUE")
                    elif state == self.ROBOT_STATES['ZONE_EXIT']:
                        self.status_label.config(text="RUNNING", foreground="purple")
                        self.mode_label.config(text="ZONE EXIT")
                    elif state == self.ROBOT_STATES['IDLE']:
                        self.status_label.config(text="IDLE", foreground="orange")
                        self.mode_label.config(text="IDLE")
                    elif state == self.ROBOT_STATES['FINISHED']:
                        self.status_label.config(text="FINISHED", foreground="black")
                        self.mode_label.config(text="FINISHED")
                    elif state == self.ROBOT_STATES['ERROR']:
                        self.status_label.config(text="ERROR!", foreground="red")
                        self.mode_label.config(text="ERROR")

                if 'line_error' in data:
                    self.line_error_label.config(text=f"{data['line_error']:.2f}")
                
                if 'motor_speeds' in data:
                    self.motor_speed_label.config(text=f"{data['motor_speeds'][0]:.2f} / {data['motor_speeds'][1]:.2f}")
                
                if 'victims_rescued' in data:
                    self.victims_rescued_label.config(text=str(data['victims_rescued']))

        except Exception as e:
            logger.error(f"GUI: Error updating GUI: {e}")
        finally:
            self.root.after(100, self._update_gui)

    def _start_line_following(self):
        logger.info("GUI: 'Start Line Following' button pressed. Setting state to LINE_FOLLOWING.")
        self.robot_state_value.value = self.ROBOT_STATES['LINE_FOLLOWING']

    def _initiate_victim_rescue(self):
        logger.info("GUI: 'Initiate Victim Rescue' button pressed. Setting state to VICTIM_RESCUE.")
        self.robot_state_value.value = self.ROBOT_STATES['VICTIM_RESCUE']

    def _initiate_zone_exit(self):
        logger.info("GUI: 'Initiate Zone Exit' button pressed. Setting state to ZONE_EXIT.")
        self.robot_state_value.value = self.ROBOT_STATES['ZONE_EXIT']

    def _stop_robot(self):
        logger.info("GUI: 'STOP ROBOT' button pressed. Setting state to FINISHED (or ERROR).")
        self.robot_state_value.value = self.ROBOT_STATES['FINISHED']
        self.terminate_flag.value = True
        self.root.quit()

    def _on_closing(self):
        logger.info("GUI window closing. Signalling termination.")
        self.terminate_flag.value = True
        self.root.quit()