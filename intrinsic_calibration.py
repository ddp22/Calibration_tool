import cv2
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk
import os
import numpy as np
import yaml
from datetime import datetime
from utils.rosbag_utils import RosbagUtils
from utils.tkinter_utils import TkinterUtils
import webbrowser

class IntrinsicCameraCalibrationExtractor:
    def __init__(self):

        self._cap = None
        self._total_frames = 0
        self._current_frame_index = 0

        # Setup GUI
        self._root = TkinterUtils.initialize_tkinter("Intrinsic Camera Calibration")

        self._window_width = 1280
        self._window_height = 720

        # Load pattern size and square size from general_configuration.yaml
        config_path = os.path.join(os.path.dirname(__file__), 'config', 'general_configuration.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self._pattern_size = tuple(config['chessboard']['pattern_size'].values())  # Number of inner corners per chessboard row and column
        self._square_size = config['chessboard']['square_size_meters']


        camera_config = config.get('camera', {})
        self._camera_topic = camera_config.get('image_topic', '/camera/compressed')
        camera_size = camera_config.get('image_size', {})
        self._camera_width = camera_size.get('width', None)
        self._camera_height = camera_size.get('height', None)

        self._enable_chessboard_detection = False

        self._obj_points = []
        self._img_points = []
        self._total_valid_images = 0
        self._actual_refined_corners = None
        self._objp = np.zeros((self._pattern_size[1] * self._pattern_size[0], 3), np.float32)
        self._objp[:, :2] = np.mgrid[0:self._pattern_size[0], 0:self._pattern_size[1]].T.reshape(-1, 2)
        self._objp *= self._square_size

        # To avoid saving multiple times the same frame
        self._current_video_name = None
        self._saved_frames_map = {}

        self._current_fps = 1  # Default FPS


    def _create_skip_text(self, master):
        inputtxt = TkinterUtils.create_text(master, height=1, width=10, side="left", padx=(10, 0), pady=10)
        # Add a validation to allow only numbers and delete letters
        def validate_input(event):
            content = inputtxt.get("1.0", "end-1c")  # Get the content of the Text widget
            filtered_content = ''.join(filter(str.isdigit, content))  # Keep only numeric characters
            
            # Ensure the first digit is not 0 if the number has 2 or more digits
            if len(filtered_content) > 1 and filtered_content[0] == '0':
                filtered_content = filtered_content.lstrip('0')  # Remove leading zeros
            
            if content != filtered_content:  # If there were invalid characters or leading zeros
                inputtxt.delete("1.0", "end")  # Clear the content
                inputtxt.insert("1.0", filtered_content)  # Insert the filtered content

            # Check if the number is greater than total frames
            if filtered_content.isdigit() and int(filtered_content) > self._total_frames:
                inputtxt.config(fg="red")  # Set text color to red
            else:
                inputtxt.config(fg="black")  # Set text color to black
        inputtxt.bind("<KeyRelease>", validate_input)
        inputtxt.insert("1.0", "0")  # Default value
        return inputtxt

    
    def _add_to_frame_index(self, value):
        new_index = self._current_frame_index + value
        if value < 0 and new_index < 0:
            new_index = 0
        elif value > 0 and new_index >= self._total_frames:
            new_index = self._total_frames - 1
        self._update_frame_index(new_index)

    def _update_frame_index(self, new_index):
        if 0 <= new_index < self._total_frames:
            self._current_frame_index = new_index
            self._show_frame(self._current_frame_index)
            self.inputtxt.delete("1.0", "end")
            self.inputtxt.insert("1.0", str(self._current_frame_index + 1))
            self.inputtxt.config(fg="black")  # Reset text color to black

    def _find_chessboard(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self._pattern_size, None)
        if not ret:
            # Second attempt with different preprocessing
            _, gray = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
            gray = cv2.bitwise_not(gray)
            ret, corners = cv2.findChessboardCorners(gray, self._pattern_size, None)
        if ret:
            cv2.drawChessboardCorners(frame, self._pattern_size, corners, ret)
            refined_corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            self._actual_refined_corners = refined_corners
        else:
            self._actual_refined_corners = None
        return frame, ret

    def _save_current_corners(self):
        if self._actual_refined_corners is not None:
            if self._current_video_name not in self._saved_frames_map:
                self._saved_frames_map[self._current_video_name] = set()
            if self._current_frame_index in self._saved_frames_map[self._current_video_name]:
                print(f"Frame {self._current_frame_index + 1} from video '{os.path.basename(self._current_video_name)}' has already been saved.")
                return  # Frame already saved, do nothing
            self._saved_frames_map[self._current_video_name].add(self._current_frame_index)
            self._obj_points.append(self._objp)
            self._img_points.append(self._actual_refined_corners)
            self._total_valid_images += 1
            print(f"Saved corners for frame {self._current_frame_index + 1}. Total valid images: {self._total_valid_images}")
        else:
            print(f"No corners found for frame {self._current_frame_index + 1}")

    def skip_to_frame(self):
        input_text = self.inputtxt.get("1.0", "end-1c")
        if len(input_text) == 0:
            return  # No input to process
        frame_index = int(input_text) - 1
        self._update_frame_index(frame_index)

    def _show_frame(self, frame_index):
        if self._cap is None:
            return  # No video loaded
        # Set the video position to the desired frame
        self._cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ret, frame = self._cap.read()
        if not ret:
            print(f"Unable to read frame {frame_index}")
            return

        # OpenCV reads in BGR, convert to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if self._enable_chessboard_detection:
            frame, found = self._find_chessboard(frame)
            if found:
                self._root.title(f"Intrinsic Camera Calibration - Chessboard Detected in Frame {frame_index + 1}")
            else:
                self._root.title(f"Intrinsic Camera Calibration - No Chessboard in Frame {frame_index + 1}")

        # Resize the frame to fit the window
        frame = cv2.resize(frame, (self._window_width, self._window_height), interpolation=cv2.INTER_AREA)

        # Convert the frame to a format compatible with Tkinter
        image = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image)

        # Update the Label
        self._image_label.configure(image=imgtk)
        self._image_label.image = imgtk

    def _show_next_frame(self):
        self._add_to_frame_index(1)

    def _show_prev_frame(self):
        self._add_to_frame_index(-1)

    def _open_video(self):
        video_path = filedialog.askopenfilename(
            title="Select a video",
            filetypes=[("Video files", "*.mp4 *.avi *.mov *.mkv"), ("All files", "*.*")]
        )
        if not video_path:
            return  # the user canceled

        # Try to open the video
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            messagebox.showerror("Error", f"Cannot open the video: {video_path}")
            return

        
        # Get video properties
        camera_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        camera_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if camera_width != self._camera_width or camera_height != self._camera_height:
            raise ValueError(f"New video resolution {camera_width}x{camera_height} does not match previous video resolution {self._camera_width}x{self._camera_height}. Please use videos with the same resolution.")

        self._current_video_name = video_path
        self._current_fps = cap.get(cv2.CAP_PROP_FPS)

        # If there was already an open video, release resources
        if self._cap is not None:
            self._cap.release()

        self._cap = cap
        self._total_frames = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self._current_frame_index = 0

        # Update the frame label
        self.frame_label.config(text=f"/{self._total_frames}")
        self.inputtxt.delete("1.0", "end")
        self.inputtxt.insert("1.0", "1")

        # Show the first frame
        self._show_frame(self._current_frame_index)

    def _calibrate_camera(self):
        if self._total_valid_images < 20:
            message = "Not enough valid images for calibration. At least 20 are required. Please save almost other "+ str(20 - self._total_valid_images) + " valid images."
            messagebox.showerror("Error", message)
            return

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self._obj_points,
            self._img_points,
            (self._camera_width, self._camera_height),
            None,
            None
        )

        calibration_data = {
            'calibration_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'camera_matrix': {
                'rows': 3,
                'columns': 3,
                'data': camera_matrix.tolist()
            },
            'distortion_coefficients': {
                'rows': 1,
                'columns': len(dist_coeffs[0]),
                'data': dist_coeffs[0].tolist()
            },
            'chessboard': {
                'pattern_size': {
                    'rows': self._pattern_size[1],
                    'columns': self._pattern_size[0]
                },
                'square_size_meters': self._square_size
            },
            'image_size': {
                'width': self._camera_width,
                'height': self._camera_height
            },
            'rms_reprojection_error': ret
        }

        config_dir = os.path.join(os.path.dirname(__file__), 'config')
        os.makedirs(config_dir, exist_ok=True)
        calibration_file = os.path.join(config_dir, 'camera_intrinsic_calibration.yaml')
        with open(calibration_file, 'w') as f:
            yaml.dump(calibration_data, f)

        messagebox.showinfo("Success", f"Calibration successful! Data saved to {calibration_file}")

    def _enable_chessboard_detection_toggle(self):
        self._enable_chessboard_detection = not self._enable_chessboard_detection
        self._root.title("Intrinsic Camera Calibration")  # Reset title
        self._show_frame(self._current_frame_index)  # Refresh the current frame to show/hide chessboard detection

    def _extract_video_from_bag(self):
        bag_folder_path = filedialog.askdirectory(title="Select a ROS2 bag folder")
        if not bag_folder_path:
            return  # the user canceled
        bag_name = os.path.basename(bag_folder_path)
        output_video_path = str(os.path.join(os.path.dirname(__file__), 'videos', f"{bag_name}.mp4"))
        os.makedirs(os.path.join(os.path.dirname(__file__), 'videos'), exist_ok=True)
        # Show a waiting message while extracting the video
        messagebox.showinfo("Info", f"Extracting video from bag. This may take a while...\nClick OK to continue.")
        results = RosbagUtils.topic_to_video(bag_folder_path, self._camera_topic, output_video_path, self._camera_width, self._camera_height)
        if results:
            messagebox.showinfo("Success", f"Video extracted successfully to {output_video_path}")
        else:
            messagebox.showerror("Error", f"Failed to extract video from bag.")

    def run(self):
        btn_top_frame = TkinterUtils.create_frame(self._root, "top")
        center_tool_frame = TkinterUtils.create_frame(self._root, "top")
        image_frame = TkinterUtils.create_frame(center_tool_frame, "left")
        image_frame.pack(side="left", fill="both", expand=True)

        self._legend_frame = TkinterUtils.create_frame(center_tool_frame, "top")

        btn_button_frame = TkinterUtils.create_frame(self._root, "top")

        # Button for extracting video from ROS2 bag
        self.btn_extract = TkinterUtils.create_button("Create video from ROS2 bag", self._extract_video_from_bag, "left", 10, 10, btn_top_frame)

        # Button for opening a new video
        self.btn_open = TkinterUtils.create_button("Open video", self._open_video, "left", 10, 10, btn_top_frame)

        readme_path = "https://github.com/ddp22/Calibration_tool/blob/main/README.md"
        self._btn_tutorial = TkinterUtils.create_button("Help", lambda: webbrowser.open(readme_path), "right", 10, 10, btn_top_frame)

        black_image = Image.new("RGB", (1280, 720), "black")
        black_imgtk = ImageTk.PhotoImage(black_image)
        self._image_label = TkinterUtils.create_label("", "left", 10, 0, image_frame)
        self._image_label.configure(image=black_imgtk)


        # Button for previous second
        TkinterUtils.create_button("<< Previous second", lambda: self._add_to_frame_index(-int(self._current_fps)), "left", 10, 10, btn_button_frame)
        # Button for previous frame
        TkinterUtils.create_button("<< Previous", self._show_prev_frame, "left", 10, 10, btn_button_frame)

        # Button and text for skipping to a specific frame
        TkinterUtils.create_button("Skip to frame:", self.skip_to_frame, "left", (50, 0), 10, btn_button_frame)
        self.inputtxt = self._create_skip_text(btn_button_frame)
        self.frame_label = TkinterUtils.create_label("/0", "left", 1, 1, btn_button_frame)

        # Check box for enabling/disabling chessboard detection
        self.chessboard_var = TkinterUtils.create_check_box("Enable Chessboard Detection",
                                                     function=self._enable_chessboard_detection_toggle,
                                                     side="left", padx=(50, 0), pady=10, master=btn_button_frame)
        self.chessboard_var.deselect()  # Default is disabled

        self.btn_save_corners = TkinterUtils.create_button("Save Corners", self._save_current_corners, "left", 10, 10, btn_button_frame)

        self.btn_calibrate = TkinterUtils.create_button("Calibrate", self._calibrate_camera, "left", 10, 10, btn_button_frame)

        # Button for next second
        self.btn_next_second = TkinterUtils.create_button("Next second >>", lambda: self._add_to_frame_index(int(self._current_fps)), "right", 10, 10, btn_button_frame)

        # Button for next frame
        self.btn_next = TkinterUtils.create_button("Next >>", self._show_next_frame, "right", 10, 10, btn_button_frame)

        self._root.mainloop()

        return {"video_width": self._camera_width, "video_height": self._camera_height,
                "pattern_size": self._pattern_size, "square_size": self._square_size}
        