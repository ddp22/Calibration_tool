from PySide6.QtWidgets import QApplication, QWidget, QFileDialog, QLabel
from PySide6.QtGui import QPixmap, QImage, QPainter
from utils.pyside_utils import PySideUtils
import cv2
from utils.rosbag_utils import RosbagUtils
import os
import yaml
from PySide6.QtCore import Qt
import open3d as o3d
import numpy as np

from PySide6 import  QtWidgets
from tqdm import tqdm
import webbrowser


class ExtrinsicCameraCalibrationExtractor:
    def __init__(self):

        self._n_skipped_frames = 0
        config_path = os.path.join(os.path.dirname(__file__), 'config', 'general_configuration.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        camera_config = config.get('camera', {})
        self._camera_topic = camera_config.get('image_topic')
        camera_size = camera_config.get('image_size', {})
        self._camera_width = camera_size.get('width')
        self._camera_height = camera_size.get('height')

        lidar_config = config.get('lidar', {})
        self._lidar_topic = lidar_config.get('lidar_topic')

        self._image_label_size = (1280, 720)  # default size

        self._scale_x = self._image_label_size[0] / self._camera_width
        self._scale_y = self._image_label_size[1] / self._camera_height

        # Setup GUI
        self._app = QApplication([])
        self._window = QWidget()
        self._window.setWindowTitle("Extrinsic Camera Calibration")
        self._extra_space_for_buttons = 80
        self._default_pad_x = 20
        self._window_size = (self._image_label_size[0], self._image_label_size[1] + 2 * self._extra_space_for_buttons)  # extra space for buttons
        print(f"Window size: {self._window_size}")
        self._window.resize(*self._window_size)
        self._window.setFixedSize(*self._window_size)
        self._window.show()

        self._current_pixmap = None
        self._current_2d_points = []
        self._current_3d_points = []
        self._point_size = 3

        self._pyside_utils = PySideUtils()
        self._rosbag_utils = RosbagUtils()

        self._current_pcd = None

        output_dir = os.path.join(os.path.dirname(__file__), 'data')
        os.makedirs(output_dir, exist_ok=True)
        self._out_txt = os.path.join(output_dir, 'extracted_points.txt')
        with open(self._out_txt, 'a') as f:
            f.write("# u, v, x, y, z\n")


    def _select_pcd_points(self, pcd):
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Select points on the cloud", width=1280, height=720)
        vis.add_geometry(pcd)

        ctr = vis.get_view_control()
        ctr.set_lookat(np.array([0.0, 0.0, 0.0]))
        ctr.set_front(np.array([-1.0, 0.0, 0.0]))
        ctr.set_up(np.array([0.0, 0.0, 1.0]))
        ctr.set_zoom(.05)
        ctr.camera_local_translate(0.0, 0.0, 1.0)

        render_opt = vis.get_render_option()
        render_opt.point_size = 5.0

        vis.run()
        vis.destroy_window()
        picked_indices = vis.get_picked_points()
        
        return picked_indices

    def _delete_nan_points(self, points_np):
        mask = ~np.isnan(points_np).any(axis=1)
        cleaned_points = points_np[mask]
        return cleaned_points

    def _choose_3d_points(self):
        self._current_3d_points.clear()
        if self._current_pcd is None:
            print("No lidar points available")
            return

        if self._current_pcd.is_empty():
            print("Lidar point cloud is empty")
            return
        

        picked_indices = self._select_pcd_points(self._current_pcd)
        if not picked_indices:
            print("No points were selected.")
            return

        self._current_3d_points = [self._current_pcd.points[i] for i in picked_indices]
        print(f"Selected {len(self._current_3d_points)} 3D points.")

        if len(self._current_2d_points) != len(self._current_3d_points):
            print("The number of 2D and 3D points must be the same.")
            return

    def _next_frame(self):
        self._reset_selection()
        self._show_frame()
        
        

    def _save_points(self):
        with open(self._out_txt, 'a') as f:
            min_len = min(len(self._current_2d_points), len(self._current_3d_points))
            for i in range(min_len):
                (u, v) = self._current_2d_points[i]
                (x, y, z) = self._current_3d_points[i]
                f.write(f"{u},{v},{x},{y},{z}\n")

    def _process_frame(self):
        camera_info, lidar_info = self._rosbag_utils.get_next_synced_messages()
        if camera_info is None or lidar_info is None:
            return
        frame, timestamp = camera_info

        if frame is None:
            print("Failed to decode image")
            return


        lidar_points_np, lidar_timestamp = lidar_info
        lidar_points_np = self._delete_nan_points(lidar_points_np)
        pcd = o3d.geometry.PointCloud()
        print(f"Number of lidar points: {len(lidar_points_np)}")
        pcd.points = o3d.utility.Vector3dVector(lidar_points_np)
        print(f"Lidar points timestamp: {lidar_timestamp}, Camera frame timestamp: {timestamp}")
        if pcd.is_empty():
            print("Lidar point cloud is empty")
            return

        self._current_pcd = pcd

        # OpenCV reads in BGR, convert to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame

    def _reset_selection(self):
        self._current_2d_points.clear()
        self._current_3d_points.clear()
        self._current_pcd = None

    def _show_frame(self):
        frame = self._process_frame()
        if frame is None:
            print("No more frames available or failed to process frame.")
            return
        # Resize the frame to fit the window
        frame = cv2.resize(frame, (self._image_label_size[0], self._image_label_size[1]), interpolation=cv2.INTER_AREA)

        # self._image_label.setGeometry(0, 0, self._image_size[0], self._image_size[1])
        pixmap = QPixmap(frame.shape[1], frame.shape[0])
        pixmap.convertFromImage(QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_RGB888))
        self._image_label.setPixmap(pixmap)
        self._current_pixmap = pixmap
        self._current_2d_points.clear()
        self._image_label.show()
        print("Displayed new frame")

    def _open_rosbag(self):
        rosbag_path = QFileDialog.getExistingDirectory(self._window, "Select ROS Bag Directory", "")
        if not rosbag_path:
            return  # the user canceled

        self._current_video_name = rosbag_path

        self._rosbag_utils.init_lidar_camera_sync(rosbag_path, self._camera_topic, self._lidar_topic)

        # Show the first frame
        self._show_frame()

    def _add_point_on_image(self, event):
        # Get the position of the click
        x = int(event.position().x())
        y = int(event.position().y())
        x_scaled = int(x / self._scale_x)
        y_scaled = int(y / self._scale_y)
        print(f"Clicked at: ({x_scaled}, {y_scaled})")
        self._current_2d_points.append((x_scaled, y_scaled))
        self._update_image_with_points()

    def _delete_last_point_on_image(self):
        if not self._current_2d_points:
            return
        x, y = self._current_2d_points.pop()
        print(f"Deleted point: ({x}, {y})")
        # Update the image with the new points
        self._update_image_with_points()

    def _update_image_with_points(self):
        if self._current_pixmap is None:
            return

        # Create a copy of the current pixmap to draw on
        pixmap_copy = QPixmap(self._current_pixmap)
        painter = QPainter(pixmap_copy)
        painter.setPen(Qt.red)
        painter.setBrush(Qt.red)

        i = 1
        for point in self._current_2d_points:
            x = int(point[0] * self._scale_x)
            y = int(point[1] * self._scale_y)
            painter.drawEllipse(x - self._point_size, y - self._point_size, self._point_size * 2, self._point_size * 2)
            painter.drawText(x + self._point_size, y - self._point_size, str(i))
            i += 1
        painter.end()
        # Update the label with the new image
        self._image_label.setPixmap(pixmap_copy)

    def _load_camera_calibration(self, yaml_path: str):
        """Loads camera calibration parameters from a YAML file."""
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"Calibration file not found: {yaml_path}")

        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        mat_data = config['camera_matrix']['data']
        camera_matrix = np.array(mat_data, dtype=np.float64)
        dist_data = config['distortion_coefficients']['data']
        dist_coeffs = np.array(dist_data, dtype=np.float64).reshape((1, -1))

        return camera_matrix, dist_coeffs
    
    def _load_extrinsic_calibration(self, yaml_path: str):
        """Loads extrinsic calibration parameters from a YAML file."""
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"Extrinsic calibration file not found: {yaml_path}")

        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        rvec_data = config.get('rvec')
        tvec_data = config.get('tvec')
        R_data = config.get('R')
        extrinsic_matrix_data = config.get('extrinsic_matrix')

        rvec = np.array(rvec_data, dtype=np.float64).reshape((3, 1)) if rvec_data else None
        tvec = np.array(tvec_data, dtype=np.float64).reshape((3, 1)) if tvec_data else None
        R = np.array(R_data, dtype=np.float64) if R_data else None
        extrinsic_matrix = np.array(extrinsic_matrix_data, dtype=np.float64) if extrinsic_matrix_data else None

        return rvec, tvec, R, extrinsic_matrix

    def _calibrate(self):
        """Solves for extrinsic parameters using 2D-3D correspondences and camera calibration."""
        camera_matrix, dist_coeffs = self._load_camera_calibration(os.path.join(os.path.dirname(__file__), 'config', 'camera_intrinsic_calibration.yaml'))
        if camera_matrix is None or dist_coeffs is None:
            print("Camera calibration data is not available.")
            return
        print(f"Camera matrix:\n{camera_matrix}")
        print(f"Distortion coefficients: {dist_coeffs}")

        if not os.path.isfile(self._out_txt):
            print(f"Correspondence file not found: {self._out_txt}")
            return

        pts_2d = []
        pts_3d = []
        with open(self._out_txt, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                splitted = line.split(',')
                if len(splitted) != 5:
                    continue
                u, v, X, Y, Z = [float(val) for val in splitted]
                pts_2d.append([u, v])
                pts_3d.append([X, Y, Z])

        pts_2d = np.array(pts_2d, dtype=np.float64)
        pts_3d = np.array(pts_3d, dtype=np.float64)

        num_points = len(pts_2d)
        print(f"Loaded {num_points} correspondences from {self._out_txt}")

        if num_points < 20:
            raise ValueError("At least 20 correspondences are required for solvePnP")

        success, rvec, tvec = cv2.solvePnP(
            pts_3d,
            pts_2d,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            raise RuntimeError("solvePnP failed to find a solution.")

        print("solvePnP succeeded.")
        print(f"rvec: {rvec.ravel()}")
        print(f"tvec: {tvec.ravel()}")

        R, _ = cv2.Rodrigues(rvec)

        T_lidar_to_cam = np.eye(4, dtype=np.float64)
        T_lidar_to_cam[0:3, 0:3] = R
        T_lidar_to_cam[0:3, 3] = tvec[:, 0]

        print(f"Transformation matrix (LiDAR -> Camera):\n{T_lidar_to_cam}")

        config_dir = os.path.join(os.path.dirname(__file__), 'config')
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)

        out_yaml = os.path.join(config_dir, "camera_extrinsic_calibration.yaml")
        data_out = {
            "rvec": rvec.tolist(),
            "tvec": tvec.tolist(),
            "R": R.tolist(),
            "extrinsic_matrix": T_lidar_to_cam.tolist()
        }

        with open(out_yaml, 'w') as f:
            yaml.dump(data_out, f, sort_keys=False)

        print(f"Extrinsic matrix saved to: {out_yaml}")

    def _clear_cache(self):
        with open(self._out_txt, 'w') as f:
            f.write("# u, v, x, y, z\n")
        print(f"Cleared all saved points in {self._out_txt}")

    def _project_lidar_to_image(self, points_3d, camera_matrix, dist_coeffs, T_lidar_to_cam):
        xyz_lidar_f64 = points_3d.astype(np.float64)
        ones = np.ones((xyz_lidar_f64.shape[0], 1), dtype=np.float64)
        xyz_lidar_h = np.hstack((xyz_lidar_f64, ones))

        xyz_cam_h = xyz_lidar_h @ T_lidar_to_cam.T
        xyz_cam = xyz_cam_h[:, :3]

        mask_in_front = (xyz_cam[:, 2] > 0.0)
        xyz_cam_front = xyz_cam[mask_in_front]
        n_front = xyz_cam_front.shape[0]
        if n_front == 0:
            print("No points in front of camera (z>0).")
            return
        
        rvec = np.zeros((3,1), dtype=np.float64)
        tvec = np.zeros((3,1), dtype=np.float64)
        image_points, _ = cv2.projectPoints(
            xyz_cam_front,
            rvec, tvec,
            camera_matrix,
            dist_coeffs
        )
        image_points = image_points.reshape(-1, 2)
        return image_points

    def _test(self):
        rosbag_path = QFileDialog.getExistingDirectory(self._window, "Select ROS Bag Directory that you want to test", "")
        if not rosbag_path:
            return  # the user canceled
        config_dir = os.path.join(os.path.dirname(__file__), 'config')
        if not os.path.exists(config_dir):
            print("Config directory does not exist.")
            return
        intrinsic_path = os.path.join(config_dir, 'camera_intrinsic_calibration.yaml')
        extrinsic_path = os.path.join(config_dir, 'camera_extrinsic_calibration.yaml')
        camera_matrix, dist_coeffs = self._load_camera_calibration(intrinsic_path)
        _, _, _, extrinsic_matrix = self._load_extrinsic_calibration(extrinsic_path)


        test_rosbag = RosbagUtils()
        test_rosbag.init_lidar_camera_sync(rosbag_path, self._camera_topic, self._lidar_topic)
        fps, count_sync, _ = RosbagUtils.get_fps_sync(rosbag_path, self._camera_topic, self._lidar_topic)
        print(f"FPS of the rosbag: {fps}")


        encoding = 'XVID'
        fourcc = cv2.VideoWriter_fourcc(*encoding)
        output_video_path = os.path.join(os.path.dirname(__file__), 'output_test.avi')
        print(f"Output video path: {output_video_path}")
        video_writer = cv2.VideoWriter(str(output_video_path), fourcc, fps, (self._camera_width, self._camera_height))
        
        with tqdm(total=count_sync, desc="Processing frames") as pbar:
            while True:
                camera_info, lidar_info = test_rosbag.get_next_synced_messages()
                if camera_info is None or lidar_info is None:
                    break
                frame, timestamp = camera_info

                if frame is None:
                    print("Failed to decode image")
                    continue

                lidar_points_np, _ = lidar_info
                lidar_points_np = self._delete_nan_points(lidar_points_np)
                if lidar_points_np.size == 0:
                    print("No valid lidar points")
                    continue

                projected_points = self._project_lidar_to_image(lidar_points_np, camera_matrix, dist_coeffs, extrinsic_matrix)
                if projected_points is None:
                    continue

                for point in projected_points:
                    u = int(point[0])
                    v = int(point[1])
                    if 0 <= u < self._camera_width and 0 <= v < self._camera_height:
                        cv2.circle(frame, (u, v), 2, (0, 255, 0), -1)

                pbar.update(1)
                # frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                video_writer.write(frame)

        video_writer.release()

    def _skip_frame(self):
        if self._n_skipped_frames <= 0:
            return
        n = self._n_skipped_frames
        self._rosbag_utils.skip_messages(n - 1)
        self._next_frame()

    def _on_text_changed(self, text):
        if not text.isdigit():
            self._number_input.setText(text[:-1])
            return

        if int(text[0]) <= 0:
            self._number_input.setText("")
            return
        
        self._n_skipped_frames = int(text)
        print(f"Set number of frames to skip: {self._n_skipped_frames}")
        self._skip_frame_button.setText(f"Skip {self._n_skipped_frames} Frames")

    def run(self):
        # add a black image
        self._image_label = QLabel(self._window)
        pixmap = QPixmap(self._image_label_size[0], self._image_label_size[1])
        pixmap.fill(Qt.black)
        self._image_label.setPixmap(pixmap)
        self._image_label.mousePressEvent = self._add_point_on_image

        # Position the image label lower in the window
        self._image_label.move(0, self._extra_space_for_buttons)  # Adjust the y-coordinate to move it down

        top_button_pos_y = 20
        bottom_button_pos_y = self._window_size[1] - self._pyside_utils.button_size[1] - 35

        # TOP ROW BUTTONS
        open_rosbag_button_pos_x = self._default_pad_x
        open_rosbag_button_pos_y = top_button_pos_y
        self._open_rosbag_button = self._pyside_utils.create_button("Open rosbag", self._window, position=(open_rosbag_button_pos_x, open_rosbag_button_pos_y), callback=self._open_rosbag)

        clear_last_point_button_pos_x = self._open_rosbag_button.geometry().x() + self._open_rosbag_button.geometry().width() + self._default_pad_x
        clear_last_point_button_pos_y = top_button_pos_y
        self._clear_pre_saved_points_button = self._pyside_utils.create_button("Clear Pre-Saved Points", self._window, position=(clear_last_point_button_pos_x, clear_last_point_button_pos_y), callback=self._clear_cache, size=(200, self._pyside_utils.button_size[1]))

        test_button_pos_x = clear_last_point_button_pos_x + self._clear_pre_saved_points_button.geometry().width() + self._default_pad_x
        test_button_pos_y = top_button_pos_y
        self._test_button = self._pyside_utils.create_button("Test projection", self._window, position=(test_button_pos_x, test_button_pos_y), callback=self._test)

        calibrate_button_pos_x = self._window_size[0] - self._pyside_utils.button_size[0] - 35
        calibrate_button_pos_y = top_button_pos_y
        self._calibrate_button = self._pyside_utils.create_button("Calibrate", self._window, position=(calibrate_button_pos_x, calibrate_button_pos_y), callback=self._calibrate)

        readme_path = "https://github.com/ddp22/Calibration_tool/blob/main/README.md"
        tutorial_button_pos_x = calibrate_button_pos_x - self._pyside_utils.button_size[0] - self._default_pad_x
        tutorial_button_pos_y = top_button_pos_y
        self._tutorial_button = self._pyside_utils.create_button("🛟 Help", self._window, position=(tutorial_button_pos_x, tutorial_button_pos_y), callback=lambda: webbrowser.open(readme_path))

        # BOTTOM ROW BUTTONS

        next_frame_button_pos_x = self._default_pad_x
        next_frame_button_pos_y = bottom_button_pos_y
        self._next_frame_button = self._pyside_utils.create_button("Next Frame", self._window, position=(next_frame_button_pos_x, next_frame_button_pos_y), callback=self._next_frame)

        delete_last_point_button_pos_x = self._next_frame_button.geometry().x() + self._next_frame_button.geometry().width() + self._default_pad_x
        delete_last_point_button_pos_y = bottom_button_pos_y
        self._delete_last_point_button = self._pyside_utils.create_button("Delete Last Point", self._window, position=(delete_last_point_button_pos_x, delete_last_point_button_pos_y), callback=self._delete_last_point_on_image)
        
        choose_3d_points_button_pos_x = delete_last_point_button_pos_x + self._delete_last_point_button.geometry().width() + self._default_pad_x
        choose_3d_points_button_pos_y = bottom_button_pos_y
        self._choose_3d_points_button = self._pyside_utils.create_button("Choose 3D Points", self._window, position=(choose_3d_points_button_pos_x, choose_3d_points_button_pos_y), callback=self._choose_3d_points)
        
        save_points_button_pos_x = self._choose_3d_points_button.geometry().x() + self._choose_3d_points_button.geometry().width() + self._default_pad_x
        save_points_button_pos_y = bottom_button_pos_y
        self._save_points_button = self._pyside_utils.create_button("Save Points", self._window, position=(save_points_button_pos_x, save_points_button_pos_y), callback=self._save_points)
        
        skip_frame_button_pos_x = save_points_button_pos_x + self._save_points_button.geometry().width() + self._default_pad_x
        skip_frame_button_pos_y = bottom_button_pos_y
        self._skip_frame_button = self._pyside_utils.create_button("Skip Frame", self._window, position=(skip_frame_button_pos_x, skip_frame_button_pos_y), callback=self._skip_frame)


        number_input_pos_x = skip_frame_button_pos_x + self._skip_frame_button.geometry().width() + self._default_pad_x
        number_input_pos_y = bottom_button_pos_y
        self._number_input = QtWidgets.QLineEdit(self._window)
        self._number_input.setPlaceholderText("Enter a number")
        self._number_input.setGeometry(number_input_pos_x, number_input_pos_y, 150, 35)
        self._number_input.textChanged.connect(self._on_text_changed)
        self._number_input.show()
        # Start the application event loop
        self._app.exec()

