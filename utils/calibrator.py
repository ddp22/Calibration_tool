# import cv2
# import numpy as np
# import os
# import yaml
# from datetime import datetime

# class Calibrator:
#     def __init__(self, pattern_size, square_size, image_width, image_height):
#         self.obj_points = []  # 3D points in real world space
#         self.img_points = []  # 2D points in image plane
#         self._pattern_size = pattern_size
#         self.square_size = square_size
#         self.image_width = image_width
#         self.image_height = image_height
#         self.i = 0

#     def _find_chessboard(self, frame):
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         ret, corners = cv2.findChessboardCorners(gray, self._pattern_size, None)
#         if not ret:
#             # Second attempt with different preprocessing
#             _, gray = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
#             gray = cv2.bitwise_not(gray)
#             ret, corners = cv2.findChessboardCorners(gray, self._pattern_size, None)
#         return ret, corners

#     def get_intrinsic_camera_calibration(self):
#         self.objp = np.zeros((self._pattern_size[1] * self._pattern_size[0], 3), np.float32)
#         self.objp[:, :2] = np.mgrid[0:self._pattern_size[0], 0:self._pattern_size[1]].T.reshape(-1, 2)
#         self.objp *= self.square_size

#         for image_folder in os.listdir(self.extracted_frame_path):
#             image_path = os.path.join(self.extracted_frame_path, image_folder)
#             for frame in os.listdir(image_path):
#                 print(f"Processing image: {frame}")
#                 img = cv2.imread(os.path.join(image_path, frame))
#                 gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#                 ret, corners = self._find_chessboard(img)
#                 if ret:
#                     self.obj_points.append(self.objp)
#                     refined_corners = cv2.cornerSubPix(
#                         gray, corners, (11, 11), (-1, -1),
#                         criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#                     )
#                     self.img_points.append(refined_corners)
#                     self.i += 1

#         print(f"Total valid images for calibration: {self.i}")
#         if self.i < 15:
#             print("Not enough images for calibration. At least 15 are required.")
#             return None
        
#         ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
#             self.obj_points,
#             self.img_points,
#             (self.image_width, self.image_height),
#             None,
#             None
#         )

#         calibration_data = {
#             'calibration_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
#             'camera_matrix': {
#                 'rows': 3,
#                 'columns': 3,
#                 'data': camera_matrix.tolist()
#             },
#             'distortion_coefficients': {
#                 'rows': 1,
#                 'columns': len(dist_coeffs[0]),
#                 'data': dist_coeffs[0].tolist()
#             },
#             'chessboard': {
#                 'pattern_size': {
#                     'rows': self._pattern_size[1],
#                     'columns': self._pattern_size[0]
#                 },
#                 'square_size_meters': self.square_size
#             },
#             'image_size': {
#                 'width': self.image_width,
#                 'height': self.image_height
#             },
#             'rms_reprojection_error': ret
#         }

#         config_dir = os.path.join(os.path.dirname(__file__), 'config')
#         os.makedirs(config_dir, exist_ok=True)
#         calibration_file = os.path.join(config_dir, 'camera_intrinsic_calibration.yaml')
#         with open(calibration_file, 'w') as f:
#             yaml.dump(calibration_data, f)
