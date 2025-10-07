from csv import reader
import cv2
from rosbags.rosbag2 import Reader
from rosbags.image import message_to_cvimage
from rosbags.typesys import Stores, get_typestore
import os
from pathlib import Path
import numpy as np

class RosbagUtils:
    def __init__(self):
        self._reader = None
        self._messages_reader = None
        self._camera_topic = None
        self._lidar_topic = None

    @staticmethod
    def topic_to_video(bag_folder_path: str, topic_name: str, output_video_path: str, img_width: int, img_height: int, fps: float = 30.0, encoding: str = "mp4v"):
        """
        Extracts messages from the topic `topic_name` (which must be images) from the bag and produces a video.

        - topic_name: name of the image topic in the bag (e.g., "/camera/image_raw" or "/camera/image_raw/compressed")
        - output_video_path: path of the output video file (e.g., "output.mp4")
        - fps: desired frames per second
        - encoding: video codec (default "mp4v" for .mp4). You can change it to “XVID”, “MJPG”, etc.

        Returns: True if successful, False otherwise.
        """


        # Open the reader
        with Reader(bag_folder_path) as reader:
            typestore = get_typestore(Stores.LATEST)
            metadata = reader.metadata
            topics_info = metadata.get('topics_with_message_count', [])
            topic_info = next((t for t in topics_info if t['topic_metadata']['name'] == topic_name), None)

            if not topic_info:
                raise ValueError(f"Topic {topic_name} not found in metadata")

            message_count = topic_info['message_count']
            duration_ns = metadata['duration']['nanoseconds']

            # Calculate FPS
            duration_seconds = duration_ns / 1e9
            fps = message_count / duration_seconds

            print(f"Using FPS: {fps}")
            fourcc = cv2.VideoWriter_fourcc(*encoding)
            print(f"Output video path: {output_video_path}")
            video_writer = cv2.VideoWriter(str(output_video_path), fourcc, fps, (img_width, img_height))
            if not video_writer.isOpened():
                print(f"Unable to open VideoWriter with {output_video_path}")
                return False

            for connection, timestamp, rawdata in reader.messages():
                if connection.topic != topic_name:
                    continue

                # Deserialize the message
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                try:
                    # Convert to OpenCV image (BGR)
                    img = message_to_cvimage(msg, 'bgr8')
                except Exception as e:
                    print(f"Error converting image for {topic_name}: {e}")
                    continue

                # Write the frame to the video
                video_writer.write(img)

            # At the end, release the video writer
            if video_writer is not None:
                video_writer.release()
                return True
            else:
                print(f"No frames written for topic {topic_name}")
                return False

    # @staticmethod
    # def get_fps(bag_folder_path: str, topic_name: str) -> float:
    #     if not os.path.exists(bag_folder_path):
    #         print(f"Bag folder path '{bag_folder_path}' does not exist.")
    #         return 0.0
        
    #     with Reader(bag_folder_path) as reader:
    #         metadata = reader.metadata
    #         topics_info = metadata.get('topics_with_message_count', [])
    #         topic_info = next((t for t in topics_info if t['topic_metadata']['name'] == topic_name), None)

    #         if not topic_info:
    #             raise ValueError(f"Topic {topic_name} not found in metadata")

    #         message_count = topic_info['message_count']
    #         duration_ns = metadata['duration']['nanoseconds']

    #         # Calculate FPS
    #         duration_seconds = duration_ns / 1e9
    #         fps = message_count / duration_seconds

    #         return fps

    @staticmethod
    def get_fps_sync(bag_folder_path: str, camera_topic: str, lidar_topic: str) -> float:
        if not os.path.exists(bag_folder_path):
            print(f"Bag folder path '{bag_folder_path}' does not exist.")
            return 0.0
        
        reader = Reader(bag_folder_path)
        reader.open()
        metadata = reader.metadata
        topics_info = metadata.get('topics_with_message_count', [])
        camera_topic_info = next((t for t in topics_info if t['topic_metadata']['name'] == camera_topic), None)
        lidar_topic_info = next((t for t in topics_info if t['topic_metadata']['name'] == lidar_topic), None)
        if not camera_topic_info:
            print(f"Camera topic '{camera_topic}' not found in bag metadata.")
            return 0.0
        if not lidar_topic_info:
            print(f"Lidar topic '{lidar_topic}' not found in bag metadata.")
            return 0.0

        count_sync = 0
        camera_msg = False
        lidar_msg = False

        for connection, _, _ in reader.messages():
                if connection.topic == camera_topic:
                    camera_msg = True
                elif connection.topic == lidar_topic:
                    lidar_msg = True

                if camera_msg and lidar_msg:
                    count_sync += 1
                    camera_msg = False
                    lidar_msg = False

        reader.close()

        duration_ns = metadata['duration']['nanoseconds']
        duration_seconds = duration_ns / 1e9
        fps = count_sync / duration_seconds if duration_seconds > 0 else 0.0

        return fps, count_sync, duration_ns

    def _pointcloud2_to_np_float32(self, msg):
        """
        Converte il messaggio PointCloud2 (già deserializzato) in un array numpy strutturato
        con x,y,z,intensity,ring,time, utilizzando float32 per i campi float.
        """
        num_points = msg.width * msg.height

        # definisci il dtype strutturato
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('ring', np.uint16),
            ('time', np.float32),
        ])

        buf = msg.data 
        arr = np.frombuffer(buf, dtype=dtype, count=num_points)
        arr = np.array([(point['x'], point['y'], point['z']) for point in arr], dtype=np.float32)

        return arr


    def init_lidar_camera_sync(self, bag_folder_path: str, camera_topic: str, lidar_topic: str):
        # check if bag path exists
        if not os.path.exists(bag_folder_path):
            print(f"Bag folder path '{bag_folder_path}' does not exist.")
            return False
        
        #check if topics exist in bag
        self._reader = Reader(bag_folder_path)
        self._reader.open()
        metadata = self._reader.metadata
        topics_info = metadata.get('topics_with_message_count', [])
        camera_topic_info = next((t for t in topics_info if t['topic_metadata']['name'] == camera_topic), None)
        lidar_topic_info = next((t for t in topics_info if t['topic_metadata']['name'] == lidar_topic), None)
        if not camera_topic_info:
            print(f"Camera topic '{camera_topic}' not found in bag metadata.")
            return False
        if not lidar_topic_info:
            print(f"Lidar topic '{lidar_topic}' not found in bag metadata.")
            return False
        
        self._messages_reader = self._reader.messages()
        self._camera_topic = camera_topic
        self._lidar_topic = lidar_topic
        return True

    def get_next_synced_messages(self):
        
        if self._messages_reader is None:
            print("Message reader not initialized. Call init_lidar_camera_sync first.")
            return None, None
        

        camera_msg = None
        lidar_msg = None
        frame = None

        try:
            while True:
                connection, timestamp, rawdata = next(self._messages_reader)
                if connection.topic == self._camera_topic:
                    camera_msg = (connection, timestamp, rawdata)
                elif connection.topic == self._lidar_topic:
                    lidar_msg = (connection, timestamp, rawdata)

                if camera_msg and lidar_msg:
                    
                    typestore = get_typestore(Stores.LATEST)
                    frame = typestore.deserialize_cdr(camera_msg[2], camera_msg[0].msgtype)
                    frame = message_to_cvimage(frame, 'bgr8')
                    lidar_raw = typestore.deserialize_cdr(lidar_msg[2], lidar_msg[0].msgtype)
                    lidar_np = self._pointcloud2_to_np_float32(lidar_raw)
                    
                    break
        except StopIteration:
            if self._reader:
                self._reader.close()
            self._reader = None
            return None, None


        camera_info = (frame, camera_msg[1])
        lidar_info = (lidar_np, lidar_msg[1])
        print("Camera info timestamp:", camera_info[1])
        print("Lidar info timestamp:", lidar_info[1])
        return camera_info, lidar_info


    def skip_messages(self, n: int):
        if self._messages_reader is None:
            print("Message reader not initialized. Call init_lidar_camera_sync first.")
            return
        
        try:
            for _ in range(n):
                next(self._messages_reader)
        except StopIteration:
            if self._reader:
                self._reader.close()
            self._reader = None
            print("Reached end of bag while skipping messages.")
            return


    def close_reader(self):
        if self._reader:
            self._reader.close()
            self._reader = None
            self._messages_reader = None




# if __name__ == "__main__":
#     bag_folder_path = "/home/ddp22/rosbags/rosbag2_2025_07_10-11_50_09"
#     topic_name = "/camera/compressed"
#     # output_video_path = str(Path(__file__).parent.parent / "output.mp4")
#     output_video_path = "/home/ddp22/Desktop/calibration/calibration/calibration_tool/videos/rosbag2_2025_07_10-11_50_09.mp4"
#     img_width = 1440
#     img_height = 1080


#     success = RosbagUtils.topic_to_video(bag_folder_path, topic_name, output_video_path, img_width, img_height)
#     if success:
#         print(f"Video successfully saved to {output_video_path}")
#     else:
#         print("Failed to create video.")


# if __name__ == "__main__":
#     bag_folder_path = "/home/ddp22/rosbags/rosbag2_2025_07_10-11_50_09"
#     camera_topic = "/camera/compressed"
#     lidar_topic = "/velodyne_points"

#     rosbag_utils = RosbagUtils()
#     if rosbag_utils.init_lidar_camera_sync(bag_folder_path, camera_topic, lidar_topic):
#         print("Lidar-Camera sync initialization completed.")
#         i = 0
#         while True:
#             camera_msg, lidar_msg = rosbag_utils.get_next_synced_messages(camera_topic, lidar_topic)
#             if camera_msg is None or lidar_msg is None:
#                 print("No more synchronized messages available.")
#                 break
#             i += 1
#             print(f"Camera Timestamp: {camera_msg[1]}, Lidar Timestamp: {lidar_msg[1]}")
#         print(f"Total synchronized message pairs: {i}")
#     else:
#         print("Lidar-Camera sync initialization failed.")