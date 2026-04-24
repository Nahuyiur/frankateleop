import numpy as np
import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API


class RSCapture:
    def get_device_serial_numbers(self):
        devices = rs.context().devices
        return [d.get_info(rs.camera_info.serial_number) for d in devices]

    def __init__(self, name, serial_number, dim=(640, 480), fps=15, depth=True):
        self.name = name
        print(self.get_device_serial_numbers())
        assert serial_number in self.get_device_serial_numbers(), serial_number + " not connected!"
        self.serial_number = serial_number
        self.depth = depth
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_device(self.serial_number)
        self.cfg.enable_stream(rs.stream.color, dim[0], dim[1], rs.format.bgr8, fps)


        if self.depth:
            self.cfg.enable_stream(rs.stream.depth, dim[0], dim[1], rs.format.z16, fps)
        self.profile = self.pipe.start(self.cfg)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        print(self.depth_scale, self.intr)

        # Intr 
        # fx, fy, cx, cy 
        # [604.859 605.033, 310.667 251.27] 
        # Extrinsics
        # x, y, z, qx, qy, qz, qw
        # 

    def read(self):
        frames = self.pipe.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        image = np.asarray(color_frame.get_data())

        if self.depth:
            depth_frame = aligned_frames.get_depth_frame()
            depth_data = np.asarray(depth_frame.get_data()) * self.depth_scale
            depth = np.expand_dims(depth_data, axis=2)
        else:
            depth = None

        return image, depth
        

    def close(self):
        self.pipe.stop()
        self.cfg.disable_all_streams()
