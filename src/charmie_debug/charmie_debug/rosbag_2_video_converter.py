import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class ImageToFileConverter(Node):
    def __init__(self):
        super().__init__('image_to_file_converter')
        print("Node initialized")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/CHARMIE/D455_head/color/image_raw',
            self.image_callback,
            10)
        self.image_count = 0
        self.output_directory = 'output_images'
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)

    def image_callback(self, msg):
        print("Image callback")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #print("Image shape:", cv_image.shape)
        print(self.image_count)
        image_filename = os.path.join(self.output_directory, f'image_{self.image_count:04d}.jpg')
        cv2.imwrite(image_filename, cv_image)
        self.image_count += 1
        print(self.image_count)

        # Check if all images are processed
        if self.image_count >= 1000:  # Modify this value according to your needs
            self.convert_images_to_video()

    def convert_images_to_video(self):
        output_video_file = 'output_video.avi'
        images_to_video(self.output_directory, output_video_file)
        print("Video saved as:", output_video_file)
        self.destroy_node()
        rclpy.shutdown()


def images_to_video(image_folder, output_video_file, fps=30):
    # Get the list of image filenames
    image_files = [os.path.join(image_folder, img) for img in os.listdir(image_folder) if img.endswith(".jpg")]
    image_files.sort()  # Sort image filenames in ascending order

    # Read the first image to get the dimensions
    first_image = cv2.imread(image_files[0])
    height, width, _ = first_image.shape

    # Initialize VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI format
    video_writer = cv2.VideoWriter(output_video_file, fourcc, fps, (width, height))

    # Iterate through image files and write them to the video
    for image_file in image_files:
        frame = cv2.imread(image_file)
        video_writer.write(frame)

    # Release VideoWriter object
    video_writer.release()


def main(args=None):
    rclpy.init(args=args)
    image_to_file_converter = ImageToFileConverter()
    rclpy.spin(image_to_file_converter)


if __name__ == '__main__':
    main()