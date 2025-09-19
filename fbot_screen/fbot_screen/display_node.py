import rclpy
import cv2
import numpy as np
import threading
import time
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageDraw, ImageFont

class MediaDisplayNode(Node):
    """
    @brief ROS2 node to display various media types (text, image, video, topic)
    #on a designated UI topic.
    """
    
    def __init__(self):
        """
        @brief Initializes the node, parameters, publishers, and subscribers.
        """
        super().__init__(node_name='media_display_node')
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        self.bridge = CvBridge()
        self.current_media_thread = None
        self.stop_media_flag = threading.Event()
        self.current_topic_subscriber = None
        ws_dir = os.path.abspath(os.path.join(get_package_share_directory('fbot_screen'), '../../../..'))
        self.file_path = os.path.join(ws_dir, "src", "fbot_hri", "fbot_screen", self.font_path)

        self.get_logger().info('Media Display Node started. Waiting for commands on /display_command')
    
    def initRosComm(self): 
        self.ui_publisher = self.create_publisher(Image, '/ui_display', 10)
        self.command_subscriber = self.create_subscription(String, '/display_command', self.command_callback, 10)

    def declareParameters(self):
        self.declare_parameter('screen.width', 800)
        self.declare_parameter('screen.height', 480)
        self.declare_parameter('screen.margin_x', 20)
        self.declare_parameter('screen.margin_y', 30)
        self.declare_parameter('font.path', '/fonts/vox_regular15.ttf')
    
    def readParameters(self):
        self.screen_width = self.get_parameter('screen.width').get_parameter_value().integer_value
        self.screen_height = self.get_parameter('screen.height').get_parameter_value().integer_value
        self.screen_margin_x = self.get_parameter('screen.margin_x').get_parameter_value().integer_value
        self.screen_margin_y = self.get_parameter('screen.margin_y').get_parameter_value().integer_value
        self.font_path = self.get_parameter('font.path').get_parameter_value().string_value

    def stop_current_media(self):
        """
        @brief Stops any currently running media display (video thread or topic subscription).
        """
        if self.current_media_thread is not None:
            self.stop_media_flag.set()
            self.current_media_thread.join()
            self.stop_media_flag.clear()
            self.current_media_thread = None

        if self.current_topic_subscriber is not None:
            self.destroy_subscription(self.current_topic_subscriber)
            self.current_topic_subscriber = None

    def command_callback(self, msg: String):
        """
        @brief: Callback function for the /display_command topic. Parses the command and calls the appropriate handler.
        @param msg: msg object containing the type and the command.
        """
        self.stop_current_media()

        parts = msg.data.split(':', 1)
        if len(parts) != 2:
            self.get_logger().error(f'Malformed command: "{msg.data}". Use "type:value".')
            return

        media_type, media_value = parts
        self.get_logger().info(f'Received command: type="{media_type}"')

        if media_type == 'sentence':
            self.handle_sentence(media_value)
        elif media_type == 'image':
            self.handle_image(media_value)
        elif media_type == 'video':
            self.current_media_thread = threading.Thread(target=self.handle_video, args=(media_value,))
            self.current_media_thread.start()
        elif media_type == 'topic':
            self.handle_topic(media_value)

    def handle_sentence(self, text:String):
        """ @brief Renders a sentence using an external TTF font and publishes it.
        This method uses the Pillow library to find the ideal font size
        (.ttf) that fits the text on the screen. It performs an iterative search,
        decreasing the font size until the text, broken into lines
        word by word, fits within the defined margins. The text is then
        centered and drawn on an image, which is converted to OpenCV format
        and published as a ROS message.
        @param text: The sentence to be displayed.
        """
        
        max_font_size = 250
        min_font_size = 10
        
        optimal_font_size = min_font_size
        final_lines = []
        
        for font_size in range(max_font_size, min_font_size - 1, -1):
            try:
                font = ImageFont.truetype(self.font_path, font_size)
            except IOError:
                self.get_logger().error(f"Font not found at '{self.font_path}'. Check the path.")
                error_image = np.zeros((self.screen_height, self.screen_width, 3), dtype=np.uint8)
                self.ui_publisher.publish(self.bridge.cv2_to_imgmsg(error_image, "bgr8"))
                return
            
            lines = []
            words = text.split()
            if not words: continue
            
            current_line = words[0]
            for word in words[1:]:
                if font.getlength(f"{current_line} {word}") <= (self.screen_width - 2 * self.screen_margin_x):
                    current_line += f" {word}"
                else:
                    lines.append(current_line)
                    current_line = word
            lines.append(current_line)
            total_height = len(lines) * font_size * 1.2
            
            if total_height <= (self.screen_height - 2 * self.screen_margin_y):
                optimal_font_size = font_size
                final_lines = lines
                break

        final_font = ImageFont.truetype(self.font_path, optimal_font_size)
        pil_image = PILImage.new('RGB', (self.screen_width, self.screen_height), color=(255, 255, 255))
        draw = ImageDraw.Draw(pil_image)
        line_height_approx = optimal_font_size * 1.2
        total_text_height = len(final_lines) * line_height_approx
        y = (self.screen_height - total_text_height) / 2

        for line in final_lines:
            line_width = final_font.getlength(line)
            x = (self.screen_width - line_width) / 2
            draw.text((x, y), line, font=final_font, fill=(0, 0, 0))
            y += line_height_approx
        image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        
        self.ui_publisher.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

    def handle_image(self, path: String):
        """
        @brief: Handles displaying an image from a given path.
        @param path: path to a image file
        """
        try:
            image = cv2.imread(path)
            if image is None:
                self.get_logger().error(f'Image not found or invalid: {path}')
                return
            resized_image = cv2.resize(image, (self.screen_width, self.screen_height))
            self.ui_publisher.publish(self.bridge.cv2_to_imgmsg(resized_image, "bgr8"))
            self.get_logger().info('Image published successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def handle_video(self, path: String):
        """
        @brief: Handles displaying a video from a given path in a loop.
        @param path: path to a video file.
        """
        cap = cv2.VideoCapture(path)
        while not self.stop_media_flag.is_set():
            ret, frame = cap.read()
            if not ret:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop the video
                continue

            resized_frame = cv2.resize(frame, (self.screen_width, self.screen_height))
            if self.stop_media_flag.is_set():
                break
            self.ui_publisher.publish(self.bridge.cv2_to_imgmsg(resized_frame, "bgr8"))
            time.sleep(1/30)  # Limit to ~30 FPS
        cap.release()

    def handle_topic(self, topic_name: String):
        """
        @brief Handles mirroring an existing image topic.
        @param topic_name: a topic name
        """
        
        self.current_topic_subscriber = self.create_subscription(
            Image, topic_name, self.ui_publisher.publish, 10)

def main(args=None):
    rclpy.init(args=args)
    node = MediaDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()