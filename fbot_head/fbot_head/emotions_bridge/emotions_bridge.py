import os
import yaml
import json
import time
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool  # Added: Import for the blinking service
from ament_index_python.packages import get_package_share_directory

class EmotionsBridge(Node):

    def __init__(self, pause=False):
        """
        @brief A Node for managing emotions and motor configurations.
        @param pause: If True, the node will not send any data to the motors.
        """
        self.declare_parameter('default_blink', False) # Default state if not specified
        default_blink_val = self.get_parameter('default_blink').value

        super().__init__('emotions_bridge')

        try:
            self.serial = serial.Serial('/dev/ttyFACE', 9600, timeout=1) # Mantido conforme padrão ESP32
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            return

        self.motors = None
        #LOAD YAML PARAMETERS  
        self.loadMotorsParams('motors.yaml')

        self.sendMotorsConfig()
        
        self.current_emotion = 'neutral'
        self.sendEmotion(self.current_emotion)

        self.sub_emotion = self.create_subscription(String, 'fbot_face/emotion', self.emotionCallback, 10)

        # ADDED: Service to enable/disable blinking (CMD 3)
        self.srv_blink = self.create_service(SetBool, 'set_blink', self.set_blink_callback)
        self.get_logger().info("Serviço 'set_blink' (CMD 3) inicializado.")

        time.sleep(2)

    # ADDED: NEW SERVICE CALLBACK (CMD 3)
    def set_blink_callback(self, request, response):
        """
        @brief Callback for the blinking service. Sends CMD 3 to the firmware.
        """
        try:
            # Mount the JSON for the CMD 3 defined in the firmware
            msg = {"cmd": 3, "blink": request.data}
            data_str = json.dumps(msg)
            data_bytes = data_str.encode('utf-8')
            
            self.serial.write(data_bytes)
            self.get_logger().info(f'Sent blink command (CMD 3): {data_str}')

            # Awaiting successful response from the microcontroller
            if self.waitSerialResponse("success"):
                response.success = True
                response.message = f"Piscagem definida como: {request.data}"
            else:
                response.success = False
                response.message = "Microcontrolador não confirmou CMD 3"
                
        except Exception as e:
            response.success = False
            response.message = f"Erro ao enviar comando de piscagem: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
        
    def sendMotorsConfig(self) -> None:  
        """
        @brief Sends motor configuration data to the microcontroller, so it can instantiate the motor classes in its firmware.
        """
        max_retries = 3
        retries = 0
        success = False

        motors_pins: dict = {
            "cmd": 1,
            "blink_cfg": {} # New sub-dictionary for blinking parameters
        }

        for motor in self.motors:
            motors_pins[motor] = self.get_parameter(motor+'.pin').value
            
            # Verifies if it's a eyelid configured for blinking (Dynamically searches in the YAML)
            if self.has_parameter(motor+'.blink_left') and self.get_parameter(motor+'.blink_left').value:
                motors_pins["blink_cfg"]["nameL"] = motor
                motors_pins["blink_cfg"]["angleL"] = self.get_parameter(motor+'.blink_closed_angle').value
            
            if self.has_parameter(motor+'.blink_right') and self.get_parameter(motor+'.blink_right').value:
                motors_pins["blink_cfg"]["nameR"] = motor
                motors_pins["blink_cfg"]["angleR"] = self.get_parameter(motor+'.blink_closed_angle').value

        data_str = json.dumps(motors_pins)
        data_bytes = data_str.encode('utf-8')

        self.serial.write(data_bytes)
        self.get_logger().info(f'Sent motor configuration: {data_str}')
        
        while not success and retries<max_retries:
            self.get_logger().info(f'Waiting for motor configuration acknowledgment... Attempt {retries+1}/{max_retries}')
            if self.waitSerialResponse("success"):
                self.get_logger().info('Motor configuration successfully sent and acknowledged by the microcontroller.')
                success = True
            else:
                retries+=1

    def waitSerialResponse(self, response_msg: str) -> bool:  
        """
        @brief Waits for a response from the serial port and checks if it matches the expected response.
        @param response_msg: (str) The expected response message.  
        """
        received_msg = ""
        while True:
            if self.serial.in_waiting > 0:
                try:
                    received_msg += self.serial.read(self.serial.in_waiting).decode('utf-8')
                except UnicodeDecodeError as e:
                    self.get_logger().error(f"Error decoding serial data: {e}")
                    return False
                if "{" in received_msg and received_msg.count("}") == received_msg.count("{"): 
                    break

        start_index = received_msg.find("{\"response\"")
        end_index = received_msg.rfind("}")
        message = received_msg[start_index : end_index+1]
        received_json = json.loads(message)

        if response_msg == received_json["response"]:
            self.get_logger().info(f'Received expected response: {message}')
            return True
        else:
            self.get_logger().warn(f'Unexpected response received: {message}')
            return False

    def emotionCallback(self, msg: String) -> None:  
        """
        @brief Callback function for the emotion subscription. It updates the current emotion and sends the corresponding motor commands
        @param msg: (std_msgs.msg.String) The message containing the new emotion.  
        """

        self.current_emotion = msg.data
        self.sendEmotion(self.current_emotion)

    def sendEmotion(self, emotion) -> None:
        """
        @brief Retrieves motor values for the given emotion and sends them as a JSON-encoded string via serial communication.
        @param emotion: (str) The emotion to be sent.  
        """
        if self.motors is None:
            self.get_logger().error("Motors configuration not loaded. Cannot send emotion.")
            return

        log_message = ''

        motors_dict = {
            "cmd": 2
        }

        for motor in self.motors:
            if not self.has_parameter(motor+'.'+emotion):
                self.get_logger().error(f"Parameter '{motor}.{emotion}' not declared. Cannot send emotion to motor '{motor}'.")
                return
            
            write_msg = [motor, self.get_parameter(motor+'.'+emotion).value] #opção sem namespace

            motors_dict[motor] = write_msg[1]

            log_message = log_message + write_msg[0] + ': ' + str(write_msg[1]) + '\n'

        data_str = json.dumps(motors_dict)

        data_bytes = data_str.encode('utf-8')

        self.get_logger().info(f'Sending emotion: {emotion}\n{log_message}')
        self.get_logger().info(f'Sending data: {data_str}')

        self.serial.write(data_bytes)

        self.waitSerialResponse("success")

        return

    
    def loadMotorsParams(self, filename: str) -> None:  
        """
        @brief Loads motor configurations from a YAML file and declares them as parameters.
        @param filename: (str) The name of the YAML file at the config directory.  
        """
        with open(os.path.join(get_package_share_directory('fbot_head'), 'config', filename)) as config_file:
            config = yaml.safe_load(config_file)[self.get_name()]['ros__parameters']

        self.motors = config
        for motor, params in config.items():
            # Itera sobre cada sub-item do motor (pin, blink_left, etc)
            for param_name, param_value in params.items():
                self.declare_parameter(motor + '.' + param_name, param_value)
                
            

def main(args=None):
    rclpy.init(args=args)
    node = EmotionsBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()