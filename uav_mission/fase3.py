import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_interfaces.msg import MissionCommand, MissionState
from uav_interfaces.srv import VehicleCommander, SetpointControl

# Gesture to velocity command mapping
GESTURE_TO_VELOCITY = {
    "forward":   {"vx": 0.3, "vy": 0.0, "vz": 0.0, "yaw": 0.0},
    "back":      {"vx": -0.3, "vy": 0.0, "vz": 0.0, "yaw": 0.0},
    "left":      {"vx": 0.0, "vy": -0.3, "vz": 0.0, "yaw": 0.0},
    "right":     {"vx": 0.0, "vy": 0.3, "vz": 0.0, "yaw": 0.0},
    "up":        {"vx": 0.0, "vy": 0.0, "vz": -0.6, "yaw": 0.0},
    "down":      {"vx": 0.0, "vy": 0.0, "vz": 0.3, "yaw": 0.0},
    "hold":      {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0},
}

# Gesture to action mapping 
GESTURE_TO_ACTION = {
    "land":          ("land",          {}),
    "return":        ("return",        {}),
    "arm":           ("arm",           {}),
    "disarm":        ("disarm",        {}),
}

class GestureControlledMissionNode(Node):
    def __init__(self):
        super().__init__('gesture_controlled_mission_node')
        
        # Declare and get namespace parameter for UAV topics
        self.declare_parameter('uav_namespace', '')
        self.uav_namespace = self.get_parameter('uav_namespace').value
        
        if self.uav_namespace:
            self.get_logger().info(f'Using UAV namespace: {self.uav_namespace}')
        else:
            self.get_logger().info('Using default UAV namespace (no prefix)')
        
        # Build topic names with namespace
        mission_cmd_topic = self._build_uav_topic('/mission_cmd')
        mission_state_topic = self._build_uav_topic('/mission_state')
        
        # Publishers e Subscribers (mesmo padrão da fase1)
        self.mission_cmd_pub = self.create_publisher(MissionCommand, mission_cmd_topic, 10)
        self.mission_status_sub = self.create_subscription(
            MissionState, 
            mission_state_topic, 
            self.mission_status_callback, 
            10
        )
        
        # Subscribe to gesture detection
        self.gesture_sub = self.create_subscription(
            String,
            'gesture_detected',
            self.gesture_callback,
            10
        )

        # Service clients for velocity control
        self.cmd_cli = self.create_client(VehicleCommander, 'vehicle_commander')
        self.setpoint_cli = self.create_client(SetpointControl, 'setpoint_controller')
        
        # Wait for services to be available
        self.cmd_cli.wait_for_service()
        self.setpoint_cli.wait_for_service()

        # Mission control variables (similar to fase1)
        self.waiting_for_response = False
        self.current_command = None
        
        # Gesture control
        self.last_gesture = None
        
        # UAV state tracking
        self.is_armed = False
        self.is_flying = False
        self.gesture_mode_active = False
        self.current_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        
        # Movement parameters
        self.step_size = 1.0  # Movimento horizontal 
        self.altitude_step = 0.5  # Movimento vertical
        self.max_altitude = 4.0  # meters
        self.min_altitude = 0.5  # meters
        
        # Takeoff sequence control
        self.takeoff_sequence_active = False
        self.takeoff_step = 0  # 0: OFFBOARD, 1: ARM, 2: TAKEOFF
        
        self.get_logger().info("=== GESTURE CONTROLLED MISSION NODE ===")
        self.get_logger().info("Integrado com FlightManagerNode e serviços de velocidade")
        self.get_logger().info("Comandos disponíveis:")
        self.get_logger().info("- takeoff: Sequência completa de decolagem")
        self.get_logger().info("- forward/back/left/right: Movimento com velocidade")
        self.get_logger().info("- up/down: Altitude com velocidade")
        self.get_logger().info("- hold: Parar movimento")
        self.get_logger().info("- land/return/arm/disarm: Comandos de veículo")
        self.get_logger().info("=========================================")

    def _build_uav_topic(self, uav_topic):
        """Build complete topic name with namespace prefix for UAV topics."""
        if self.uav_namespace:
            return f"{self.uav_namespace}{uav_topic}"
        return uav_topic

    def create_mission_command(self, command_dict):
        """Create MissionCommand message (same as fase1)"""
        msg = MissionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.command = command_dict["command"]
        
        # Create pose target
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position if specified
        pose.pose.position.x = command_dict.get("x", 0.0)
        pose.pose.position.y = command_dict.get("y", 0.0)
        pose.pose.position.z = command_dict.get("z", 0.0)
        
        # Default orientation
        pose.pose.orientation.w = 1.0
        
        msg.target_pose = pose
        msg.target_heading = command_dict.get("heading", 0.0)
        
        return msg

    def send_velocity_setpoint(self, cmd):
        """Send velocity setpoint command"""
        req = SetpointControl.Request()
        req.type = 'vel'
        req.vx = cmd["vx"]
        req.vy = cmd["vy"]
        req.vz = cmd["vz"]
        req.yaw = cmd["yaw"]
        
        self.get_logger().info(f"Enviando comando de velocidade: vx={cmd['vx']:.1f}, vy={cmd['vy']:.1f}, vz={cmd['vz']:.1f}, yaw={cmd['yaw']:.1f}")
        self.setpoint_cli.call_async(req)

    def send_vehicle_command(self, command, **kwargs):
        """Send vehicle command"""
        req = VehicleCommander.Request()
        req.command = command
        req.mode = kwargs.get('mode', "")
        req.param1 = float(kwargs.get('param1', 0.0))
        
        self.get_logger().info(f"Enviando comando de veículo: {command}")
        self.cmd_cli.call_async(req)

    def send_mission_command(self, command_dict):
        """Send mission command to FlightManagerNode """
        if self.waiting_for_response:
            self.get_logger().warn(f"Comando anterior ainda sendo executado. Ignorando: {command_dict['command']}")
            return False
            
        # Create and publish the message
        msg = self.create_mission_command(command_dict)
        self.mission_cmd_pub.publish(msg)
        
        # Update control state
        self.waiting_for_response = True
        self.current_command = command_dict
        
        command_name = command_dict["command"]
        if "x" in command_dict or "y" in command_dict or "z" in command_dict:
            x = command_dict.get("x", 0.0)
            y = command_dict.get("y", 0.0) 
            z = command_dict.get("z", 0.0)
            self.get_logger().info(f"Comando enviado para FlightManager: {command_name} -> ({x:.1f}, {y:.1f}, {z:.1f})")
        else:
            self.get_logger().info(f"Comando enviado para FlightManager: {command_name}")
            
        return True

    def gesture_callback(self, msg: String):
        """Process detected gestures and convert to mission commands"""
        gesture = msg.data.lower().strip()
        
        if not gesture:
            return
            
        # Avoid continuous repetition
        if gesture == self.last_gesture:
            return
            
        self.last_gesture = gesture
        self.get_logger().info(f" Gesto detectado: {gesture}")
        
        # Handle takeoff sequence (keep existing logic)
        if gesture == "takeoff":
            self.handle_takeoff_gesture()
        # Handle velocity commands 
        elif gesture in GESTURE_TO_VELOCITY and self.is_flying and self.gesture_mode_active:
            self.send_velocity_setpoint(GESTURE_TO_VELOCITY[gesture])
        # Handle vehicle commands
        elif gesture in GESTURE_TO_ACTION and self.is_flying:
            command, kwargs = GESTURE_TO_ACTION[gesture]
            self.send_vehicle_command(command, **kwargs)
        else:
            if gesture not in ["takeoff"]:
                self.get_logger().info(f"Gesto {gesture} ignorado - UAV não está voando ou modo gesture inativo")

    def handle_takeoff_gesture(self):
        """Handle takeoff sequence: OFFBOARD -> ARM -> TAKEOFF"""
        if self.is_flying:
            self.get_logger().warn("UAV já está voando!")
            return
            
        if not self.takeoff_sequence_active:
            # Start takeoff sequence
            self.takeoff_sequence_active = True
            self.takeoff_step = 0
            self.get_logger().info(" Iniciando sequência de decolagem por gesto...")
            
            # Step 1: OFFBOARD
            command = {"command": "OFFBOARD"}
            self.send_mission_command(command)
        else:
            self.get_logger().info("Sequência de decolagem já em andamento...")

    def mission_status_callback(self, msg: MissionState):
        """Handle mission status updates from FlightManagerNode (same pattern as fase1)"""
        if not self.waiting_for_response:
            return
            
        command_name = self.current_command["command"] if self.current_command else "UNKNOWN"
        
        self.get_logger().info(f"Status do FlightManager: {msg.status} - {msg.info}")
        
        if msg.status == "SUCCESS":
            self.get_logger().info(f"Comando {command_name} executado com sucesso!")
            self.waiting_for_response = False
            
            # Handle takeoff sequence progression
            if self.takeoff_sequence_active:
                self.handle_takeoff_sequence_success(command_name)
            
            # Update state based on completed command
            elif command_name == "LAND":
                self.is_flying = False
                self.gesture_mode_active = False
                self.current_position = {"x": 0.0, "y": 0.0, "z": 0.0}
                # Wait for automatic disarm
                self.create_timer(0.5, self.check_disarm_after_land)
                
            elif command_name == "DISARM":
                self.is_armed = False
                self.get_logger().info("Modo de controle por gestos DESATIVADO!")

        elif msg.status == "FAILED":
            self.get_logger().error(f"Comando {command_name} falhou: {msg.info}")
            self.waiting_for_response = False
            
            # Reset takeoff sequence on failure
            if self.takeoff_sequence_active:
                self.takeoff_sequence_active = False
                self.takeoff_step = 0
                self.get_logger().error("Sequência de decolagem falhou!")
            
        # If status is ONGOING, continue waiting

    def handle_takeoff_sequence_success(self, command_name):
        """Handle successful steps in takeoff sequence"""
        if command_name == "OFFBOARD":
            # Step 2: ARM
            self.takeoff_step = 1
            self.get_logger().info("OFFBOARD ativo. Armando drone...")
            self.create_timer(0.2, self.send_arm_command)
            
        elif command_name == "ARM":
            # Step 3: TAKEOFF
            self.takeoff_step = 2
            self.is_armed = True
            self.get_logger().info("Drone armado. Decolando...")
            self.create_timer(0.2, self.send_takeoff_command)
            
        elif command_name == "TAKEOFF":
            # Takeoff sequence complete
            self.takeoff_sequence_active = False
            self.takeoff_step = 0
            self.is_flying = True
            self.gesture_mode_active = True
            self.current_position = {"x": 0.0, "y": 0.0, "z": 2.0}
            self.get_logger().info(" DECOLAGEM COMPLETA! Modo de controle por gestos ATIVADO!")
            self.get_logger().info("Agora você pode usar gestos para controlar o drone:")
            self.get_logger().info("- forward/back/left/right para movimento com velocidade")
            self.get_logger().info("- up/down para altitude com velocidade")
            self.get_logger().info("- hold para parar movimento")
            self.get_logger().info("- land/return/arm/disarm para comandos de veículo")

    def send_arm_command(self):
        """Send ARM command (delayed)"""
        command = {"command": "ARM"}
        self.send_mission_command(command)

    def send_takeoff_command(self):
        """Send TAKEOFF command (delayed)"""
        command = {"command": "TAKEOFF", "z": 2.0}
        self.send_mission_command(command)

    def check_disarm_after_land(self):
        """Send DISARM command after landing"""
        if not self.is_flying:
            command = {"command": "DISARM"}
            self.send_mission_command(command)

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlledMissionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Missão de controle por gestos interrompida pelo usuário")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 