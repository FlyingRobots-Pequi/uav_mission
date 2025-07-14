import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_interfaces.msg import MissionCommand, MissionState
from uav_interfaces.srv import SetpointControl

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
        setpoint_service_name = self._build_uav_topic('/setpoint_controller')
        
        # Publishers and Subscribers
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

        # Velocity control service client
        self.setpoint_client = self.create_client(SetpointControl, setpoint_service_name)
        while not self.setpoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aguardando serviço setpoint_controller...')

        # Mission control variables
        self.waiting_for_response = False
        self.current_command = None
        
        # UAV state tracking
        self.is_armed = False
        self.is_flying = False
        self.gesture_mode_active = False
        
        # Velocity control parameters
        self.move_velocity = 1.0  # m/s
        self.vertical_velocity = 0.5  # m/s
        self.movement_duration = 1.0  # seconds
        self.current_velocity_timer = None
        self.velocity_active = False
        
        # Takeoff sequence control
        self.takeoff_sequence_active = False
        self.takeoff_step = 0  # 0: OFFBOARD, 1: ARM, 2: TAKEOFF
        
        self.get_logger().info("=== GESTURE CONTROLLED MISSION NODE - VELOCITY MODE ===")
        self.get_logger().info("Integrado com FlightManagerNode")
        self.get_logger().info("Comandos disponíveis:")
        self.get_logger().info("- takeoff: Sequência completa de decolagem")
        self.get_logger().info("- land: Pousar e desarmar")
        self.get_logger().info("- hold: Manter posição atual")
        self.get_logger().info("- forward/back/left/right: Movimento por velocidade")
        self.get_logger().info("- up/down: Movimento vertical por velocidade")
        self.get_logger().info("- return: Voltar para origem (0,0,2)")
        self.get_logger().info("- stop: Parar movimento atual")
        self.get_logger().info("=========================================")

    def _build_uav_topic(self, uav_topic):
        """Build complete topic name with namespace prefix for UAV topics."""
        if self.uav_namespace:
            return f"{self.uav_namespace}{uav_topic}"
        return uav_topic

    def create_mission_command(self, command_dict):
        """Create MissionCommand message"""
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

    def send_mission_command(self, command_dict):
        """Send mission command to FlightManagerNode"""
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

    def send_velocity_command(self, vx, vy, vz, duration=None):
        """Send velocity command via setpoint service"""
        if not self.is_flying or not self.gesture_mode_active:
            self.get_logger().warn("Comando de velocidade ignorado - UAV não está em modo gesture ativo")
            return False
            
        # Stop any existing velocity timer
        if self.current_velocity_timer is not None:
            self.current_velocity_timer.cancel()
            
        # Create service request
        request = SetpointControl.Request()
        request.type = 'vel'
        request.vx = vx
        request.vy = vy
        request.vz = vz
        request.yaw = 0.0
        
        # Send velocity command
        future = self.setpoint_client.call_async(request)
        
        self.velocity_active = True
        self.get_logger().info(f"Comando de velocidade: vx={vx:.1f}, vy={vy:.1f}, vz={vz:.1f}")
        
        # Set timer to stop movement after duration
        if duration is None:
            duration = self.movement_duration
            
        self.current_velocity_timer = self.create_timer(duration, self.stop_velocity_movement)
        
        return True

    def stop_velocity_movement(self):
        """Stop current velocity movement"""
        if self.velocity_active:
            # Send zero velocity to stop
            request = SetpointControl.Request()
            request.type = 'vel'
            request.vx = 0.0
            request.vy = 0.0
            request.vz = 0.0
            request.yaw = 0.0
            
            future = self.setpoint_client.call_async(request)
            
            self.velocity_active = False
            self.get_logger().info("Movimento parado")
            
        # Cancel timer
        if self.current_velocity_timer is not None:
            self.current_velocity_timer.cancel()
            self.current_velocity_timer = None

    def gesture_callback(self, msg: String):
        """Process detected gestures and convert to mission commands"""
        gesture = msg.data.lower().strip()
        
        if not gesture:
            return
            
        self.get_logger().info(f"Gesto detectado: {gesture}")
        
        # Handle takeoff sequence
        if gesture == "takeoff":
            self.handle_takeoff_gesture()
        elif gesture == "land":
            self.handle_land_gesture()
        elif gesture == "hold":
            self.handle_hold_gesture()
        elif gesture == "stop":
            self.handle_stop_gesture()
        elif self.is_flying and self.gesture_mode_active:
            # Movement commands only when flying
            if gesture == "forward":
                self.handle_movement_gesture("forward")
            elif gesture == "back":
                self.handle_movement_gesture("back")
            elif gesture == "left":
                self.handle_movement_gesture("left")
            elif gesture == "right":
                self.handle_movement_gesture("right")
            elif gesture == "up":
                self.handle_movement_gesture("up")
            elif gesture == "down":
                self.handle_movement_gesture("down")
            elif gesture == "return":
                self.handle_return_gesture()
            else:
                self.get_logger().warn(f"Gesto não reconhecido: {gesture}")
        else:
            if gesture not in ["takeoff", "land", "hold", "stop"]:
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
            self.get_logger().info("Iniciando sequência de decolagem por gesto...")
            
            # Step 1: OFFBOARD
            command = {"command": "OFFBOARD"}
            self.send_mission_command(command)
        else:
            self.get_logger().info("Sequência de decolagem já em andamento...")

    def handle_land_gesture(self):
        """Handle landing sequence"""
        if not self.is_flying:
            self.get_logger().warn("UAV não está voando!")
            return
            
        # Stop any velocity movement first
        self.stop_velocity_movement()
            
        self.get_logger().info("Iniciando pouso por gesto...")
        command = {"command": "LAND"}
        self.send_mission_command(command)

    def handle_hold_gesture(self):
        """Handle hold position command"""
        if not self.is_flying:
            self.get_logger().warn("UAV não está voando!")
            return
            
        # Stop velocity movement and switch to position hold
        self.stop_velocity_movement()
            
        self.get_logger().info("Mantendo posição por gesto...")
        command = {"command": "HOLD"}
        self.send_mission_command(command)

    def handle_stop_gesture(self):
        """Handle stop movement gesture"""
        if not self.is_flying:
            self.get_logger().warn("UAV não está voando!")
            return
            
        self.get_logger().info("Parando movimento por gesto...")
        self.stop_velocity_movement()

    def handle_movement_gesture(self, direction):
        """Handle movement gestures using velocity control"""
        if not self.is_flying or not self.gesture_mode_active:
            self.get_logger().warn("Movimento ignorado - UAV não está em modo gesture ativo")
            return
            
        # Calculate velocity based on direction
        vx = 0.0
        vy = 0.0
        vz = 0.0
        
        if direction == "forward":
            vx = self.move_velocity
            self.get_logger().info(f"Movendo para frente - velocidade: {self.move_velocity}m/s por {self.movement_duration}s")
        elif direction == "back":
            vx = -self.move_velocity
            self.get_logger().info(f"Movendo para trás - velocidade: {self.move_velocity}m/s por {self.movement_duration}s")
        elif direction == "left":
            vy = self.move_velocity
            self.get_logger().info(f"Movendo para esquerda - velocidade: {self.move_velocity}m/s por {self.movement_duration}s")
        elif direction == "right":
            vy = -self.move_velocity
            self.get_logger().info(f"Movendo para direita - velocidade: {self.move_velocity}m/s por {self.movement_duration}s")
        elif direction == "up":
            vz = -self.vertical_velocity  # NED frame: negative Z is up
            self.get_logger().info(f"Subindo - velocidade: {self.vertical_velocity}m/s por {self.movement_duration}s")
        elif direction == "down":
            vz = self.vertical_velocity  # NED frame: positive Z is down
            self.get_logger().info(f"Descendo - velocidade: {self.vertical_velocity}m/s por {self.movement_duration}s")
            
        # Send velocity command
        self.send_velocity_command(vx, vy, vz)

    def handle_return_gesture(self):
        """Handle return to origin gesture"""
        # Stop velocity movement first
        self.stop_velocity_movement()
        
        self.get_logger().info("Retornando para origem por gesto...")
        command = {
            "command": "GOTO",
            "x": 0.0,
            "y": 0.0,
            "z": 2.0
        }
        self.send_mission_command(command)

    def mission_status_callback(self, msg: MissionState):
        """Handle mission status updates from FlightManagerNode"""
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
                # Stop any velocity movement
                self.stop_velocity_movement()
                # Wait for automatic disarm
                self.create_timer(0.5, self.check_disarm_after_land)
                
            elif command_name == "DISARM":
                self.is_armed = False
                self.get_logger().info("Modo de controle por gestos DESATIVADO!")
                
            elif command_name in ["GOTO", "HOLD"]:
                self.get_logger().info(f"Comando {command_name} completado")
                    
        elif msg.status == "FAILED":
            self.get_logger().error(f"Comando {command_name} falhou: {msg.info}")
            self.waiting_for_response = False
            
            # Reset takeoff sequence on failure
            if self.takeoff_sequence_active:
                self.takeoff_sequence_active = False
                self.takeoff_step = 0
                self.get_logger().error("Sequência de decolagem falhou!")

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
            self.get_logger().info("DECOLAGEM COMPLETA! Modo de controle por gestos ATIVADO!")
            self.get_logger().info("Agora você pode usar gestos para controlar o drone:")
            self.get_logger().info("- forward/back/left/right para movimento horizontal")
            self.get_logger().info("- up/down para movimento vertical")
            self.get_logger().info("- return para voltar à origem")
            self.get_logger().info("- stop para parar movimento")
            self.get_logger().info("- land para pousar")

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