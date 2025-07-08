import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from uav_interfaces.msg import MissionCommand, MissionState

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # Publishers e Subscribers
        self.mission_cmd_pub = self.create_publisher(MissionCommand, '/mission_cmd', 10)
        self.mission_status_sub = self.create_subscription(
            MissionState, 
            '/mission_state', 
            self.mission_status_callback, 
            10
        )

        # Controle da missão
        self.mission_index = 0
        self.waiting_for_response = False
        
        # Sequência de comandos da missão
        self.mission_commands = [
            {"command": "OFFBOARD"},
            {"command": "ARM"},
            {"command": "TAKEOFF"},  # Usa default_takeoff_altitude do config
            {"command": "HOLD"},
            {"command": "GOTO", "x": 2.0, "y": -1.0, "z": 2.0},
            {"command": "HOLD"},
            {"command": "GOTO", "x": 3.0, "y": -4.0, "z": 5.0},
            {"command": "HOLD"},
            {"command": "LAND"},
            {"command": "DISARM"}
        ]

        # Inicia a missão
        self.get_logger().info("Mission Node iniciado. Começando missão...")
        self.send_next_command()

    def create_mission_command(self, command_dict):
        """Cria uma mensagem MissionCommand a partir do dicionário"""
        msg = MissionCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.command = command_dict["command"]
        
        # Cria pose target
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # Define posição se especificada
        pose.pose.position.x = command_dict.get("x", 0.0)
        pose.pose.position.y = command_dict.get("y", 0.0)
        pose.pose.position.z = command_dict.get("z", 0.0)
        
        # Orientação padrão
        pose.pose.orientation.w = 1.0
        
        msg.target_pose = pose
        msg.target_heading = command_dict.get("heading", 0.0)
        
        return msg

    def send_next_command(self):
        """Envia o próximo comando da missão"""
        if self.mission_index >= len(self.mission_commands):
            self.get_logger().info("Missão completa!")
            return
            
        if self.waiting_for_response:
            return

        # Pega o próximo comando
        current_command = self.mission_commands[self.mission_index]
        
        # Cria e publica a mensagem
        msg = self.create_mission_command(current_command)
        self.mission_cmd_pub.publish(msg)
        
        # Atualiza estado
        self.waiting_for_response = True
        
        self.get_logger().info(f"Comando enviado: {current_command['command']}")

    def mission_status_callback(self, msg: MissionState):
        """Callback para receber status da missão"""
        if not self.waiting_for_response:
            return
            
        command_name = self.mission_commands[self.mission_index]["command"]
        
        self.get_logger().info(f"Status recebido: {msg.status} - {msg.info}")
        
        if msg.status == "SUCCESS":
            self.get_logger().info(f"Comando {command_name} executado com sucesso!")
            self.mission_index += 1
            self.waiting_for_response = False
            
            # Pequeno delay antes do próximo comando (50ms)
            self.create_timer(0.05, self.send_next_command_delayed)
            
        elif msg.status == "FAILED":
            self.get_logger().error(f"Comando {command_name} falhou: {msg.info}")
            self.get_logger().error("Parando missão devido a falha.")
            self.waiting_for_response = False
            
        # Se status é ONGOING, continua aguardando

    def send_next_command_delayed(self):
        """Envia próximo comando após delay (chamado uma vez apenas)"""
        self.send_next_command()

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Missão interrompida pelo usuário")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
