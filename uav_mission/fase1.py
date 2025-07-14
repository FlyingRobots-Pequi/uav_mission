import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped, PoseArray
from uav_interfaces.msg import MissionCommand, MissionState


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
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
        
        # Publishers e Subscribers
        self.mission_cmd_pub = self.create_publisher(MissionCommand, mission_cmd_topic, 10)
        self.mission_status_sub = self.create_subscription(
            MissionState, 
            mission_state_topic, 
            self.mission_status_callback, 
            10
        )

        # Subscriber para bases detectadas
        self.unique_positions_sub = self.create_subscription(
            PoseArray,
            '/unique_positions',
            self.unique_positions_callback,
            10
        )

        # Controle da missão
        self.mission_index = 0
        self.waiting_for_response = False
        
        # Controle de bases detectadas
        self.detected_bases = []
        self.bases_received = False
        self.current_base_index = 0
        self.visiting_bases = False
        
        # Controle simplificado de visitação
        self.visit_step = "IDLE"  # GOTO, LAND, OFFBOARD, ARM, TAKEOFF
        self.approach_altitude = -2.5   # Altitude de aproximação
        
        # Controle do passeio pela arena
        self.passeando_arena = False
        self.current_waypoint_index = 0
        self.arena_waypoints = [
            {"x": 3.0, "y": 0.0, "z": 2.5},    # Ponto 1
            {"x": 6.0, "y": 0.0, "z": 2.5},    # Ponto 2
            {"x": 6.0, "y": -3.0, "z": 2.5},   # Ponto 3
            {"x": 6.0, "y": -6.0, "z": 2.5},   # Ponto 4
            {"x": 3.0, "y": -6.0, "z": 2.5},   # Ponto 5
            {"x": 0.0, "y": -6.0, "z": 2.5},   # Ponto 6
            {"x": 0.0, "y": -3.0, "z": 2.5},   # Ponto 7
            {"x": 0.0, "y": 0.0, "z": 2.5},    # Volta ao centro
        ]

        # Sequência de comandos da missão - Fase 1 
        self.mission_commands = [
            {"command": "OFFBOARD"},
            {"command": "ARM"},
            {"command": "TAKEOFF", "z": 3.0},  # Altitude adequada para detecção
            {"command": "HOLD"},  # Estabilizar para detecção
            {"command": "PASSEAR_ARENA"},  # Percorre arena para detectar bases
            {"command": "VISIT_DETECTED_BASES"},  # Visita todas as bases
            {"command": "LAND"},  # Pouso final
            {"command": "DISARM"}
        ]

        # Inicia a missão
        self.get_logger().info("Mission Node iniciado. Começando Fase 1...")
        self.get_logger().info("Aguardando detecção de bases pelo sistema base_detection...")
        self.get_logger().info("")
        self.get_logger().info("=== SISTEMA DE POUSO SIMPLIFICADO ===")
        self.get_logger().info("Usando estados nativos do PX4 via flight_manager_node:")
        self.get_logger().info("   1. GOTO -> posição da base")
        self.get_logger().info("   2. LAND -> pouso automático + desarme")
        self.get_logger().info("   3. OFFBOARD -> retorna ao modo offboard")
        self.get_logger().info("   4. ARM -> rearma o drone")
        self.get_logger().info("   5. TAKEOFF -> decola para próxima base")
        self.get_logger().info("================================")
        self.get_logger().info("")
        self.send_next_command()

    def _build_uav_topic(self, uav_topic):
        """Build complete topic name with namespace prefix for UAV topics."""
        if self.uav_namespace:
            return f"{self.uav_namespace}{uav_topic}"
        return uav_topic

    def unique_positions_callback(self, msg: PoseArray):
        """Callback para receber bases detectadas do sistema base_detection"""
        if len(msg.poses) >= 3:  # Pelo menos 3 bases detectadas
            self.detected_bases = []
            for i, pose in enumerate(msg.poses):
                base = {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": self.approach_altitude,  # Usa altitude de aproximação
                    "id": i + 1
                }
                self.detected_bases.append(base)
            
            if not self.bases_received:  # Primeira vez que recebe bases
                self.bases_received = True
                self.get_logger().info(f"Detectadas {len(self.detected_bases)} bases!")
                self.get_logger().info(f"Altitude de aproximação: {self.approach_altitude:.1f}m")
                for base in self.detected_bases:
                    self.get_logger().info(f"   Base {base['id']}: ({base['x']:.3f}, {base['y']:.3f})")

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

    def order_bases_by_proximity(self, current_position):
        """Ordena as bases detectadas por proximidade da posição atual"""
        if not self.detected_bases:
            return
            
        import math
        
        # Calcula distância de cada base para posição atual
        for base in self.detected_bases:
            distance = math.sqrt(
                (base['x'] - current_position[0])**2 + 
                (base['y'] - current_position[1])**2
            )
            base['distance'] = distance
        
        # Ordena por distância (mais próxima primeiro)
        self.detected_bases.sort(key=lambda b: b['distance'])
        
        self.get_logger().info("Bases ordenadas por proximidade:")
        for i, base in enumerate(self.detected_bases):
            self.get_logger().info(f"   {i+1}. Base {base['id']}: ({base['x']:.3f}, {base['y']:.3f}) - distância: {base['distance']:.3f}m")

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

    def passear_pela_arena(self):
        """Envia comandos para passear pela arena e detectar bases"""
        if self.current_waypoint_index >= len(self.arena_waypoints):
            # Terminou de passear - volta para sequência normal
            self.passeando_arena = False
            self.mission_index += 1  # Próximo comando (VISIT_DETECTED_BASES)
            self.get_logger().info("Passeio pela arena completo! Iniciando visitação das bases...")
            self.send_next_command()
            return

        # Pega o waypoint atual
        current_waypoint = self.arena_waypoints[self.current_waypoint_index]
        
        self.get_logger().info(f"Passeio ponto {self.current_waypoint_index + 1}/{len(self.arena_waypoints)}: ({current_waypoint['x']:.1f}, {current_waypoint['y']:.1f}, {current_waypoint['z']:.1f})")
        
        # Cria comando GOTO para o waypoint
        goto_command = {
            "command": "GOTO",
            "x": current_waypoint["x"],
            "y": current_waypoint["y"],
            "z": current_waypoint["z"]
        }
        
        msg = self.create_mission_command(goto_command)
        self.mission_cmd_pub.publish(msg)
        
        self.waiting_for_response = True

    def send_next_base_visit(self):
        """Lógica simplificada para visitar bases usando estados do flight_manager"""
        if self.current_base_index >= len(self.detected_bases):
            # Terminou de visitar todas as bases
            self.visiting_bases = False
            self.visit_step = "IDLE"
            self.mission_index += 1  # Próximo comando (LAND final)
            self.get_logger().info("Todas as bases foram visitadas! Finalizando missão...")
            self.send_next_command()
            return

        # Pega a base atual
        current_base = self.detected_bases[self.current_base_index]
        
        if self.visit_step == "IDLE":
            # Inicia nova base - vai para posição
            self.visit_step = "GOTO"
            self.get_logger().info(f"=== VISITANDO BASE {current_base['id']} ({self.current_base_index + 1}/{len(self.detected_bases)}) ===")
            self.get_logger().info(f"[1/5] GOTO - Navegando para posição da base")
            
            goto_command = {
                "command": "GOTO",
                "x": current_base["x"],
                "y": current_base["y"], 
                "z": self.approach_altitude
            }
            msg = self.create_mission_command(goto_command)
            self.mission_cmd_pub.publish(msg)
            
        elif self.visit_step == "GOTO":
            # Chegou na posição - inicia pouso
            self.visit_step = "LAND"
            self.get_logger().info(f"[2/5] LAND - Pouso automático na Base {current_base['id']}")
            
            land_command = {"command": "LAND"}
            msg = self.create_mission_command(land_command)
            self.mission_cmd_pub.publish(msg)
            
        elif self.visit_step == "LAND":
            # Pousou e desarmou - volta ao offboard
            self.visit_step = "OFFBOARD"
            self.get_logger().info(f"[3/5] OFFBOARD - Ativando modo offboard para Base {current_base['id']}")
            
            offboard_command = {"command": "OFFBOARD"}
            msg = self.create_mission_command(offboard_command)
            self.mission_cmd_pub.publish(msg)
            
        elif self.visit_step == "OFFBOARD":
            # Modo offboard ativo - rearma
            self.visit_step = "ARM"
            self.get_logger().info(f"[4/5] ARM - Rearmando drone para Base {current_base['id']}")
            
            arm_command = {"command": "ARM"}
            msg = self.create_mission_command(arm_command)
            self.mission_cmd_pub.publish(msg)
            
        elif self.visit_step == "ARM":
            # Rearmado - decola
            self.visit_step = "TAKEOFF"
            self.get_logger().info(f"[5/5] TAKEOFF - Decolando da Base {current_base['id']}")
            
            takeoff_command = {
                "command": "TAKEOFF",
                "z": 2.5
            }
            msg = self.create_mission_command(takeoff_command)
            self.mission_cmd_pub.publish(msg)
            
        elif self.visit_step == "TAKEOFF":
            # Decolou - base visitada, próxima base
            self.get_logger().info(f"Base {current_base['id']} VISITADA COMPLETAMENTE!")
            self.current_base_index += 1
            self.visit_step = "IDLE"
            # Continua para próxima base sem aguardar resposta
            self.waiting_for_response = False
            self.send_next_base_visit()
            return
        
        self.waiting_for_response = True

    def mission_status_callback(self, msg: MissionState):
        """Callback simplificado para receber status da missão"""
        if not self.waiting_for_response:
            return
            
        current_command = self.mission_commands[self.mission_index]["command"]
        
        self.get_logger().info(f"Status recebido: {msg.status} - {msg.info}")
        
        if msg.status == "SUCCESS":
            self.get_logger().info(f"Comando {current_command} executado com sucesso!")
            
            # Se estamos visitando bases, processa próximo passo da visitação
            if self.visiting_bases:
                self.waiting_for_response = False
                self.send_next_base_visit()
                
            # Comandos especiais
            elif current_command == "PASSEAR_ARENA" and not self.passeando_arena:
                # Inicia passeio pela arena
                self.passeando_arena = True
                self.current_waypoint_index = 0
                self.waiting_for_response = False
                self.get_logger().info("Iniciando passeio pela arena para detectar bases...")
                self.passear_pela_arena()
                
            elif self.passeando_arena:
                # Completou waypoint atual - vai para próximo
                self.current_waypoint_index += 1
                self.waiting_for_response = False
                self.passear_pela_arena()
                    
            elif current_command == "VISIT_DETECTED_BASES":
                if not self.bases_received:
                    self.get_logger().error("Nenhuma base detectada! Pulando visitação...")
                    self.mission_index += 1
                    self.waiting_for_response = False
                    self.send_next_command()
                else:
                    # Ordena bases por proximidade antes de iniciar visitação
                    last_arena_position = self.arena_waypoints[-1]
                    current_pos = [last_arena_position['x'], last_arena_position['y']]
                    self.order_bases_by_proximity(current_pos)
                    
                    # Inicia processo de visitação
                    self.visiting_bases = True
                    self.current_base_index = 0
                    self.visit_step = "IDLE"
                    self.waiting_for_response = False
                    self.get_logger().info(f"Iniciando visitação de {len(self.detected_bases)} bases")
                    self.send_next_base_visit()
             
            else:
                # Comando normal da missão
                self.mission_index += 1
                self.waiting_for_response = False
                # Pequeno delay antes do próximo comando
                self.create_timer(0.05, self.send_next_command)
            
        elif msg.status == "FAILED":
            self.get_logger().error(f"Comando {current_command} falhou: {msg.info}")
            
            if self.visiting_bases:
                self.get_logger().error(f"Falha ao visitar Base {self.detected_bases[self.current_base_index]['id']}")
                # Tenta próxima base
                self.current_base_index += 1
                self.visit_step = "IDLE"
                self.waiting_for_response = False
                self.send_next_base_visit()
            elif self.passeando_arena:
                self.get_logger().error(f"Falha no waypoint {self.current_waypoint_index + 1} do passeio")
                # Tenta próximo waypoint
                self.current_waypoint_index += 1
                self.waiting_for_response = False
                self.passear_pela_arena()
            else:
                self.get_logger().error("Parando missão devido a falha.")
                self.waiting_for_response = False

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