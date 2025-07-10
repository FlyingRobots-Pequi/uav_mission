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

        # NOVO: Subscriber para bases detectadas
        self.unique_positions_sub = self.create_subscription(
            PoseArray,
            '/unique_positions',
            self.unique_positions_callback,
            10
        )

        # Controle da missão
        self.mission_index = 0
        self.waiting_for_response = False
        
        # NOVO: Controle de bases detectadas
        self.detected_bases = []
        self.bases_received = False
        self.current_base_index = 0
        self.visiting_bases = False
        
        # NOVO: Controle avançado de visitação
        self.visit_type = "HOVER"  # Opções: "HOVER", "LOW_LAND", "FULL_LAND"
        self.visit_hover_altitude = -0.5  # Altitude para hover sobre a base
        self.visit_low_land_altitude = -1.0  # Altitude para pouso baixo
        self.visit_full_land_altitude = -1.8  # Altitude para pouso completo
        self.visit_duration = 3.0  # Tempo em segundos para ficar em cada base
        self.current_visit_start_time = None
        self.visit_timer_started = False  # Flag para controlar se timer já foi iniciado
        
        # NOVO: Controle de fases de visitação (pouso e decolagem)
        self.visit_phase = "IDLE"  # APPROACHING, LANDING, ON_GROUND, TAKING_OFF, COMPLETED
        self.approach_altitude = -0.5   # Altitude de aproximação
        self.landing_altitude = -1.8    # Altitude de pouso 
        self.ground_duration = 3.0      # Tempo no solo em segundos
        self.processing_visit_transition = False  # Flag para evitar múltiplas transições
        
        # NOVO: Controle do passeio pela arena
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

        # NOVA Sequência de comandos da missão - Fase 1 
        # Fase 1: 
        #   1. Decola para altitude de detecção
        #   2. Passeia pela arena para detectar bases
        #   3. Aguarda detecção das bases 
        #   4. Navega para cada base detectada e pousa
        #   5. Repete para todas as bases
        self.mission_commands = [
            {"command": "OFFBOARD"},
            {"command": "ARM"},
            {"command": "TAKEOFF", "z": 3.0},  # Altitude adequada para detecção
            {"command": "HOLD"},  # Estabilizar para detecção
            {"command": "PASSEAR_ARENA"},  # Percorre arena para detectar bases  # Aguarda bases serem detectadas
            {"command": "VISIT_DETECTED_BASES"},  # Visita todas as bases
            {"command": "FINAL_LAND"},  # Pouso final
            {"command": "DISARM"}
        ]

        # Inicia a missão
        self.get_logger().info("Mission Node iniciado. Começando Fase 1...")
        self.get_logger().info("Aguardando detecção de bases pelo sistema base_detection...")
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
                    "z": self.get_visit_altitude(),  # Usa altitude configurável
                    "id": i + 1
                }
                self.detected_bases.append(base)
            
            if not self.bases_received:  # Primeira vez que recebe bases
                self.bases_received = True
                self.get_logger().info(f"Detectadas {len(self.detected_bases)} bases!")
                self.get_logger().info(f"Modo de visitação: {self.visit_type} (altitude: {self.get_visit_altitude():.1f}m)")
                for base in self.detected_bases:
                    self.get_logger().info(f"   Base {base['id']}: ({base['x']:.3f}, {base['y']:.3f})")
            else:
                # Atualização das bases (não loga novamente)
                self.get_logger().debug(f"Atualizadas {len(self.detected_bases)} bases detectadas")
        else:
            self.get_logger().debug(f"Detectadas apenas {len(msg.poses)} bases (mínimo: 3)")

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
    
    def get_visit_altitude(self):
        """Retorna a altitude adequada baseada no tipo de visitação"""
        if self.visit_type == "HOVER":
            return self.visit_hover_altitude
        elif self.visit_type == "LOW_LAND":
            return self.visit_low_land_altitude
        elif self.visit_type == "FULL_LAND":
            return self.visit_full_land_altitude
        else:
            return self.visit_hover_altitude  # Padrão

    def set_visit_configuration(self, visit_type="HOVER", duration=3.0):
        """Configura o tipo e duração da visitação das bases"""
        valid_types = ["HOVER", "LOW_LAND", "FULL_LAND"]
        if visit_type not in valid_types:
            self.get_logger().error(f"Tipo de visitação inválido: {visit_type}. Opções: {valid_types}")
            return False
            
        self.visit_type = visit_type
        self.visit_duration = duration
        
        self.get_logger().info(f"Configuração de visitação atualizada:")
        self.get_logger().info(f"   Tipo: {visit_type} (altitude: {self.get_visit_altitude():.1f}m)")
        self.get_logger().info(f"   Duração: {duration:.1f}s por base")
        return True

    def send_next_command(self):
        """Envia o próximo comando da missão"""
        if self.mission_index >= len(self.mission_commands):
            self.get_logger().info("Fase 1 completa! Todas as bases foram visitadas.")
            return
            
        if self.waiting_for_response:
            return

        # Lógica especial para visitar bases
        if self.visiting_bases:
            self.send_next_base_visit()
            return
            
        # Lógica especial para passear pela arena
        if self.passeando_arena:
            self.passear_pela_arena()
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
        """Envia comandos para visitar a próxima base detectada com sequência completa"""
        if self.current_base_index >= len(self.detected_bases):
            # Terminou de visitar todas as bases
            self.visiting_bases = False
            self.visit_phase = "IDLE"
            self.processing_visit_transition = False
            self.mission_index += 1  # Próximo comando (FINAL_LAND)
            self.get_logger().info("Todas as bases foram visitadas! Finalizando missão...")
            self.send_next_command()
            return

        # Pega a base atual
        current_base = self.detected_bases[self.current_base_index]
        
        # Determina a próxima fase da visitação
        if self.visit_phase == "IDLE" or self.visit_phase == "COMPLETED":
            # Inicia nova base - fase de aproximação
            if self.visit_phase == "COMPLETED":
                self.get_logger().info(f"Iniciando visitação da Base {current_base['id']}")
            self.visit_phase = "APPROACHING"
            self.get_logger().info(f"[FASE 1/4] APROXIMAÇÃO da Base {current_base['id']} ({self.current_base_index + 1}/{len(self.detected_bases)})")
            self.get_logger().info(f"   Posição: ({current_base['x']:.3f}, {current_base['y']:.3f}, {self.approach_altitude:.3f})")
            
            # Comando GOTO para aproximação
            goto_command = {
                "command": "GOTO",
                "x": current_base["x"],
                "y": current_base["y"], 
                "z": self.approach_altitude
            }
            msg = self.create_mission_command(goto_command)
            self.mission_cmd_pub.publish(msg)
            self.processing_visit_transition = False  # Reset após enviar comando
            
        elif self.visit_phase == "APPROACHING":
            # Aproximação completa - comando LAND real
            self.visit_phase = "LANDING"
            self.get_logger().info(f"[FASE 2/4] POUSO REAL na Base {current_base['id']}")
            self.get_logger().info(f"   Comando LAND - pousará na altura real do terreno")
            
            # Comando LAND real do PX4
            land_command = {"command": "LAND"}
            msg = self.create_mission_command(land_command)
            self.mission_cmd_pub.publish(msg)
            self.processing_visit_transition = False  # Reset após enviar comando
            
        elif self.visit_phase == "LANDING":
            # Pouso completo - aguardar no solo
            self.visit_phase = "ON_GROUND"
            self.get_logger().info(f"[FASE 3/4] NO SOLO - Base {current_base['id']} (aguardando {self.ground_duration}s)")
            self.current_visit_start_time = time.time()
            self.visit_timer_started = True
            self.processing_visit_transition = False  # Reset antes do timer
            # Inicia timer para tempo no solo
            self.create_timer(1.0, lambda: self.check_ground_time())
            return  # Não envia comando, apenas aguarda tempo
            
        elif self.visit_phase == "ON_GROUND":
            # Tempo no solo completo - comando TAKEOFF real
            self.visit_phase = "TAKING_OFF"
            takeoff_altitude = 2.0  # Altitude para decolagem
            self.get_logger().info(f"[FASE 4/4] DECOLAGEM REAL da Base {current_base['id']}")
            self.get_logger().info(f"   Comando TAKEOFF para {takeoff_altitude}m")
            
            # Comando TAKEOFF real do PX4
            takeoff_command = {
                "command": "TAKEOFF",
                "z": takeoff_altitude
            }
            msg = self.create_mission_command(takeoff_command)
            self.mission_cmd_pub.publish(msg)
            self.processing_visit_transition = False  # Reset após enviar comando
            
        elif self.visit_phase == "TAKING_OFF":
            # Decolagem completa - próxima base
            self.visit_phase = "COMPLETED"
            self.current_base_index += 1
            self.visit_timer_started = False
            self.get_logger().info(f"Base {current_base['id']} visitada completamente!")
            self.get_logger().info(f"Avançando para próxima base ({self.current_base_index + 1}/{len(self.detected_bases)})...")
            self.processing_visit_transition = False  # Reset para permitir próxima base
            self.send_next_base_visit()  # Recursivo para próxima base
            return
        
        if 'distance' in current_base:
            self.get_logger().info(f"   Distância: {current_base['distance']:.3f}m")
        
        self.waiting_for_response = True

    def check_ground_time(self):
        """Verifica se o tempo no solo da base atual foi completado."""
        if not self.visiting_bases or self.waiting_for_response:
            return

        elapsed_time = time.time() - self.current_visit_start_time

        if elapsed_time >= self.ground_duration:
            self.get_logger().info(f"Tempo no solo da Base {self.detected_bases[self.current_base_index]['id']} completo! ({elapsed_time:.1f}s)")
            self.visit_phase = "TAKING_OFF"
            self.visit_timer_started = False
            self.processing_visit_transition = True  # Inicia nova transição
            self.send_next_base_visit()
        else:
            remaining_time = self.ground_duration - elapsed_time
            self.get_logger().info(f"Tempo no solo - Base {self.detected_bases[self.current_base_index]['id']} - restam {remaining_time:.1f}s")
            # Cria timer para verificar novamente (apenas se ainda no solo)
            if self.visiting_bases:
                self.create_timer(1.0, lambda: self.check_ground_time())

    def mission_status_callback(self, msg: MissionState):
        """Callback para receber status da missão"""
        if not self.waiting_for_response:
            return
            
        current_command = self.mission_commands[self.mission_index]["command"]
        
        self.get_logger().info(f"Status recebido: {msg.status} - {msg.info}")
        
        if msg.status == "SUCCESS":
            self.get_logger().info(f"Comando {current_command} executado com sucesso!")
            
            # PRIORIDADE: Se estamos visitando bases, processa a lógica de visitação primeiro
            if self.visiting_bases:
                # Verifica se estamos na fase ON_GROUND aguardando timer
                if self.visit_phase == "ON_GROUND" and self.visit_timer_started:
                    # Estamos aguardando tempo no solo - não faz nada, deixa o timer controlar
                    self.get_logger().info("Aguardando término do tempo no solo...")
                    self.waiting_for_response = False
                elif self.processing_visit_transition:
                    # Já estamos processando uma transição - ignora callbacks adicionais
                    self.get_logger().debug("Transição já em andamento, ignorando callback...")
                    self.waiting_for_response = False
                else:
                    # Completou fase atual - avança para próxima fase
                    self.processing_visit_transition = True
                    self.get_logger().info(f"Fase {self.visit_phase} concluída!")
                    self.waiting_for_response = False
                    self.send_next_base_visit()  # Avança para próxima fase
            
            # Lógica especial para comandos customizados (apenas se NÃO estamos visitando bases)
            elif current_command == "PASSEAR_ARENA" and not self.passeando_arena:
                # Inicia passeio pela arena (apenas na primeira vez)
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
                
            elif current_command == "WAIT_FOR_BASES":
                if self.bases_received:
                    self.get_logger().info("Bases detectadas! Iniciando visitação...")
                    self.mission_index += 1
                    self.waiting_for_response = False
                    self.send_next_command()
                else:
                    # Ainda aguardando bases - continua no mesmo comando
                    self.waiting_for_response = False
                    return
                    
            elif current_command == "VISIT_DETECTED_BASES":
                if not self.bases_received:
                    self.get_logger().error("Nenhuma base detectada! Pulando visitação...")
                    self.mission_index += 1
                    self.waiting_for_response = False
                    self.send_next_command()
                else:
                    # Ordena bases por proximidade antes de iniciar visitação
                    # Posição inicial assumida como última posição do passeio
                    last_arena_position = self.arena_waypoints[-1]
                    current_pos = [last_arena_position['x'], last_arena_position['y']]
                    self.order_bases_by_proximity(current_pos)
                    
                    # Inicia processo de visitação
                    self.visiting_bases = True
                    self.current_base_index = 0
                    self.waiting_for_response = False
                    self.get_logger().info(f"Iniciando visitação de {len(self.detected_bases)} bases (modo: {self.visit_type})")
                    self.send_next_base_visit()
             
            else:
                # Comando normal
                self.mission_index += 1
                self.waiting_for_response = False
                # Pequeno delay antes do próximo comando
                self.create_timer(0.05, self.send_next_command_delayed)
            
        elif msg.status == "FAILED":
            self.get_logger().error(f"Comando {current_command} falhou: {msg.info}")
            
            # PRIORIDADE: Se estamos visitando bases, trata falha na visitação primeiro
            if self.visiting_bases:
                self.get_logger().error(f"Falha ao visitar Base {self.detected_bases[self.current_base_index]['id']}")
                # Tenta próxima base
                self.current_base_index += 1
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
            
        # Se status é ONGOING, continua aguardando
        elif msg.status == "ONGOING":
            # PRIORIDADE: Durante visitação de bases, ONGOING pode significar que chegou na base
            if self.visiting_bases and not self.visit_timer_started:
                # Verifica se comando é GOTO para uma base
                if "GOTO" in msg.info or True:  # Aceita qualquer ONGOING durante visitação
                    self.get_logger().info("Drone chegou na base! Iniciando timer de visitação...")
                    self.visit_timer_started = True
                    self.waiting_for_response = False
                    # Inicia verificação de tempo
                    self.create_timer(1.0, lambda: self.check_visit_completion())
            
            # Para outros casos, apenas continua aguardando

    def send_next_command_delayed(self):
        """Envia próximo comando após delay (chamado uma vez apenas)"""
        self.send_next_command()

    def check_visit_completion(self):
        """Verifica se o tempo de visitação da base atual foi completado."""
        if not self.visiting_bases or self.waiting_for_response:
            return

        current_base = self.detected_bases[self.current_base_index]
        elapsed_time = time.time() - self.current_visit_start_time

        if elapsed_time >= self.visit_duration:
            self.get_logger().info(f"Base {current_base['id']} visitada completamente! ({elapsed_time:.1f}s)")
            self.current_base_index += 1
            self.visit_timer_started = False  # Reset flag para próxima base
            self.send_next_base_visit()
        else:
            remaining_time = self.visit_duration - elapsed_time
            self.get_logger().info(f"Visitando Base {current_base['id']} - restam {remaining_time:.1f}s")
            # Cria timer para verificar novamente (apenas se ainda visitando)
            if self.visiting_bases:
                self.create_timer(1.0, lambda: self.check_visit_completion())

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
