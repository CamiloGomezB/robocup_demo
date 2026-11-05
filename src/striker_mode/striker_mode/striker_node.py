#!/usr/bin/env python3
"""Modo Striker reactivo para Booster K1.

No usa odometria ni localizacion en la cancha. Solo:
  - la vision del demo (/booster_soccer/detection): pelota y postes en el marco del robot,
  - el yaw de la IMU (/low_state) para saber hacia que lado esta el arco rival,
  - el API Loco por RPC (/LocoApiTopicReq) para caminar, mover la cabeza y patear.

Maquina de estados:
  SEARCH    -> no veo la pelota: barro con la cabeza y luego giro el cuerpo.
  ACQUIRE   -> la acabo de ver: me detengo, la fijo con la cabeza y giro hasta tenerla de frente; solo cuando
               la veo de forma estable empiezo a caminar (si no, al arrancar la pierdo).
  APPROACH  -> voy hacia la pelota (lejos); si se me va de lado, primero giro y despues avanzo.
  LOOK_GOAL -> cerca de la pelota y sin datos frescos del arco: levanto la vista para buscar los postes.
  ALIGN     -> orbito alrededor de la pelota hasta que pelota y arco queden alineados.
  DRIBBLE   -> arco lejos: conduzco la pelota caminando hacia el arco.
  KICK      -> pelota en la zona de tiro (o se acabo el tiempo de conduccion): pateo.

Pelota afuera: con las lineas de la cancha (/booster_soccer/line_segments) y el yaw de la IMU busco
una linea de borde (alineada con los ejes de la cancha y con lineas solo hacia un lado). Si la pelota
queda del lado sin lineas, esta afuera: me alineo y la pateo de vuelta hacia adentro.

Activacion (control_source):
  topic  -> sim: arranca/para con /booster_agent/soccer_game_control ("play"/"stop").
  remote -> robot real: SOLO con el control remoto. Mantener LB+RB+Y para activar, LB+RB+X para desactivar;
            mover cualquier stick tambien desactiva (el operador toma el control).
Seguridad en el robot real (restore_walking_mode: false): el Striker nunca cambia el modo del robot. Si el
robot sale de Walking (p. ej. LT+BACK = amortiguacion de emergencia), se cae, o la vision deja de publicar,
el Striker se desactiva y deja de mandar movimiento.
"""
import signal
import json
import math
import uuid
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from booster_msgs.msg import RpcReqMsg, RpcRespMsg
from booster_interface.msg import LowState, Odometer, RawBytesMsg, RemoteControllerState
from .goal_tracker import GoalTracker, in_goal_box, route_around_goal
from vision_interface.msg import Detections, LineSegments
from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

API_CHANGE_MODE = 2000
API_MOVE = 2001
API_ROTATE_HEAD = 2004
API_GET_STATUS = 2018
API_VISUAL_KICK = 2038
MODE_WALKING = 2

SEARCH, ACQUIRE, APPROACH, LOOK_GOAL, ALIGN, DRIBBLE, KICK = (
    'SEARCH', 'ACQUIRE', 'APPROACH', 'LOOK_GOAL', 'ALIGN', 'DRIBBLE', 'KICK')

# Barrido de cabeza (pitch, yaw) para buscar la pelota: pasos cortos para que el barrido sea suave
# (lejos/horizonte de izquierda a derecha, y de vuelta mirando mas cerca)
SCAN_PATTERN = [(0.35, -1.0), (0.35, -0.5), (0.35, 0.0), (0.35, 0.5), (0.35, 1.0),
                (0.7, 1.0), (0.7, 0.5), (0.7, 0.0), (0.7, -0.5), (0.7, -1.0)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


class StrikerNode(Node):
    def __init__(self):
        super().__init__('striker_node')
        p = self.declare_parameter
        self.autostart = p('autostart', False).value
        # Seguridad / activacion (los valores por defecto son los del sim; el robot real los cambia por YAML)
        self.control_source = p('control_source', 'topic').value  # topic (sim) | remote (control remoto, real)
        self.restore_walking_mode = p('restore_walking_mode', True).value  # False en el real: nunca cambio el modo
        self.stop_on_vision_loss = p('stop_on_vision_loss', False).value  # True en el real
        self.vision_timeout = p('vision_timeout', 1.0).value  # s sin detecciones -> me detengo (si stop_on_vision_loss)
        self.remote_hold_time = p('remote_hold_time', 1.0).value  # s manteniendo LB+RB+Y para activar
        self.stick_deadzone = p('stick_deadzone', 0.1).value  # stick mas alla de esto = el operador toma el control
        self.attack_offset = p('attack_offset', math.pi / 2).value  # rumbo del arco rival respecto al rumbo al dar play
        self.attack_yaw_abs = p('attack_yaw', 999.0).value  # rumbo absoluto del arco rival en el marco de la IMU (999 = usar offset)
        self.kick_mode = p('kick_mode', 'walk').value  # walk | visual
        if self.kick_mode == 'visual' and not self.restore_walking_mode:
            # La patada visual del firmware necesita cambiar de modo (kSoccer) y depende de la version:
            # en el robot real (sin cambios de modo) no se usa
            self.get_logger().warn('kick_mode visual no se permite con restore_walking_mode false: uso walk')
            self.kick_mode = 'walk'
        self.ball_conf_min = p('ball_conf_min', 50.0).value
        self.ball_conf_low = p('ball_conf_low', 20.0).value  # detecciones debiles: solo cerca de la pelota que ya seguia
        self.ball_gate = p('ball_gate', 0.8).value  # m: de cerca, cuanto puede moverse la pelota entre cuadros
        self.ball_gate_yaw = p('ball_gate_yaw', 0.15).value  # rad: de lejos, diferencia de angulo tolerada
        self.ball_gate_range = p('ball_gate_range', 0.35).value  # de lejos, diferencia relativa de distancia tolerada
        self.candidate_hold = p('candidate_hold', 1.2).value  # s: en la busqueda, cuanto me quedo mirando un candidato
        self.post_conf_min = p('post_conf_min', 30.0).value
        self.ball_memory = p('ball_memory', 1.5).value
        self.ball_memory_close = p('ball_memory_close', 3.0).value  # la pelota a los pies sale del campo de vision
        self.goal_fresh = p('goal_fresh', 4.0).value  # s: el arco cuenta como "visto hace poco"
        self.goal_init_cone = p('goal_init_cone', 1.0).value  # rad: para registrar el arco rival la primera vez
        self.goal_near_range = p('goal_near_range', 7.0).value  # m: postes mas cerca corrigen la distancia al arco
        self.odom_factor = p('odom_factor', 1.0).value  # escala de /odometer_state (sim ~1.0; el demo usa 0.8 en el K1 real)
        # Acomodar la pelota frente al arco cuando quedo muy de costado o pasada la linea de fondo
        self.setup_dist = p('setup_dist', 3.0).value  # m delante del arco: punto al que llevo la pelota
        self.setup_max_angle = p('setup_max_angle', 0.6).value  # rad: angulo de tiro peor que esto -> acomodo
        self.setup_exit_angle = p('setup_exit_angle', 0.4).value  # rad: con angulo mejor que esto dejo de acomodar
        self.setup_front = p('setup_front', 0.8).value  # m: pelota a menos de esto de la linea de fondo -> acomodo
        self.setup_align_tol = p('setup_align_tol', 0.5).value  # rad: alineacion suficiente para acomodar
        self.align_timeout = p('align_timeout', 8.0).value  # s rodeando sin lograrlo -> duplico la tolerancia
        self.goal_width = p('goal_width', 2.6).value
        self.field_length = p('field_length', 14.0).value  # solo para el diagnostico del sim
        self.post_height = p('post_height', 1.8).value  # m: altura del poste (confirma la distancia por su tamano)
        self.cam_fy = p('camera_fy', 216.5).value  # se actualiza con camera_info
        self.robot_height = p('robot_height', 0.8).value
        self.vx_max = p('vx_max', 0.8).value
        self.vy_max = p('vy_max', 0.4).value
        self.vyaw_max = p('vyaw_max', 1.0).value
        self.min_vx = p('min_vx', 0.3).value
        self.min_vy = p('min_vy', 0.3).value
        self.min_vyaw = p('min_vyaw', 0.25).value
        self.near_dist = p('near_dist', 1.0).value  # a partir de aqui alineo
        self.align_dist = p('align_dist', 0.5).value  # distancia a la que me quedo detras de la pelota
        self.align_tol = p('align_tol', 0.25).value  # rad entre (yo->pelota) y (pelota->arco)
        self.shoot_dist_min = p('shoot_dist_min', 2.5).value  # pelota mas cerca del arco: la empujo, no pateo
        self.shoot_dist_max = p('shoot_dist_max', 6.0).value  # pelota mas lejos del arco: conduzco
        self.shoot_align_tol = p('shoot_align_tol', 0.12).value  # alineacion fina antes de patear (rad)
        self.dribble_touch_time = p('dribble_touch_time', 1.2).value  # duracion de cada toque al conducir
        self.quick_kick_tol = p('quick_kick_tol', 0.2).value  # "patearlo de una" si el error es menor
        self.dribble_speed = p('dribble_speed', 0.4).value
        self.dribble_max_time = p('dribble_max_time', 20.0).value
        self.kick_speed = p('kick_speed', 1.0).value
        self.kick_time = p('kick_time', 1.5).value
        self.visual_kick_time = p('visual_kick_time', 5.0).value
        self.scan_step_time = p('scan_step_time', 0.7).value  # tiempo quieto en cada punto del barrido
        self.acquire_time = p('acquire_time', 1.5).value  # al verla: quieto este tiempo confirmandola antes de caminar
        self.acquire_hits = p('acquire_hits', 4).value  # detecciones minimas en ese tiempo
        self.approach_turn_tol = p('approach_turn_tol', 0.4).value  # rad: mas de costado que esto, giro sin avanzar
        self.approach_speed = p('approach_speed', 0.5).value  # velocidad maxima yendo hacia la pelota
        self.look_back_time = p('look_back_time', 2.0).value  # al perderla: quieto mirando donde la vi
        self.search_turn_speed = p('search_turn_speed', 0.3).value  # giro lento entre barridos
        self.search_turn_step = p('search_turn_step', math.radians(45)).value  # cuanto gira entre barridos (IMU)
        self.search_turn_timeout = p('search_turn_timeout', 8.0).value  # por si la IMU no ayuda
        self.search_settle_time = p('search_settle_time', 0.8).value  # quieto tras girar antes de barrer
        self.acquire_turn_speed = p('acquire_turn_speed', 0.3).value  # giro maximo mientras fijo la pelota
        self.search_walk_dist = p('search_walk_dist', 2.0).value  # m que camina si una vuelta completa no la encuentra
        self.shot_settle_time = p('shot_settle_time', 1.0).value  # quieto y alineado este tiempo antes del tiro
        self.shot_goal_fresh = p('shot_goal_fresh', 2.0).value  # el arco tiene que haberse visto hace menos de esto
        self.shot_speed = p('shot_speed', 0.7).value  # patada en la zona de tiro (los toques al conducir son mas suaves)
        self.line_min_len = p('line_min_len', 0.6).value  # segmentos mas cortos se ignoran
        self.out_margin = p('out_margin', 0.4).value  # cuanto tiene que pasar la pelota la linea de borde
        self.out_line_min_len = p('out_line_min_len', 1.2).value  # largo minimo de un segmento para ser borde
        self.out_confirm = p('out_confirm', 3).value  # detecciones seguidas para dar la pelota por afuera
        self.out_memory = p('out_memory', 6.0).value
        self.recover_attack_blend = p('recover_attack_blend', 0.5).value  # al devolverla, cuanto apunto hacia el arco rival

        self.pub = self.create_publisher(RpcReqMsg, 'LocoApiTopicReq', 10)
        self.create_subscription(Detections, '/booster_soccer/detection', self.on_detections, 10)
        self.setup_ball_fallback()
        self.create_subscription(LowState, '/low_state', self.on_low_state, 10)
        self.create_subscription(Odometer, '/odometer_state', self.on_odom, 10)
        self.create_subscription(LineSegments, '/booster_soccer/line_segments', self.on_lines, 1)
        self.create_subscription(RpcRespMsg, 'LocoApiTopicResp', self.on_rpc_resp, 10)
        # Diagnostico solo para el sim: compara la punteria con la pose real del robot (no se usa para decidir)
        self.debug_sim = p('debug_sim', False).value
        self.sim_pose = None
        self.diag = None
        self.last_aim = None
        if self.debug_sim:
            self.create_subscription(Pose2D, '/soccer/sim/localization/robot_pose', self.on_sim_pose, 10)
        self.status_uuid = None
        self.status_misses = 0  # consultas de estado seguidas sin respuesta
        self.robot_mode = None  # ultimo modo informado por kGetStatus (2 = Walking)
        self.robot_mode_t = -100.0
        # Vigilo el modo del robot: en el sim lo devuelvo a Walking si quedo en Prepare (al reiniciar la escena);
        # en el real (restore_walking_mode: false) si sale de Walking me desactivo y no lo toco
        self.create_timer(1.0 if not self.restore_walking_mode else 2.0, self.check_mode)
        self.create_subscription(String, '/booster_agent/soccer_game_control', self.on_command, 1)
        # Control remoto (activar/desactivar) y estado de caida del robot
        self.create_subscription(RemoteControllerState, '/remote_controller_state', self.on_remote, 10)
        self.create_subscription(RawBytesMsg, 'fall_down_recovery_state', self.on_fall_state, 10)
        self.fall_state = 0  # 0 listo, 1 cayendo, 2 caido, 3 levantandose (RobotRecoveryState del firmware)
        self.activate_press_t = None  # desde cuando se mantiene LB+RB+Y
        self.activation_pending = None  # pedi activar: espero que el robot confirme que esta en Walking

        self.playing = False
        self.yaw = None
        self.attack_yaw = None
        self.ball = None  # (x, y, t)
        self.goal = GoalTracker(self.goal_width / 2, self.goal_near_range, self.goal_init_cone)
        self.odom = None  # (x, y, theta) de /odometer_state
        self.setup_active = False  # llevando la pelota frente al arco antes de tirar
        self.goal_scored_t = -100.0
        self.ball_out = None  # (direccion hacia adentro en el marco de la cancha, t)
        self.out_count = 0
        self.state = SEARCH
        self.state_t = self.now()
        self.scan_i = 0
        self.search_back = False
        self.look_back = None  # (pitch, yaw) donde vi la pelota por ultima vez
        self.ball_candidate = None  # (x, y, t) algo que parece la pelota, aun sin confirmar
        self.hold_start = -100.0
        self.ball_hits = deque(maxlen=50)  # momentos en que la vision (o el respaldo) confirmo la pelota
        self.search_turn_target = None  # yaw (IMU) al que estoy girando entre barridos
        self.search_turn_start = 0.0
        self.search_turns = 0  # giros hechos en esta busqueda (vuelta completa -> camino a otro lugar)
        self.search_walk = None
        self.last_look = -100.0
        self.last_kick_end = -100.0
        self.last_det_msg_t = None  # ultima vez que la vision publico detecciones (aunque vengan vacias)
        self.last_vision_warn = -100.0
        self.orbit_dir = None  # sentido de orbita fijo cuando quedo del lado equivocado de la pelota
        self.aligned_since = None  # desde cuando estoy alineado y quieto preparando el tiro
        self.next_kick_speed = self.kick_speed
        self.cur_kick_speed = self.kick_speed
        self.scan_t = self.now()
        self.dribble_t = None
        self.dribble_acc = 0.0  # tiempo total conduciendo desde la ultima patada
        self.kick_started = False
        self.last_head = (None, None, 0.0)
        self.last_status = 0.0

        self.create_timer(0.1, self.tick)
        if self.control_source == 'remote':
            self.get_logger().info(
                f'Striker listo (kick_mode={self.kick_mode}). Control remoto: mantener LB+RB+Y '
                f'{self.remote_hold_time:.0f} s para ACTIVAR, LB+RB+X para DESACTIVAR (mover un stick tambien lo '
                'desactiva). El robot tiene que estar en Walking. Emergencia del firmware: LT+BACK.')
        else:
            self.get_logger().info(f'Striker listo (kick_mode={self.kick_mode}). Esperando "play".')

    # ---------------------------------------------------------------- utilidades
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rpc(self, api_id, body):
        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())
        msg.header = json.dumps({'api_id': api_id})
        msg.body = json.dumps(body)
        self.pub.publish(msg)
        return msg.uuid

    def on_sim_pose(self, msg):
        self.sim_pose = (msg.x, msg.y, msg.theta)

    def ball_to_field(self, bx, by):
        x, y, th = self.sim_pose
        return (x + bx * math.cos(th) - by * math.sin(th), y + bx * math.sin(th) + by * math.cos(th))

    def diag_kick_start(self):
        if not self.debug_sim or self.sim_pose is None or self.last_aim is None:
            return
        g, kick_dir, err, byaw, bx, by, gdist = self.last_aim
        x, y, th = self.sim_pose
        goal_x = self.field_length / 2
        bfx, bfy = self.ball_to_field(bx, by)
        g_true = wrap(math.atan2(-y, goal_x - x) - th)
        ideal_f = math.atan2(-bfy, goal_x - bfx)  # pelota -> centro del arco, marco de la cancha
        kick_true = wrap(ideal_f - th)
        self.diag = {'t0': self.now(), 'ball0': (bfx, bfy), 'ideal': ideal_f, 'aim_f': wrap(kick_dir + th)}
        self.get_logger().info(
            'DIAG patada: robot=(%.2f,%.2f,%.0fdeg) pelota=(%.2f,%.2f) | arco est=%.0fdeg real=%.0fdeg dist_est=%s | '
            'apunto est=%.0fdeg ideal=%.0fdeg (error de punteria %.0fdeg) | err=%.0fdeg byaw=%.0fdeg' % (
                x, y, math.degrees(th), bfx, bfy, math.degrees(g), math.degrees(g_true),
                '?' if gdist is None else f'{gdist:.1f}', math.degrees(kick_dir), math.degrees(kick_true),
                math.degrees(wrap(kick_dir - kick_true)), math.degrees(err), math.degrees(byaw)))

    def diag_kick_result(self, t):
        if self.diag is None or self.sim_pose is None or self.ball is None:
            return
        if t - self.diag['t0'] < self.kick_time + 1.5 or t - self.ball[2] > 0.3:
            return
        bfx, bfy = self.ball_to_field(self.ball[0], self.ball[1])
        x0, y0 = self.diag['ball0']
        moved = math.hypot(bfx - x0, bfy - y0)
        if moved > 0.3:
            out = math.atan2(bfy - y0, bfx - x0)
            self.get_logger().info(
                'DIAG resultado: la pelota se movio %.1f m hacia %.0fdeg | apunte a %.0fdeg, ideal %.0fdeg '
                '(desvio de la patada %.0fdeg, desvio total %.0fdeg)' % (
                    moved, math.degrees(out), math.degrees(self.diag['aim_f']), math.degrees(self.diag['ideal']),
                    math.degrees(wrap(out - self.diag['aim_f'])), math.degrees(wrap(out - self.diag['ideal']))))
        else:
            self.get_logger().info('DIAG resultado: la pelota casi no se movio (%.2f m)' % moved)
        self.diag = None

    def check_mode(self):
        if not self.playing or (self.state == KICK and self.restore_walking_mode):
            return
        if self.status_uuid is not None:
            self.status_misses += 1
            if not self.restore_walking_mode:
                # Robot real: si no puedo confirmar que sigue en Walking, no sigo mandando movimiento
                if self.status_misses >= 3:
                    self.deactivate('el robot no responde la consulta de modo')
                    return
            elif self.status_misses >= 2:
                # Sim: la consulta quedo sin respuesta (pasa tras reiniciar la escena): por las dudas pido
                # Walking; si ya esta caminando no cambia nada, y si quedo en Prepare lo destraba
                self.get_logger().warn('El robot no responde el estado: le pido Walking por si quedo en Prepare')
                self.rpc(API_CHANGE_MODE, {'mode': MODE_WALKING})
                self.status_misses = 0
        self.status_uuid = self.rpc(API_GET_STATUS, {})

    def on_rpc_resp(self, msg):
        if msg.uuid != self.status_uuid:
            return
        self.status_uuid = None
        self.status_misses = 0
        try:
            mode = json.loads(msg.body).get('current_mode')
        except (ValueError, AttributeError):
            return
        if mode is None:
            return
        self.robot_mode, self.robot_mode_t = mode, self.now()
        if self.activation_pending is not None:
            self.finish_activation(mode)
            return
        if mode != MODE_WALKING and self.playing:
            if not self.restore_walking_mode:
                # Robot real: el operador (o el firmware) lo saco de Walking -> me quito del medio, NO lo devuelvo
                self.deactivate(f'el robot salio de Walking (modo {mode})')
            else:
                self.get_logger().warn(f'Robot en modo {mode}, no en Walking: lo cambio a Walking')
                self.rpc(API_CHANGE_MODE, {'mode': MODE_WALKING})
                self.set_state(SEARCH)

    # ---------------------------------------------------------------- activacion / desactivacion
    def on_remote(self, msg):
        """Control remoto (solo con control_source: remote).
        Activar: mantener LB+RB+Y remote_hold_time. Desactivar: LB+RB+X, o mover cualquier stick."""
        if self.control_source != 'remote':
            return
        t = self.now()
        sticks = max(abs(msg.lx), abs(msg.ly), abs(msg.rx), abs(msg.ry))
        if self.playing:
            if msg.lb and msg.rb and msg.x:
                self.deactivate('LB+RB+X en el control')
            elif sticks > self.stick_deadzone:
                self.deactivate('el operador movio un stick (toma manual)')
            return
        combo = msg.lb and msg.rb and msg.y and not (msg.lt or msg.rt or msg.x)
        if not combo or sticks > self.stick_deadzone:
            self.activate_press_t = None
            return
        if self.activate_press_t is None:
            self.activate_press_t = t
        elif t - self.activate_press_t >= self.remote_hold_time and self.activation_pending is None:
            self.activate_press_t = None
            self.request_activation()

    def request_activation(self):
        """Antes de activar confirmo con el robot que esta en Walking (nunca lo cambio de modo yo)."""
        if self.fall_state != 0:
            self.get_logger().warn('No activo el Striker: el robot esta caido o levantandose')
            return
        if self.yaw is None or self.odom is None:
            self.get_logger().warn('No activo el Striker: todavia no llegan la IMU (/low_state) o la odometria')
            return
        self.activation_pending = self.now()
        self.status_uuid = self.rpc(API_GET_STATUS, {})
        self.get_logger().info('LB+RB+Y: consulto el modo del robot antes de activar...')

    def finish_activation(self, mode):
        self.activation_pending = None
        if mode != MODE_WALKING:
            self.get_logger().warn(
                f'No activo el Striker: el robot esta en modo {mode}, no en Walking. '
                'Ponlo a caminar con el control (PREP y luego WALK) y vuelve a intentar.')
            return
        if self.fall_state != 0:
            self.get_logger().warn('No activo el Striker: el robot esta caido o levantandose')
            return
        self.start()

    def deactivate(self, reason):
        """Me detengo y dejo de mandar movimiento. No cambio el modo del robot."""
        was = self.playing
        self.playing = False
        self.activation_pending = None
        for _ in range(3):
            self.rpc(API_MOVE, {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0})
        self.rpc(API_ROTATE_HEAD, {'pitch': 0.0, 'yaw': 0.0})
        self.last_head = (None, None, 0.0)
        self.set_state(SEARCH)
        if was:
            self.get_logger().warn(f'STRIKER DESACTIVADO: {reason}. Velocidad 0.')

    def on_fall_state(self, msg):
        """fall_down_recovery_state: primer byte = 0 listo, 1 cayendo, 2 caido, 3 levantandose."""
        if not msg.msg:
            return
        b = msg.msg[0]  # char[]: segun la version de ROS 2 llega como str o como int
        state = ord(b) if isinstance(b, str) else (b[0] if isinstance(b, (bytes, bytearray)) else int(b))
        if state != self.fall_state and state != 0 and self.playing:
            self.deactivate({1: 'el robot se esta cayendo', 2: 'el robot se cayo',
                             3: 'el robot se esta levantando'}.get(state, f'estado de caida {state}'))
        self.fall_state = state

    def move(self, vx, vy, vyaw):
        def floor(v, vmin):
            if abs(v) < 0.05:
                return 0.0
            return math.copysign(max(abs(v), vmin), v)
        vx = floor(clamp(vx, self.vx_max), self.min_vx)
        vy = floor(clamp(vy, self.vy_max), self.min_vy)
        vyaw = floor(clamp(vyaw, self.vyaw_max), self.min_vyaw)
        self.rpc(API_MOVE, {'vx': vx, 'vy': vy, 'vyaw': vyaw})

    def head(self, pitch, yaw):
        pitch = max(0.0, min(1.0, pitch))
        yaw = clamp(yaw, 1.1)
        lp, ly, lt = self.last_head
        t = self.now()
        if lp is not None and abs(lp - pitch) < 0.05 and abs(ly - yaw) < 0.05 and t - lt < 1.0:
            return
        self.last_head = (pitch, yaw, t)
        self.rpc(API_ROTATE_HEAD, {'pitch': pitch, 'yaw': yaw})

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[{self.state} -> {s}]')
            if self.state == KICK:
                self.last_kick_end = self.now()
            self.state = s
            self.state_t = self.now()
            self.orbit_dir = None
            self.aligned_since = None
            if self.dribble_t is not None:
                self.dribble_acc += self.now() - self.dribble_t
                self.dribble_t = None
            if s == DRIBBLE:
                self.dribble_t = self.now()
            if s == KICK:
                self.kick_started = False
                self.dribble_acc = 0.0
                self.cur_kick_speed = self.next_kick_speed
                self.diag_kick_start()
            if s == LOOK_GOAL:
                self.last_look = self.now()
            if s == SEARCH:
                self.search_turn_target = None
                self.search_turns = 0
                self.search_walk = None
                self.scan_t = self.now()

    # ---------------------------------------------------------------- callbacks
    def on_command(self, msg):
        if msg.data == 'play':
            if self.control_source != 'topic':
                self.get_logger().warn('Ignoro "play" por topico: en este modo solo se activa con el control (LB+RB+Y)')
                return
            self.start()
        elif msg.data in ('stop', 'ready', 'locate'):
            # Detenerse siempre se acepta, venga de donde venga
            self.deactivate(f'"{msg.data}" por topico')

    def initial_attack_yaw(self):
        if abs(self.attack_yaw_abs) < 10.0:
            return self.attack_yaw_abs
        return None if self.yaw is None else wrap(self.yaw + self.attack_offset)

    def start(self):
        self.playing = True
        self.attack_yaw = self.initial_attack_yaw()
        self.goal.reset()
        self.setup_active = False
        self.ball = None
        self.status_uuid, self.status_misses = None, 0
        self.last_det_msg_t = self.now()  # el plazo de la vigilancia de vision empieza al activar
        if self.restore_walking_mode:
            self.rpc(API_CHANGE_MODE, {'mode': MODE_WALKING})  # solo en el sim; en el real nunca cambio el modo
        self.set_state(SEARCH)
        self.get_logger().info('STRIKER ACTIVADO (%s): rumbo de ataque = %s' % (
            'control remoto' if self.control_source == 'remote' else 'topico',
            'desconocido (sin IMU)' if self.attack_yaw is None else f'{self.attack_yaw:.2f} rad'))

    def on_low_state(self, msg):
        self.yaw = float(msg.imu_state.rpy[2])
        if self.playing and self.attack_yaw is None:
            self.attack_yaw = self.initial_attack_yaw()

    def on_detections(self, msg):
        t = self.now()
        best = None
        posts = []
        self.last_det_msg_t = t
        weak = []
        for obj in msg.detected_objects:
            if len(obj.position_projection) < 2:
                continue
            x, y = float(obj.position_projection[0]), float(obj.position_projection[1])
            if obj.label == 'Ball' and -0.5 < x < 15.0:
                if obj.confidence >= self.ball_conf_min:
                    if best is None or obj.confidence > best[2]:
                        best = (x, y, obj.confidence)
                elif obj.confidence >= self.ball_conf_low:
                    weak.append((x, y, obj.confidence))
            elif obj.label == 'Goalpost' and obj.confidence >= self.post_conf_min and x > 0.0:
                posts.append((x, y, obj.confidence, self.post_range_reliable(obj, math.hypot(x, y))))
        if best is None and weak:
            ref = self.ball if self.ball is not None and t - self.ball[2] < 1.0 else None
            if ref is not None:
                # Cuadro malo (pelota borrosa, lejos, en un piso distinto): acepto una deteccion debil
                # solo si es coherente con la pelota que venia siguiendo
                near = min(weak, key=lambda q: self.ball_mismatch(q, ref))
                if self.ball_mismatch(near, ref) < 1.0:
                    best = near
            else:
                self.note_candidate(max(weak, key=lambda q: q[2]), t)  # en la busqueda: me detengo a mirarla
        if best:
            self.ball = (best[0], best[1], t)
            self.yolo_ball_t = t
            self.ball_hits.append(t)
        self.update_goal_tracker(posts, t)

    # ---------------------------------------------------------------- pelota: detector de respaldo
    def setup_ball_fallback(self):
        """Detector clasico de la pelota sobre la imagen de la camara, para cuando YOLO no la ve
        (otro piso, pelota lejos y chica, mala iluminacion). Opcional: sin topico de imagen no se activa."""
        p = self.declare_parameter
        self.fb = None
        self.yolo_ball_t = -100.0
        self.fb_last = None  # ultimo candidato (x, y, t) para pedir dos cuadros coherentes
        self.fb_used_t = -100.0
        self.head2base = None
        self.fb_last_run = 0.0
        image_topic = p('fallback_image_topic', '').value
        info_topic = p('fallback_camera_info_topic', '').value
        self.fb_rate = p('fallback_rate', 8.0).value
        self.fb_yolo_grace = p('fallback_yolo_grace', 0.4).value  # si YOLO vio la pelota hace menos, no corro el respaldo
        if info_topic:  # intrinsecos: los usa el respaldo y el chequeo de distancia de los postes
            self.create_subscription(CameraInfo, info_topic, self.on_camera_info, 1)
        if not image_topic:
            return
        try:
            import yaml
            from ament_index_python.packages import get_package_share_directory
            from .ball_fallback import BallFallbackDetector
            cfg = p('fallback_vision_config', '').value or \
                get_package_share_directory('vision') + '/config/vision.yaml'
            with open(cfg) as f:
                extrin = yaml.safe_load(f)['camera']['extrin']
            self.fb = BallFallbackDetector(extrin, p('ball_radius', 0.11).value)
        except Exception as e:  # sin OpenCV o sin calibracion: sigo solo con YOLO
            self.get_logger().warn(f'Detector de respaldo de la pelota desactivado: {e}')
            return
        self.create_subscription(Pose, '/head_pose', self.on_head_pose, 10)
        if 'compressed' in image_topic:
            self.create_subscription(CompressedImage, image_topic, self.on_image_compressed, 1)
        else:
            self.create_subscription(Image, image_topic, self.on_image_raw, 1)
        self.get_logger().info(f'Detector de respaldo de la pelota activo sobre {image_topic}')

    def on_camera_info(self, msg):
        self.cam_fy = msg.k[4]
        if self.fb is not None:
            self.fb.set_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def post_range_reliable(self, obj, rng):
        """La distancia proyectada a un poste lejano puede salir hasta ~50 % mal (un poste a 12 m se mide a 6 m).
        La confirmo con otra medicion independiente: la altura del poste en la imagen (mide post_height)."""
        if rng >= self.goal_near_range:
            return False
        if obj.ymin <= 1:
            return rng < 4.0  # el poste se sale por arriba de la imagen: esta cerca de verdad
        h_px = max(1, obj.ymax - obj.ymin)
        rng_h = self.cam_fy * self.post_height / h_px
        return abs(rng_h - rng) / rng < 0.35

    def on_head_pose(self, msg):
        from .ball_fallback import quat_to_rot
        import numpy as np
        o = msg.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(o.x, o.y, o.z, o.w)
        T[:3, 3] = (msg.position.x, msg.position.y, msg.position.z)
        self.head2base = T

    def on_image_compressed(self, msg):
        import cv2
        import numpy as np
        if self.fallback_due():
            bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is not None:
                self.run_fallback(bgr)

    def on_image_raw(self, msg):
        import cv2
        import numpy as np
        if not self.fallback_due() or msg.encoding not in ('bgr8', 'rgb8'):
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        self.run_fallback(img if msg.encoding == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    def fallback_due(self):
        t = self.now()
        if (self.fb is None or self.fb.K is None or self.head2base is None
                or t - self.yolo_ball_t < self.fb_yolo_grace or t - self.fb_last_run < 1.0 / self.fb_rate):
            return False
        self.fb_last_run = t
        return True

    def run_fallback(self, bgr):
        t = self.now()
        cands = self.fb.detect(bgr, self.head2base)
        if not cands:
            return
        # Prefiero el candidato mas cerca de la pelota que ya seguia; si no, el de mejor puntaje
        ref = self.fb_last if self.fb_last and t - self.fb_last[2] < 0.8 else \
            (self.ball if self.ball and t - self.ball[2] < 1.0 else None)
        c = min(cands, key=lambda q: self.ball_mismatch(q, ref)) if ref else cands[0]
        prev, self.fb_last = self.fb_last, (c[0], c[1], t)
        self.note_candidate(c, t)
        # Dos cuadros seguidos coherentes (o coherente con la pelota que ya seguia) antes de creerle
        if ref is None or self.ball_mismatch(c, ref) > 1.0:
            return
        if prev is None and not (self.ball and t - self.ball[2] < 1.0):
            return
        self.ball = (c[0], c[1], t)
        self.ball_hits.append(t)
        if t - self.fb_used_t > 3.0:
            self.get_logger().info(f'Pelota por detector de respaldo: ({c[0]:.2f}, {c[1]:.2f}) m')
        self.fb_used_t = t

    def ball_mismatch(self, a, b):
        """Que tan distintas son dos estimaciones de la pelota (<1 = coherentes). Se compara en angulo y en
        distancia relativa: de lejos un error minimo del angulo de la cabeza mueve la distancia casi un metro,
        asi que una tolerancia fija en metros rechazaria pelotas lejanas buenas."""
        ra, rb = math.hypot(a[0], a[1]), math.hypot(b[0], b[1])
        dyaw = abs(wrap(math.atan2(a[1], a[0]) - math.atan2(b[1], b[0])))
        drng = abs(ra - rb) / max(0.5, min(ra, rb))
        near = math.hypot(a[0] - b[0], a[1] - b[1]) < self.ball_gate  # de cerca alcanza con estar cerca
        return 0.0 if near else max(dyaw / self.ball_gate_yaw, drng / self.ball_gate_range)

    def note_candidate(self, c, t):
        """Vi algo que parece la pelota (sin confirmar): durante la busqueda me quedo mirandolo."""
        if self.ball_candidate is None or t - self.ball_candidate[2] > 3.0:
            self.hold_start = t  # candidato nuevo (no lo venia viendo)
        self.ball_candidate = (c[0], c[1], t)

    def on_odom(self, msg):
        new = (msg.x * self.odom_factor, msg.y * self.odom_factor, float(msg.theta))
        if self.odom is not None and math.hypot(new[0] - self.odom[0], new[1] - self.odom[1]) > 1.0:
            # Salto de odometria (escena reiniciada / robot reubicado): el arco recordado ya no vale
            self.get_logger().info('Salto de odometria: olvido el arco registrado')
            self.goal.reset()
        self.odom = new

    def field_dir_odom(self):
        """Angulo del eje x de la cancha (hacia el arco rival) en el marco de odometria."""
        psi = self.field_heading()
        return None if psi is None or self.odom is None else wrap(self.odom[2] - psi)

    def update_goal_tracker(self, posts, t):
        fd = self.field_dir_odom()
        if not posts or fd is None:
            return
        had_dir, had_pos = self.goal.far_b is not None, self.goal.c is not None
        self.goal.update(posts, self.odom, fd, t)
        if not had_pos and self.goal.c is not None:
            g, d = self.goal.relative(self.odom)
            self.get_logger().info(f'Arco rival ubicado de cerca: {math.degrees(g):.0f} grados, {d:.1f} m')
        elif not had_dir and self.goal.far_b is not None:
            g, _ = self.goal.relative(self.odom)
            self.get_logger().info(f'Arco rival visto de lejos: {math.degrees(g):.0f} grados (solo direccion)')

    def field_heading(self):
        """Rumbo del robot en el marco de la cancha (eje x = hacia el arco rival)."""
        if self.yaw is None or self.attack_yaw is None:
            return None
        return wrap(self.yaw - self.attack_yaw)

    def on_lines(self, msg):
        psi = self.field_heading()
        t = self.now()
        if psi is None or self.ball is None or t - self.ball[2] > 0.5:
            return
        c, s = math.cos(psi), math.sin(psi)

        def to_field(x, y):  # marco del robot -> marco alineado con la cancha (centrado en el robot)
            return (c * x - s * y, s * x + c * y)

        segs = []
        co = msg.coordinates
        for i in range(len(co) // 4):
            p0 = to_field(co[4 * i], co[4 * i + 1])
            p1 = to_field(co[4 * i + 2], co[4 * i + 3])
            if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < self.line_min_len:
                continue
            ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            if abs(math.sin(ang)) < 0.25:
                axis = 1  # paralela al eje x de la cancha (banda): y constante
            elif abs(math.cos(ang)) < 0.25:
                axis = 0  # paralela al eje y (fondo / medio / area): x constante
            else:
                axis = None
            segs.append((axis, p0, p1))

        ball_f = to_field(self.ball[0], self.ball[1])
        if math.hypot(*ball_f) > 5.0:
            return
        out_dir = None
        arcs = [((q0[0] + q1[0]) / 2, (q0[1] + q1[1]) / 2) for a, q0, q1 in segs if a is None]
        for i, (axis, p0, p1) in enumerate(segs):
            if axis is None or math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < self.out_line_min_len:
                continue  # los bordes son lineas largas
            v = (p0[axis] + p1[axis]) / 2
            # Linea de medio campo: la cruza el circulo central (segmentos curvos cerca)
            if any(abs(m[axis] - v) < 1.5 for m in arcs):
                continue
            # Del lado de la cancha hay mas lineas; del lado de afuera no hay ninguna
            sides = [q[axis] - v for j, (_, q0, q1) in enumerate(segs) if j != i for q in (q0, q1)]
            sides = [d for d in sides if abs(d) > 0.4]
            if len(sides) < 2 or (min(sides) < 0 < max(sides)):
                continue  # sin evidencia, o linea interior (hay lineas a ambos lados)
            field_sign = 1.0 if sides[0] > 0 else -1.0
            # Solo confio en el lado que veo: yo del lado de la cancha y la pelota mas alla del borde.
            # (Las lineas detras o debajo del robot no se ven, asi que "sin lineas" de mi lado no prueba nada.)
            robot_inside = (0.0 - v) * field_sign > 0.2
            if robot_inside and (ball_f[axis] - v) * field_sign < -self.out_margin:
                out_dir = (0.0 if field_sign > 0 else math.pi) if axis == 0 else math.copysign(math.pi / 2, field_sign)
                break
        if out_dir is None:
            self.out_count = 0
            return
        self.out_count += 1
        if self.out_count >= self.out_confirm:
            if self.ball_out is None:
                self.get_logger().info(f'PELOTA AFUERA: la devuelvo hacia {math.degrees(out_dir):.0f} grados (cancha)')
            self.ball_out = (out_dir, t)

    # ---------------------------------------------------------------- decision
    def goal_rel(self, t):
        """(direccion del arco en el marco del robot, distancia o None, dato fresco?)"""
        yaw = self.yaw if self.yaw is not None else 0.0
        rel = self.goal.relative(self.odom) if self.odom is not None else None
        if rel is not None:
            # La distancia solo es confiable si alguna vez lo vi de cerca; si no, solo la direccion
            dist = rel[1] if self.goal.t_near > -1e8 else None
            return rel[0], dist, t - self.goal.t < self.goal_fresh
        if self.attack_yaw is not None:
            return wrap(self.attack_yaw - yaw), None, False
        return 0.0, None, False

    def tick(self):
        if self.activation_pending is not None and self.now() - self.activation_pending > 2.0:
            self.activation_pending = None
            self.get_logger().warn('No activo el Striker: el robot no respondio la consulta de modo')
        if not self.playing:
            if self.autostart and self.yaw is not None and self.control_source == 'topic':
                self.start()
            return
        t = self.now()
        # Si la vision deja de publicar (topico de camara equivocado, nodo caido) lo digo en vez de buscar a ciegas
        silent = t - self.last_det_msg_t if self.last_det_msg_t is not None else t - self.state_t
        if self.stop_on_vision_loss and silent > self.vision_timeout:
            # Robot real: sin vision no se a donde voy -> me detengo
            self.deactivate(f'la vision no publica detecciones hace {silent:.1f} s')
            return
        if silent > 3.0 and t - self.last_vision_warn > 5.0:
            self.last_vision_warn = t
            self.get_logger().warn(f'La vision no publica detecciones hace {silent:.0f} s: revisa vision.log y el topico de la camara')
        if self.state == LOOK_GOAL and self.ball is not None:
            # Mientras miro el arco la pelota sale de la imagen: asumo que sigue donde estaba
            self.ball = (self.ball[0], self.ball[1], t)
        ball_close = self.ball is not None and math.hypot(self.ball[0], self.ball[1]) < 0.6
        memory = self.ball_memory_close if ball_close else self.ball_memory
        ball_ok = self.ball is not None and t - self.ball[2] < memory
        if self.state == KICK:
            self.do_kick(t, ball_ok)
            return
        if not ball_ok:
            if self.state != SEARCH:
                self.search_back = ball_close  # la perdi a los pies: primero miro abajo y retrocedo
                # Primero miro hacia donde la vi por ultima vez, quieto, antes de barrer
                self.look_back = None if self.ball is None else (
                    math.atan2(self.robot_height, max(0.3, math.hypot(self.ball[0], self.ball[1]))),
                    clamp(math.atan2(self.ball[1], self.ball[0]), 1.1))
            self.set_state(SEARCH)
            self.do_search(t)
            self.status(t, None, None)
            return

        bx, by, _ = self.ball
        rng = math.hypot(bx, by)
        byaw = math.atan2(by, bx)
        g, gdist, gfresh = self.goal_rel(t)
        # Direccion pelota -> arco (si conozco la distancia al arco la uso; si no, la direccion basta)
        if gdist is not None:
            kick_dir = math.atan2(gdist * math.sin(g) - by, gdist * math.cos(g) - bx)
            goal_from_ball = math.hypot(gdist * math.cos(g) - bx, gdist * math.sin(g) - by)
        else:
            kick_dir, goal_from_ball = g, None
        err = wrap(kick_dir - byaw)  # 0 = estoy justo detras de la pelota mirando al arco
        # Zona de tiro: ni tan lejos que el tiro tenga que ser perfecto, ni pegado al arco (ahi se empuja)
        in_zone = goal_from_ball is not None and self.shoot_dist_min <= goal_from_ball <= self.shoot_dist_max
        dribbled = self.dribble_acc + (t - self.dribble_t if self.dribble_t else 0.0)
        # Si nunca veo el arco, despues de mucho conducir pateo igual hacia el rumbo de ataque
        shoot = in_zone or (goal_from_ball is None and dribbled > self.dribble_max_time)
        tol = self.shoot_align_tol if shoot else self.align_tol

        # Pelota afuera: la devuelvo hacia adentro (perpendicular al borde, algo inclinada hacia el arco rival)
        recovering = self.ball_out is not None and t - self.ball_out[1] < self.out_memory
        psi = self.field_heading()
        if recovering and psi is not None:
            n = self.ball_out[0]
            vx_f = math.cos(n) + self.recover_attack_blend
            vy_f = math.sin(n)
            kick_dir = wrap(math.atan2(vy_f, vx_f) - psi)
            err = wrap(kick_dir - byaw)
            in_zone, shoot, tol = False, True, self.align_tol
        else:
            recovering = False

        # Pelota mal ubicada respecto del arco (muy de costado, o pasada la linea de fondo al lado del arco):
        # antes de tirar la llevo a un punto frente al arco. Trabajo en el marco de la cancha (IMU), con el
        # arco que recuerda el seguidor aunque no lo este viendo.
        tag = 'AFUERA' if recovering else ''
        if not recovering and gdist is not None and psi is not None:
            cp, sp = math.cos(psi), math.sin(psi)
            gx_r, gy_r = gdist * math.cos(g), gdist * math.sin(g)
            gfx, gfy = cp * gx_r - sp * gy_r, sp * gx_r + cp * gy_r
            bfx, bfy = cp * bx - sp * by, sp * bx + cp * by
            dx, dy = bfx - gfx, bfy - gfy  # pelota respecto del centro del arco (el arco abre hacia -x)
            mouth = self.goal_width / 2 - 0.2
            confirmed = t - self.goal.t_pair < 5.0  # arco bien ubicado (postes de los dos lados hace poco)
            in_goal = -0.25 < dx < 0.3 and abs(dy) < mouth  # entre la linea y la red del fondo (no detras de la red)
            if not in_goal and self.goal_scored_t >= 0 and t - self.goal_scored_t >= 15.0:
                self.goal_scored_t = -100.0  # la pelota ya salio del arco (p. ej. escena reiniciada): listo para otro
            if in_goal and confirmed and self.goal_scored_t < 0:
                # Pelota adentro del arco
                self.goal_scored_t = t
                where = ''
                if self.debug_sim and self.sim_pose is not None:
                    wx, wy = self.ball_to_field(bx, by)
                    inside = self.field_length / 2 < wx < self.field_length / 2 + 0.6 and abs(wy) < self.goal_width / 2
                    where = ' | pelota real en (%.2f, %.2f): %s' % (wx, wy, 'adentro del arco' if inside else 'NO esta adentro')
                self.get_logger().info('GOL!' + where)
        if 0 <= self.goal_scored_t and t - self.goal_scored_t < 15.0:
            # Despues de un gol me quedo quieto (como mucho 15 s, por si me equivoque), aunque la deteccion parpadee
            self.move(0.0, 0.0, 0.0)
            self.head(math.atan2(self.robot_height, rng), clamp(byaw, 1.1))
            self.status(t, rng, err, gdist, 'GOL')
            return
        if not recovering and gdist is not None and psi is not None:
            shot_angle = math.atan2(abs(dy), max(-dx, 1e-3))  # 0 = justo enfrente del arco
            beside = dx > -self.setup_front and abs(dy) > mouth  # junto a la linea de fondo, al costado del arco
            if beside or dx > 0.7 or shot_angle > self.setup_max_angle:
                self.setup_active = True
            elif dx < -1.5 and shot_angle < self.setup_exit_angle:
                self.setup_active = False
            if self.setup_active:
                px, py = -self.setup_dist, 0.0  # punto frente al arco (relativo al centro del arco)
                if math.hypot(px - dx, py - dy) < 0.8:
                    self.setup_active = False  # ya quedo frente al arco
                else:
                    # Si la pelota esta detras del arco no la empujo contra la red: la llevo primero al costado
                    tx, ty = route_around_goal((dx, dy), (px, py))
                    kick_dir = wrap(math.atan2(ty - dy, tx - dx) - psi)  # conduzco hacia el punto frente al arco
                    err = wrap(kick_dir - byaw)
                    # Para acomodar no hace falta precision: solo llevarla a un mejor lugar
                    in_zone, shoot, tol = False, False, self.setup_align_tol
                    tag = 'ACOMODO'
        if not tag and in_zone:
            tag = 'ZONA'
        # Tiro al arco con mas fuerza; devolver la pelota a la cancha, mas suave
        self.next_kick_speed = self.shot_speed if in_zone else self.kick_speed
        self.last_aim = (g, kick_dir, err, byaw, bx, by, gdist)
        self.diag_kick_result(t)

        if self.state == SEARCH:
            self.set_state(ACQUIRE)  # la vi: antes de caminar me aseguro de que de verdad esta ahi

        if self.state == ACQUIRE:
            self.head(math.atan2(self.robot_height, rng), clamp(byaw, 1.1))
            # Giro en el lugar (sin avanzar) hasta tenerla de frente
            self.move(0.0, 0.0, clamp(0.8 * byaw, self.acquire_turn_speed) if abs(byaw) > 0.15 else 0.0)
            hits = sum(1 for h in self.ball_hits if t - h < self.acquire_time)
            if (t - self.state_t > self.acquire_time and hits >= self.acquire_hits
                    and abs(byaw) < self.approach_turn_tol):
                self.set_state(APPROACH if rng >= self.near_dist else ALIGN)

        elif self.state == APPROACH:
            waypoint = self.goal_waypoint(g, gdist, psi, bx, by)
            if waypoint is not None:
                # El arco esta entre la pelota y yo (o estoy adentro del arco): lo rodeo por afuera
                wx, wy = waypoint
                wyaw = math.atan2(wy, wx)
                self.head(math.atan2(self.robot_height, rng), clamp(byaw, 1.1))
                if abs(wyaw) > self.approach_turn_tol:
                    self.move(0.0, 0.0, clamp(0.8 * wyaw, self.acquire_turn_speed))
                else:
                    self.move(min(self.approach_speed, 0.5 * math.hypot(wx, wy) + 0.2), 0.0, 1.0 * wyaw)
                tag = tag or 'RODEO ARCO'
            elif rng < self.near_dist:
                # "Patearlo de una" solo para devolver la pelota a la cancha; el tiro al arco se prepara en ALIGN
                if (recovering and abs(err) < self.quick_kick_tol and abs(byaw) < 0.4
                        and t - self.last_kick_end > 2.0):
                    self.set_state(KICK)
                    return
                self.set_state(ALIGN if gfresh or recovering else LOOK_GOAL)
            else:
                self.head(math.atan2(self.robot_height, rng), byaw)
                if abs(byaw) > self.approach_turn_tol:
                    # Se me fue de lado: dejo de avanzar y giro en el lugar (caminando la pierdo de vista)
                    self.move(0.0, 0.0, clamp(0.8 * byaw, self.acquire_turn_speed))
                else:
                    # Avanzo mas lento cuanto mas de costado la tengo, y freno al acercarme
                    vx = min(self.approach_speed, 0.5 * (rng - 0.4)) * (1.0 - 0.6 * abs(byaw) / self.approach_turn_tol)
                    self.move(vx, 0.0, 1.0 * byaw)

        elif self.state == LOOK_GOAL:
            # Barro alrededor del arco hasta ver los dos postes juntos (con uno solo no se donde esta el centro)
            self.move(0.0, 0.0, 0.0)
            el = t - self.state_t
            sweep = (0.0, -0.6, 0.6)
            self.head(0.15, clamp(g + sweep[min(int(el / 0.8), 2)], 1.1))
            if t - self.goal.t_pair < 0.5 or el > 2.4:
                self.set_state(ALIGN)

        elif self.state == ALIGN:
            if self.goal_waypoint(g, gdist, psi, bx, by, only_inside=True) is not None:
                self.set_state(APPROACH)  # me meti adentro del arco: primero salgo (APPROACH lo rodea)
                return
            if t - self.state_t > self.align_timeout and not in_zone:
                # Llevo mucho rodeandola sin lograrlo (postes, red o paredes en el camino): me conformo con menos
                tol *= 2.0
            aligned = abs(err) < tol and abs(byaw) < 0.25 and rng < self.align_dist + 0.3
            # Para tirar exijo haber visto de cerca postes de los DOS lados hace poco: con uno solo no se si el
            # centro esta a su izquierda o a su derecha (un tiro salio 49 grados desviado por eso)
            goal_age = t - self.goal.t_pair
            if rng > self.near_dist + 0.4:
                self.set_state(APPROACH)
            elif aligned and not shoot:
                self.set_state(DRIBBLE)
            elif aligned and in_zone and goal_age > self.shot_goal_fresh and t - self.last_look > 2.0:
                self.set_state(LOOK_GOAL)  # antes de tirar confirmo donde esta el arco
            elif aligned:
                # Preparo el tiro: quieto y alineado durante shot_settle_time antes de patear
                self.head(math.atan2(self.robot_height, rng), byaw)
                self.move(0.0, 0.0, 0.0)
                if self.aligned_since is None:
                    self.aligned_since = t
                if not in_zone or t - self.aligned_since > self.shot_settle_time:
                    self.set_state(KICK)
            else:
                self.aligned_since = None
                self.head(math.atan2(self.robot_height, rng), byaw)
                # Si quede del lado equivocado (err ~ +-pi), el signo de err salta de un ciclo a otro y el
                # robot gira sin rumbo: elijo un sentido de orbita y lo mantengo hasta quedar detras de la pelota
                if abs(err) > 2.2 and self.orbit_dir is None:
                    self.orbit_dir = -1.0 if err > 0 else 1.0
                if self.orbit_dir is not None and abs(err) < 1.2:
                    self.orbit_dir = None
                if self.orbit_dir is not None:
                    vy = self.orbit_dir * self.vy_max
                    radius = self.align_dist + 0.2  # orbito un poco mas abierto para no tocarla
                else:
                    vy = -0.8 * err
                    radius = self.align_dist
                vx = 0.6 * (rng - radius)
                if in_zone:  # en la zona de tiro me acomodo con mas calma
                    vy = clamp(0.6 * vy, 0.25)
                    vx = clamp(vx, 0.15)
                self.move(clamp(vx, 0.25), vy, 1.5 * byaw)

        elif self.state == DRIBBLE:
            if rng > self.near_dist or abs(by) > 0.3 or abs(err) > 0.8:
                self.set_state(APPROACH)
            elif shoot:
                self.set_state(ALIGN)  # entre a la zona de tiro: me alineo fino y pateo
            elif not gfresh and abs(g) < 1.2 and t - self.state_t > self.dribble_touch_time:
                self.set_state(LOOK_GOAL)  # despues de cada toque levanto la vista para medir el arco
            else:
                self.head(math.atan2(self.robot_height, rng), byaw)
                self.move(self.dribble_speed, -0.4 * err, 1.5 * byaw + 0.5 * err)

        self.status(t, rng, err, gdist, tag)

    def goal_waypoint(self, g, gdist, psi, bx, by, only_inside=False):
        """Si para llegar a la pelota tengo que atravesar el arco (o estoy adentro), devuelve el punto intermedio
        (en el marco del robot) por donde rodearlo; si el camino esta libre, None."""
        if gdist is None or psi is None or self.goal.t_near < -1e8:
            return None
        cp, sp = math.cos(psi), math.sin(psi)
        gfx = cp * gdist * math.cos(g) - sp * gdist * math.sin(g)
        gfy = sp * gdist * math.cos(g) + cp * gdist * math.sin(g)
        robot = (-gfx, -gfy)  # yo, relativo al centro del arco (marco de la cancha)
        if only_inside and not in_goal_box(robot):
            return None
        ball = (cp * bx - sp * by - gfx, sp * bx + cp * by - gfy)
        goal_to = ball
        if in_goal_box(ball):
            # La pelota esta pegada al arco (detras de la red o junto a un poste): voy a un punto justo afuera
            # de la caja, del lado donde esta, y desde ahi directo a ella
            if ball[0] > 0.3:
                goal_to = (0.65 + 0.25, ball[1])
            elif abs(ball[1]) > 1.3:
                goal_to = (ball[0], math.copysign(1.75 + 0.25, ball[1]))
            else:
                goal_to = (-0.5 - 0.25, ball[1])
            if math.hypot(goal_to[0] - robot[0], goal_to[1] - robot[1]) < 0.35:
                return None  # ya estoy al lado: el ultimo tramo es directo
        tgt = route_around_goal(robot, goal_to)
        if tgt == ball:
            return None  # camino directo y libre hacia la pelota
        wfx, wfy = tgt[0] + gfx, tgt[1] + gfy  # relativo a mi, marco de la cancha
        return (cp * wfx + sp * wfy, -sp * wfx + cp * wfy)  # al marco del robot

    def do_search(self, t):
        if self.search_back and t - self.state_t < 1.5:
            self.head(1.0, 0.0)
            self.move(-0.3, 0.0, 0.0)
            return
        if self.look_back is not None and t - self.state_t < self.look_back_time:
            self.head(*self.look_back)
            self.move(0.0, 0.0, 0.0)
            return
        cand = self.ball_candidate
        # Lo miro mientras siga apareciendo, pero con tope: un falso positivo que nunca se confirma no me frena
        if cand is not None and t - cand[2] < self.candidate_hold and t - self.hold_start < 2 * self.candidate_hold:
            # Vi algo que parece la pelota: freno todo y lo miro fijo hasta confirmarlo (o hasta que se pase el tiempo)
            self.search_turn_target = None
            self.move(0.0, 0.0, 0.0)
            self.head(math.atan2(self.robot_height, max(0.3, math.hypot(cand[0], cand[1]))),
                      clamp(math.atan2(cand[1], cand[0]), 1.1))
            self.scan_t = t  # el barrido sigue desde donde estaba cuando termine de mirar
            return
        # 1) Quieto, barro con la cabeza parando en cada punto lo suficiente para que la vision (lenta en CPU)
        #    procese varias imagenes. 2) Si no la vi, giro el cuerpo search_turn_step (45 grados) despacio,
        #    medido con la IMU, me detengo y vuelvo a barrer.
        if self.do_search_walk(t):
            return
        if self.search_turn_target is not None:
            remaining = wrap(self.search_turn_target - self.yaw) if self.yaw is not None else 0.0
            if abs(remaining) > 0.08 and t - self.search_turn_start < self.search_turn_timeout:
                self.head(0.5, 0.0)
                self.move(0.0, 0.0, math.copysign(self.search_turn_speed, remaining))
                return
            self.search_turn_target = None
            self.scan_t = t + self.search_settle_time  # quieto un momento antes de volver a barrer
        self.move(0.0, 0.0, 0.0)
        if t - self.scan_t > self.scan_step_time:
            self.scan_t = t
            self.scan_i += 1
            if self.scan_i % len(SCAN_PATTERN) == 0 and self.yaw is not None:
                self.search_turns += 1
                if (self.search_turns * self.search_turn_step >= 2 * math.pi and self.odom is not None
                        and self.search_walk_dist > 0):
                    self.start_search_walk(t)  # vuelta completa sin verla: la busco desde otro lugar
                else:
                    self.search_turn_target = wrap(self.yaw + self.search_turn_step)
                    self.search_turn_start = t
        self.head(*SCAN_PATTERN[self.scan_i % len(SCAN_PATTERN)])

    def start_search_walk(self, t):
        """Camino search_walk_dist hacia el frente del arco rival (si lo recuerdo) o hacia el lado de ataque."""
        self.search_turns = 0
        rel = self.goal.relative(self.odom)
        if rel is not None and rel[1] is not None:
            g, d = rel
            psi = self.field_heading() or 0.0
            # punto setup_dist delante del arco, en el marco del robot
            px = d * math.cos(g) - self.setup_dist * math.cos(-psi)
            py = d * math.sin(g) - self.setup_dist * math.sin(-psi)
            heading = math.atan2(py, px)
        elif rel is not None:
            heading = rel[0]  # solo se la direccion al arco: camino hacia alla
        else:
            heading = wrap(self.attack_yaw - self.yaw) if self.attack_yaw is not None else 0.0
        self.search_walk = {'yaw': wrap(self.yaw + heading), 'start': self.odom[:2], 't': t}
        self.get_logger().info(f'No encuentro la pelota desde aca: camino {self.search_walk_dist:.1f} m y busco de nuevo')

    def do_search_walk(self, t):
        """Devuelve True mientras estoy caminando a otro lugar para seguir buscando."""
        w = self.search_walk
        if w is None:
            return False
        walked = math.hypot(self.odom[0] - w['start'][0], self.odom[1] - w['start'][1])
        if walked > self.search_walk_dist or t - w['t'] > 15.0:
            self.search_walk = None
            self.move(0.0, 0.0, 0.0)
            self.scan_t = t + self.search_settle_time
            return False
        e = wrap(w['yaw'] - self.yaw)
        self.head(0.4, 0.0)
        if abs(e) > 0.3:
            self.move(0.0, 0.0, math.copysign(self.search_turn_speed, e))
        else:
            self.move(0.4, 0.0, 0.8 * e)
        return True

    def do_kick(self, t, ball_ok):
        el = t - self.state_t
        if self.kick_mode == 'visual':
            if not self.kick_started:
                self.move(0.0, 0.0, 0.0)
                self.rpc(API_VISUAL_KICK, {'start': True})
                self.kick_started = True
                self.get_logger().info('KICK: kVisualKick start')
            elif el > self.visual_kick_time:
                self.rpc(API_VISUAL_KICK, {'start': False})
                if self.restore_walking_mode:
                    self.rpc(API_CHANGE_MODE, {'mode': MODE_WALKING})
                self.ball_out = None
                self.set_state(APPROACH)
            return
        # Patada caminando: acelero a traves de la pelota (como el Kick del demo)
        if el > self.kick_time:
            self.move(0.0, 0.0, 0.0)
            self.ball_out = None
            self.set_state(APPROACH)
            return
        self.head(0.8, 0.0)
        a = math.atan2(self.ball[1], self.ball[0]) if ball_ok else 0.0
        speed = min(self.cur_kick_speed, 0.3 + 0.6 * el)
        self.move(speed * math.cos(a), speed * math.sin(a), 0.0)

    def status(self, t, rng, err, gdist=None, tag=''):
        if t - self.last_status < 1.0:
            return
        self.last_status = t
        if self.debug_sim and self.sim_pose is not None and self.odom is not None and int(t) % 5 == 0:
            rel = self.goal.relative(self.odom)
            if rel is not None and rel[1] is None:
                x, y, th = self.sim_pose
                real_b = wrap(math.atan2(-y, self.field_length / 2 + 0.3 - x) - th)
                self.get_logger().info('DIAG arco (solo direccion, de lejos): %.0f vs real %.0f grados' % (
                    math.degrees(rel[0]), math.degrees(real_b)))
            if rel is not None and rel[1] is not None:
                x, y, th = self.sim_pose
                ex, ey = x + rel[1] * math.cos(th + rel[0]), y + rel[1] * math.sin(th + rel[0])
                rx, ry = self.field_length / 2 + 0.3, 0.0  # centro real del arco rival (con la red)
                real_b = wrap(math.atan2(ry - y, rx - x) - th)
                self.get_logger().info(
                    'DIAG arco recordado=(%.1f,%.1f) real=(%.1f,%.1f): error %.1f m, direccion %.0f vs %.0f grados%s' % (
                        ex, ey, rx, ry, math.hypot(ex - rx, ey - ry), math.degrees(rel[0]), math.degrees(real_b),
                        '' if self.goal.t_near > -1e8 else ' (aun sin verlo de cerca)'))
        b = 'no' if rng is None else f'{rng:.2f}m'
        e = '-' if err is None else f'{err:+.2f}'
        g = '?' if gdist is None else f'{gdist:.1f}m'
        self.get_logger().info(f'[{self.state}] pelota={b} err={e} arco={g} {tag}'.rstrip())


def main():
    # Manejo yo SIGINT/SIGTERM (no el de ROS) para que el contexto siga vivo y alcance a salir la parada
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()

    def _stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    node = StrikerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Al cerrar: velocidad 0 (varias veces, por si se pierde alguna) antes de soltar el robot
        node.playing = False
        for _ in range(5):
            node.rpc(API_MOVE, {'vx': 0.0, 'vy': 0.0, 'vyaw': 0.0})
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info('Striker cerrado: velocidad 0 enviada')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
