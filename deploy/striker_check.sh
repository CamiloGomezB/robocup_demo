#!/bin/bash
# Chequeo previo (SOLO LECTURA) antes de arrancar el Striker en el K1 real.
# No mueve el robot ni cambia nada: solo revisa que todo lo necesario este disponible.
# Uso (en el robot):  bash deploy/striker_check.sh
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)

source /opt/ros/humble/setup.bash
[ -f "$ROOT/install/setup.bash" ] && source "$ROOT/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

OK=0; WARN=0; FAIL=0
ok()   { echo "  [OK]    $*"; OK=$((OK + 1)); }
warn() { echo "  [AVISO] $*"; WARN=$((WARN + 1)); }
fail() { echo "  [FALLA] $*"; FAIL=$((FAIL + 1)); }

echo "== Compilacion"
for pkg in striker_mode vision booster_interface booster_msgs vision_interface; do
    if [ -d "$ROOT/install/$pkg" ]; then ok "paquete $pkg compilado"; else fail "falta compilar $pkg (ver deploy/README.md)"; fi
done

echo "== Modelos de vision (TensorRT)"
for m in $(grep -E "^\s*model_path:" src/vision/config/vision.yaml | awk '{print $2}'); do
    if [ -f "$ROOT/$m" ]; then ok "modelo $m"; else fail "no existe el modelo $m"; fi
done

echo "== Procesos que compiten por el control del robot"
if pgrep -x brain_node >/dev/null; then fail "brain_node esta corriendo: detenelo (./scripts/stop.sh) antes del Striker"; else ok "brain_node no esta corriendo"; fi
if pgrep -x striker_node >/dev/null; then warn "ya hay un striker_node corriendo"; else ok "no hay otro striker_node"; fi

echo "== Topicos del robot"
TOPICS=$(timeout 15 ros2 topic list 2>/dev/null)
for tp in /boostercamera/head/rgb /boostercamera/head/rgb/camera_info /head_pose /odometer_state /low_state \
          /remote_controller_state /fall_down_recovery_state /LocoApiTopicReq /LocoApiTopicResp; do
    if echo "$TOPICS" | grep -qx "$tp"; then ok "$tp"; else fail "no aparece el topico $tp"; fi
done

echo "== Modo actual del robot (consulta kGetStatus, no cambia nada)"
MODE=$(timeout 20 python3 - <<'EOF' 2>/dev/null
import json, time, uuid
import rclpy
from rclpy.node import Node
from booster_msgs.msg import RpcReqMsg, RpcRespMsg
rclpy.init()
n = Node('striker_check')
got = {}
uid = str(uuid.uuid4())
n.create_subscription(RpcRespMsg, 'LocoApiTopicResp', lambda m: got.setdefault('b', m.body) if m.uuid == uid else None, 10)
pub = n.create_publisher(RpcReqMsg, 'LocoApiTopicReq', 10)
t = time.time()
while time.time() - t < 2.0:
    rclpy.spin_once(n, timeout_sec=0.1)
req = RpcReqMsg(); req.uuid = uid; req.header = json.dumps({'api_id': 2018}); req.body = '{}'
for _ in range(3):
    pub.publish(req)
    t = time.time()
    while time.time() - t < 1.5 and 'b' not in got:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'b' in got:
        break
print(json.loads(got['b']).get('current_mode') if 'b' in got else 'sin_respuesta')
EOF
)
case "$MODE" in
    2) ok "el robot esta en Walking (modo 2): listo para activar con LB+RB+Y" ;;
    0) warn "el robot esta en Damping (0): pasalo a PREP (LT+START) y luego a WALK antes de activar" ;;
    1) warn "el robot esta en Prepare (1): pasalo a WALK antes de activar" ;;
    sin_respuesta|"") fail "el robot no respondio la consulta de modo" ;;
    *) warn "el robot esta en el modo $MODE (el Striker solo se activa en Walking = 2)" ;;
esac

echo
echo "Resultado: $OK ok, $WARN avisos, $FAIL fallas"
[ "$FAIL" -eq 0 ] || exit 1
