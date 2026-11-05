#!/bin/bash
# Demo 1 vs nadie dentro del contenedor (red compartida con el robot virtual).

LOGDIR=/opt/robocup_demo/logs
mkdir -p "$LOGDIR"
cd /opt/robocup_demo

source /opt/ros/humble/setup.bash
source /opt/robocup_demo/install/setup.bash
if [ -f /opt/booster/BoosterRos2/fastdds_profile_udp_only.xml ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
fi
export PYTHONWARNINGS="ignore"

LOCATE_TIMEOUT=${LOCATE_TIMEOUT:-300}
# MODE=striker: modo Striker reactivo (src/striker_mode), sin brain ni odometria.
# MODE=demo:    brain original del repo.
MODE=${MODE:-striker}
# La patada visual (kVisualKick) no la ejecuta el robot virtual: el brain se queda quieto a <4 m de la pelota.
VISUAL_KICK=${VISUAL_KICK:-false}
KICK_MODE=${KICK_MODE:-walk}
# CONTROL=topic:  el Striker arranca solo con "play" (lo de siempre en el sim).
# CONTROL=remote: igual que en el robot real (config striker_real_k1.yaml): solo se activa con el control
#                 remoto (LB+RB+Y). En el sim se prueba publicando /remote_controller_state a mano.
CONTROL=${CONTROL:-topic}
echo "[MODO] $MODE"

echo "[ESPERANDO CAMARA DEL ROBOT VIRTUAL]"
CAM_INFO=""
while [ -z "$CAM_INFO" ]; do
    CAM_INFO=$(ros2 topic list 2>/dev/null | grep -E 'rgbd_camera/rgb/camera_info$' | head -1)
    [ -z "$CAM_INFO" ] && sleep 3
done
CAM=${CAM_INFO%/rgb/camera_info}
# Espero a que la imagen se anuncie de verdad: al cargar la escena camera_info aparece antes que la imagen,
# y si elijo un topico que no existe la vision se queda esperando cuadros que nunca llegan.
COLOR=""
for i in $(seq 1 40); do
    TOPICS=$(ros2 topic list 2>/dev/null)
    if echo "$TOPICS" | grep -qx "$CAM/rgb/image_compressed"; then
        COLOR="$CAM/rgb/image_compressed"; break
    elif echo "$TOPICS" | grep -qx "$CAM/rgb/image_raw"; then
        COLOR="$CAM/rgb/image_raw"; break
    fi
    sleep 1
done
if [ -z "$COLOR" ]; then
    echo "[ERROR] no aparece ningun topico de imagen en $CAM/rgb/ (image_compressed ni image_raw)"
    exit 1
fi
DEPTH="$CAM/depth/image_raw"
DEPTH_INFO="$CAM/depth/camera_info"
echo "[CAMARA] $CAM  color=$COLOR"

mkdir -p ~/agents/booster_soccer
cat > ~/agents/booster_soccer/brain.yaml <<EOF
brain_node:
  ros__parameters:
    game:
      agent_mode: true
      player_role: "striker"
    RLVisionKick:
      enableAutoVisualKick: $VISUAL_KICK
    vision:
      image_camera_info_topic: "$CAM_INFO"
      depth_image_topic: "$DEPTH"
      depth_camera_info_topic: "$DEPTH_INFO"
EOF

MODEL_DIR=/opt/robocup_demo/src/vision/model
echo "[START VISION]"
ros2 launch vision launch.py sim:=true \
    color_topic:="$COLOR" depth_topic:="$DEPTH" intrin_topic:="$CAM_INFO" \
    detection_model_path:="$MODEL_DIR/sim_data_det_0126.onnx" \
    segmentation_model_path:="$MODEL_DIR/sim_data_seg_0126.onnx" \
    > "$LOGDIR/vision.log" 2>&1 &
# Ojo: save_data:=false hace segfault (DataLogger nulo en VisionNode::Init), se deja en true.
if [ "$MODE" = demo ]; then
    echo "[START BRAIN]"
    ros2 launch brain launch.py sim:=true agent_mode:=true role:=striker disable_com:=true \
        > "$LOGDIR/brain.log" 2>&1 &
else
    echo "[START STRIKER] kick_mode=$KICK_MODE control=$CONTROL"
    if [ "$CONTROL" = remote ]; then
        # Misma configuracion que el robot real (sin cambiar de modo, solo con el control remoto)
        ros2 launch striker_mode striker.launch.py sim:=true config:=striker_real_k1.yaml \
            fallback_image_topic:="$COLOR" fallback_camera_info_topic:="$CAM_INFO" \
            > "$LOGDIR/striker.log" 2>&1 &
    else
        # En el sim el yaw de la IMU coincide con el de la cancha: el arco rival esta en +x (yaw 0)
        ros2 launch striker_mode striker.launch.py sim:=true kick_mode:="$KICK_MODE" attack_yaw:="${ATTACK_YAW:-0.0}" debug_sim:="${DEBUG_SIM:-true}" \
            fallback_image_topic:="$COLOR" fallback_camera_info_topic:="$CAM_INFO" \
            > "$LOGDIR/striker.log" 2>&1 &
    fi
fi
echo "[START GAME_CONTROLLER]"
ros2 launch game_controller launch.py > "$LOGDIR/game_controller.log" 2>&1 &

sleep 10

echo "[MODO WALKING]"
for i in 1 2 3; do
    ros2 topic pub --once /LocoApiTopicReq booster_msgs/msg/RpcReqMsg \
        "{uuid: 'demo-walk-$i', header: '{\"api_id\":2000}', body: '{\"mode\":2}'}" > /dev/null
    sleep 2
done

if [ "$MODE" != demo ]; then
    if [ "$CONTROL" = remote ]; then
        echo "[ESPERANDO EL CONTROL] activar: LB+RB+Y (1 s), desactivar: LB+RB+X"
    else
        echo "[PLAY]"
        ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: play}" > /dev/null
    fi
    echo "[STRIKER CORRIENDO] logs en $LOGDIR. 'docker rm -f robocup-demo' para parar."
    tail -F "$LOGDIR/striker.log"
    exit 0
fi

echo "[LOCATE]"
ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: locate}" > /dev/null
START=$(date +%s)
LOCATED=0
while [ $(( $(date +%s) - START )) -lt "$LOCATE_TIMEOUT" ]; do
    if grep -qE 'SelfLocateEnterField.*(left success: 1|right success 1)' "$LOGDIR/brain.log"; then
        LOCATED=1; break
    fi
    sleep 2
done
if [ "$LOCATED" = 1 ]; then
    echo "[LOCATE] ubicado"
    sleep 5
else
    echo "[LOCATE] no se ubico a tiempo (reinicia la escena y relanza). Paso a PLAY igual."
fi

echo "[PLAY]"
ros2 topic pub --once /booster_agent/soccer_game_control std_msgs/msg/String "{data: play}" > /dev/null

echo "[DEMO CORRIENDO] logs en $LOGDIR. Ctrl+C o 'docker rm -f robocup-demo' para parar."
tail -F "$LOGDIR/brain.log" | grep --line-buffered -E 'self_locate|SelfLocateEnterField|RLVisionKick|Received agent command|ERROR|Error'
