#!/bin/bash
# Arranca la vision y el Striker en el Booster K1 REAL.
# El Striker queda ESPERANDO: no mueve el robot hasta que el operador mantenga LB+RB+Y en el control
# (con el robot ya en Walking). Desactivar: LB+RB+X o mover un stick. Emergencia: LT+BACK.
# Uso (en el robot):  bash deploy/striker_start.sh            -> logs en deploy/logs/
#                     bash deploy/striker_start.sh --fg       -> deja el log del Striker en pantalla
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)
LOGDIR="$ROOT/deploy/logs"
mkdir -p "$LOGDIR"

# Camara de la cabeza del K1 (firmware reciente). Se puede cambiar sin editar el script:
#   CAMERA_COLOR=/otro/topico bash deploy/striker_start.sh
CAMERA_COLOR=${CAMERA_COLOR:-/boostercamera/head/rgb}
CAMERA_DEPTH=${CAMERA_DEPTH:-/boostercamera/head/depth}
CAMERA_INFO=${CAMERA_INFO:-/boostercamera/head/rgb/camera_info}

echo "[1/4] Detengo nodos previos (brain, vision, striker) para que nadie mas mande ordenes al robot"
bash deploy/striker_stop.sh >/dev/null 2>&1
bash scripts/stop.sh >/dev/null 2>&1

source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash" || { echo "Falta compilar: ver deploy/README.md"; exit 1; }
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

echo "[2/4] Chequeo previo"
if ! bash deploy/striker_check.sh; then
    echo "El chequeo encontro fallas: no arranco el Striker. Revisa los mensajes de arriba."
    exit 1
fi

echo "[3/4] Arranco la vision (camara $CAMERA_COLOR, modelos TensorRT de src/vision/config/vision.yaml)"
# Ojo: no pasar save_data:=false, la vision del repo se cae al arrancar con eso (DataLogger nulo)
nohup ros2 launch vision launch.py \
    color_topic:="$CAMERA_COLOR" depth_topic:="$CAMERA_DEPTH" intrin_topic:="$CAMERA_INFO" \
    > "$LOGDIR/vision.log" 2>&1 &
sleep 5

echo "[4/4] Arranco el Striker (configuracion del robot real: striker_real_k1.yaml)"
STRIKER_CMD=(ros2 launch striker_mode striker.launch.py config:=striker_real_k1.yaml
             fallback_image_topic:="$CAMERA_COLOR" fallback_camera_info_topic:="$CAMERA_INFO")
cat <<'EOF'

  =====================================================================
   STRIKER LISTO Y EN ESPERA (no se mueve hasta que lo actives)
     1. Robot en WALK con el control (LT+START y despues RT+A)
     2. Apunta el robot hacia el arco RIVAL
     3. ACTIVAR:    mantener LB + RB + Y durante 1 segundo
     DESACTIVAR:    LB + RB + X   (o mover cualquier stick)
     EMERGENCIA:    LT + BACK     (amortiguacion del firmware)
     DETENER TODO:  bash deploy/striker_stop.sh
  =====================================================================

EOF
if [ "$1" = "--fg" ]; then
    "${STRIKER_CMD[@]}" 2>&1 | tee "$LOGDIR/striker.log"
else
    nohup "${STRIKER_CMD[@]}" > "$LOGDIR/striker.log" 2>&1 &
    echo "Log del Striker: tail -f $LOGDIR/striker.log"
fi
