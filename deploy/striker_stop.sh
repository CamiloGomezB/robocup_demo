#!/bin/bash
# Detiene el Striker y la vision de forma ordenada en el K1 real:
#   1. manda velocidad 0 al robot (varias veces) para que no siga caminando,
#   2. cierra el Striker con SIGINT (al cerrarse tambien manda velocidad 0),
#   3. cierra la vision, y solo si algo no responde lo fuerza.
# No cambia el modo del robot: queda en Walking parado. Para apagarlo usa el control (LT+START, LT+BACK).
cd "$(dirname "$0")/.." || exit 1

source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

echo "[STOP] velocidad 0 al robot"
for i in 1 2 3; do
    timeout 5 ros2 topic pub --once /LocoApiTopicReq booster_msgs/msg/RpcReqMsg \
        "{uuid: 'striker-stop-$i', header: '{\"api_id\":2001}', body: '{\"vx\":0.0,\"vy\":0.0,\"vyaw\":0.0}'}" \
        >/dev/null 2>&1
done

# $1 = opciones de pgrep/pkill: "-x nombre" (nombre exacto del proceso) o "-f ^patron" (comando anclado al
# comienzo). Nunca un -f suelto: cazaria cualquier consola cuyo comando mencione ese texto.
stop_proc() {  # $1 = opcion, $2 = patron, $3 = nombre legible
    if pgrep "$1" "$2" >/dev/null; then
        echo "[STOP] cierro $3"
        pkill -INT "$1" "$2"
        for _ in $(seq 1 20); do pgrep "$1" "$2" >/dev/null || return 0; sleep 0.25; done
        echo "[STOP] $3 no respondio: lo fuerzo"
        pkill -9 "$1" "$2"
    fi
}
stop_proc -x striker_node "el Striker"
stop_proc -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch striker_mode" "el launch del Striker"
stop_proc -x vision_node "la vision"
stop_proc -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch vision" "el launch de la vision"
echo "[STOP] listo (el robot queda en su modo actual, parado)"
