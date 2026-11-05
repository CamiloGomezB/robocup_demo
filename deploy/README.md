# Despliegue del modo Striker en el Booster K1 real

Esta carpeta lleva el **modo Striker** (`src/striker_mode`) a un Booster K1 real. La prioridad es la
**seguridad del robot**.

## Qué hace y qué no hace

- **Hace:** usa la cámara de la cabeza para ver la pelota y el arco, y le pide al firmware que camine
  (`kMove`) y mueva la cabeza (`kRotateHead`). La caminata es **la del firmware**: no hay caminata propia, ni
  RL, ni modo Custom.
- **Patea empujando la pelota al caminar.** No usa la patada visual del firmware ni cambia de modo.
- **Nunca cambia el modo del robot.** El operador pone el robot a caminar con el control y lo activa. Si el
  robot sale de Walking por cualquier motivo, el Striker se desactiva solo.
- **No esquiva obstáculos ni personas.** Siempre tiene que haber un operador con el control en la mano.

## Control remoto

| Acción | Botones |
|---|---|
| **Activar** el Striker | Mantener **LB + RB + Y** durante 1 segundo (con el robot ya en Walking) |
| **Desactivar** el Striker | **LB + RB + X** (inmediato) |
| Tomar el control a mano | Mover **cualquier stick**: el Striker se desactiva y manda el operador |
| **Emergencia** (firmware) | **LT + BACK** → amortiguación. El Striker lo detecta y se desactiva |
| Preparación / caminar (firmware) | **LT + START** (PREP) y luego **RT + A** (WALK)* |

\* Según el manual del K1; algunas versiones usan **LT + RT + A** para caminar. Confírmalo en el manual de tu
versión de firmware.

El Striker **se desactiva solo** (velocidad 0, cabeza al frente) si:
- el robot sale de Walking (amortiguación, preparación, etc.),
- el robot se cae, se está cayendo o se está levantando (`/fall_down_recovery_state`),
- la visión deja de publicar detecciones por más de 1 s,
- el robot no responde la consulta de modo,
- se cierra el programa (Ctrl+C, `striker_stop.sh` o una señal del sistema).

Después de desactivarse no vuelve a moverse hasta que se lo active de nuevo con **LB + RB + Y**.

## Requisitos

- Booster K1 con firmware reciente y la cámara de la cabeza publicando en `/boostercamera/head/rgb`. Si tu K1
  usa otros tópicos, pásalos por variables de entorno (ver más abajo).
- En el robot: Ubuntu con ROS 2 Humble y JetPack 6.2 / TensorRT 10.3. Ya vienen en el robot. Los modelos
  `*_10.3.engine` del repo son para esa versión de TensorRT.
- Espacio libre de al menos 3 × 3 m, sin personas cerca del robot, y un operador con el control.

## Instalación en el robot (una sola vez)

Conéctate al robot por SSH y clona el fork **en el robot**. No copies los archivos desde Windows: los
scripts necesitan fin de línea de Linux.

```bash
cd ~/Workspace
git clone <URL-de-tu-fork> robocup_striker
cd robocup_striker
git lfs install && git lfs pull        # los modelos de visión están en Git LFS
sudo apt-get install -y ros-humble-backward-ros   # dependencia del repo (ver README principal)
./scripts/build.sh --packages-up-to vision striker_mode
```

`scripts/build.sh` es el script oficial del repo: compila con CUDA/TensorRT para la Jetson del robot.

## Uso

1. Enciende el robot y ponlo en **PREP** (LT + START) y luego en **WALK** con el control.
2. En el robot:
   ```bash
   cd ~/Workspace/robocup_striker
   ./deploy/striker_start.sh --fg
   ```
   El script detiene cualquier `brain` o visión anterior, corre el **chequeo previo** y arranca la visión y el
   Striker **en espera**. Si el chequeo falla, no arranca nada.
3. Apunta el robot hacia el **arco rival**: al activarlo, ese es el rumbo de ataque.
4. **Activa** con LB + RB + Y (1 s). **Desactiva** con LB + RB + X.
5. Para cerrar todo:
   ```bash
   ./deploy/striker_stop.sh
   ```
   Manda velocidad 0 antes de cerrar los programas. El robot queda en su modo actual, parado.

Sin `--fg` corre en segundo plano y los logs quedan en `deploy/logs/` (`striker.log`, `vision.log`).

### Primera prueba (obligatoria)

1. **Robot sostenido o colgado**, con los pies apenas tocando el suelo:
   - Activa y desactiva varias veces y comprueba que reacciona a LB + RB + Y y a LB + RB + X.
   - Comprueba que mover un stick lo desactiva.
   - Comprueba que LT + BACK lo deja en amortiguación y que el Striker queda desactivado (`STRIKER DESACTIVADO`
     en el log).
2. **En el suelo**, con la pelota a 2–3 m: actívalo y deja el dedo cerca de LB + RB + X.
3. Las velocidades de `src/striker_mode/config/striker_real_k1.yaml` son conservadoras. Súbelas de a poco solo
   cuando todo funcione bien.

### Chequeo por separado

```bash
./deploy/striker_check.sh
```

Solo lee; no mueve el robot. Revisa:
- que los paquetes estén compilados,
- que existan los modelos,
- que el `brain` no esté corriendo (competiría por el control),
- que estén los tópicos de cámara, IMU, odometría, control remoto y caídas,
- en qué modo está el robot.

### Cambiar los tópicos de la cámara

```bash
CAMERA_COLOR=/mi/camara/rgb CAMERA_INFO=/mi/camara/rgb/camera_info ./deploy/striker_start.sh --fg
```

## Configuración

`src/striker_mode/config/striker_real_k1.yaml` se carga encima de `striker.yaml`. Los parámetros de seguridad
que **no deben cambiarse** en el robot real:

| Parámetro | Valor | Para qué |
|---|---|---|
| `control_source` | `remote` | solo el control activa el Striker |
| `restore_walking_mode` | `false` | nunca cambia el modo del robot |
| `stop_on_vision_loss` | `true` | sin visión se detiene |
| `search_walk_dist` | `0.0` | no camina a ciegas buscando la pelota |
| `kick_mode` | `walk` | sin patada visual ni cambios de modo |

## Pendiente de confirmar en el robot real

- Que el firmware de tu K1 no use **LB + RB + Y** ni **LB + RB + X** para otra cosa. El manual oficial v1.6+
  requiere inicio de sesión y no se pudo revisar.
- Los tópicos de la cámara (`/boostercamera/head/*`) y que los `.engine` carguen con la versión de TensorRT del
  robot.
- `odom_factor` (0.8, el valor del demo oficial para el K1).

## Probar el control en el simulador

Ver `sim_docker/`: `CONTROL=remote` arranca el Striker como en el robot real. Ahí se activa publicando
mensajes de control simulados.
