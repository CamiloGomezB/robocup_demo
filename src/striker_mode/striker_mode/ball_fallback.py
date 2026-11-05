"""Detector clasico de la pelota (respaldo de YOLO).

YOLO se entreno con pasto verde: en otro piso (gris, iluminacion distinta) o con la pelota lejos y chica
puede no verla. Este detector no depende del piso ni de las lineas: busca una mancha compacta y redonda que
mezcle blanco y negro (los paneles de la pelota) y que se destaque del color dominante del piso alrededor.

La posicion en el piso se calcula con la misma cadena de calibracion que usa la vision del demo:
    T_cam2base = T_head2base (/head_pose) * T_cam2head (camera.extrin de vision.yaml)
y se intersecta el rayo del centro de la pelota con el plano z = radio de la pelota.
"""
import math

import cv2
import numpy as np


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


class BallFallbackDetector:
    def __init__(self, cam2head, ball_radius=0.11):
        self.cam2head = np.asarray(cam2head, dtype=float).reshape(4, 4)
        self.ball_radius = ball_radius
        self.K = None  # (fx, fy, cx, cy)

    def set_intrinsics(self, fx, fy, cx, cy):
        self.K = (fx, fy, cx, cy)

    def project(self, u, v, head2base):
        """Pixel del centro de la pelota -> (x, y) en el marco del robot, y distancia camara-pelota."""
        fx, fy, cx, cy = self.K
        cam2base = head2base @ self.cam2head
        d = cam2base[:3, :3] @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        o = cam2base[:3, 3]
        if d[2] > -1e-3:
            return None  # el rayo no baja hacia el piso
        s = (self.ball_radius - o[2]) / d[2]
        p = o + s * d
        return float(p[0]), float(p[1]), float(s * np.linalg.norm(d))

    def detect(self, bgr, head2base, debug=None):
        """Devuelve candidatos [(x, y, score)] ordenados de mejor a peor.
        debug: lista opcional donde se anota, por cada mancha, su caja y por que se descarto."""
        if self.K is None:
            return []
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hh, ss, vv = hsv[..., 0].astype(np.int16), hsv[..., 1].astype(np.int16), hsv[..., 2].astype(np.int16)

        # Color dominante del piso: mediana de la mitad inferior de la imagen (sea gris, verde o lo que sea)
        lower = hsv[h // 2:, :].reshape(-1, 3)
        fh, fs, fv = (int(np.median(lower[:, i])) for i in range(3))
        hue_diff = np.minimum(np.abs(hh - fh), 180 - np.abs(hh - fh))
        same_color = (np.abs(ss - fs) < 50) & ((fs < 40) | (hue_diff < 15))
        floor_like = (np.abs(vv - fv) < 40) & same_color  # para el anillo de alrededor (tolerante)
        floor_strict = (np.abs(vv - fv) < 22) & same_color  # para separar la pelota del piso

        # La pelota iluminada tiene un brillo blanco y una parte sombreada/paneles oscuros; entre medio hay
        # tonos intermedios, asi que tomo todo lo que no es piso y despues exijo brillo y sombra en la mancha
        white = (vv > max(fv + 45, 180)) & (ss < 80)
        dark = vv < min(fv - 55, 90)
        cand = (~floor_strict).astype(np.uint8)
        cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        n, _, stats, cents = cv2.connectedComponentsWithStats(cand, connectivity=8)

        out = []

        def reject(box, why):
            if debug is not None:
                debug.append((box, why))

        for i in range(1, n):
            x0, y0, bw, bh, area = (int(v) for v in stats[i])
            box = (x0, y0, bw, bh)
            if area < 5 or area > 4000 or bw < 3 or bh < 3:
                if 3 <= area <= 6000:
                    reject(box, 'tamano')
                continue
            if not 0.6 <= bw / bh <= 1.6 or area / float(bw * bh) < 0.45:
                reject(box, 'forma')
                continue  # no es redonda/compacta (postes, lineas, paredes)
            if y0 + bh >= h or ((x0 <= 0 or y0 <= 0 or x0 + bw >= w) and area > 150):
                reject(box, 'borde')
                continue  # borde inferior (brazos/cuerpo del robot) o algo grande cortado: no confiable
            box_w = white[y0:y0 + bh, x0:x0 + bw]
            box_d = dark[y0:y0 + bh, x0:x0 + bw]
            if box_w.mean() < 0.04 or box_d.mean() < 0.04:
                reject(box, 'sin blanco/negro (w=%.2f d=%.2f)' % (box_w.mean(), box_d.mean()))
                continue  # la pelota tiene brillo y sombra (o paneles negros) a la vez
            # Alrededor tiene que haber piso (descarta cosas en las paredes o en el cielo)
            m = max(2, max(bw, bh) // 2)
            ya, yb, xa, xb = max(0, y0 - m), min(h, y0 + bh + m), max(0, x0 - m), min(w, x0 + bw + m)
            ring = floor_like[ya:yb, xa:xb].copy()
            ring[y0 - ya:y0 - ya + bh, x0 - xa:x0 - xa + bw] = False
            ring_n = (yb - ya) * (xb - xa) - bw * bh
            ring_frac = ring.sum() / float(max(ring_n, 1))
            if ring_frac < 0.5:
                reject(box, 'sin piso alrededor (%.2f)' % ring_frac)
                continue
            u, v = float(cents[i][0]), float(cents[i][1])
            proj = self.project(u, v, head2base)
            if proj is None:
                reject(box, 'rayo sobre el horizonte')
                continue
            x, y, rng = proj
            # Tamano coherente con la distancia: el radio en pixeles tiene que ser el de una pelota a esa distancia
            expected_r = self.K[0] * self.ball_radius / max(rng, 0.1)
            r_px = 0.25 * (bw + bh)
            if not 0.5 <= r_px / expected_r <= 2.0 or not (-0.5 < x < 12.0):
                reject(box, 'tamano vs distancia (r=%.1fpx esperado %.1fpx a %.1fm)' % (r_px, expected_r, rng))
                continue
            score = ring_frac * min(1.0, area / float(bw * bh) / 0.7)
            reject(box, 'OK (%.2f, %.2f) m' % (x, y))
            out.append((x, y, score))
        out.sort(key=lambda c: -c[2])
        return out
