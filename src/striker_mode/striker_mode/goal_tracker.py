"""Seguidor del arco rival.

Guarda el centro del arco rival en el marco de odometria del robot, asi lo sigue "sabiendo" aunque lo tenga
de espaldas o lejos. La odometria se usa solo como memoria de corto plazo, no para ubicarse en la cancha.

Lo que se midio en el sim (postes de YOLO contra la verdad):
  - la DIRECCION a un poste es buena a cualquier distancia (~4 grados);
  - la DISTANCIA solo sirve de cerca (3-7 m: 4-15 %); de lejos (12-20 m) el error es ~50 %.
Por eso los postes cercanos corrigen la posicion completa y los lejanos solo la direccion.

Arco rival vs propio: mientras no hay arco registrado, solo acepto postes hacia el lado de ataque (IMU).
Despues, solo los que caen cerca del arco registrado (el propio esta 14 m mas alla).
"""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# Caja que ocupa el arco (postes + red) con margen para el cuerpo del robot o la pelota, en el marco de la cancha
# relativo al centro del arco (x hacia el fondo de la red, y a lo ancho). El arco abre hacia -x.
GOAL_BOX = (-0.5, 0.65, -1.75, 1.75)  # xmin, xmax, ymin, ymax


def in_goal_box(p, box=GOAL_BOX):
    return box[0] < p[0] < box[1] and box[2] < p[1] < box[3]


def segment_hits_box(a, b, box=GOAL_BOX):
    """El segmento a->b atraviesa la caja? (Liang-Barsky)"""
    t0, t1 = 0.0, 1.0
    dx, dy = b[0] - a[0], b[1] - a[1]
    for p, q in ((-dx, a[0] - box[0]), (dx, box[1] - a[0]), (-dy, a[1] - box[2]), (dy, box[3] - a[1])):
        if abs(p) < 1e-9:
            if q < 0:
                return False
        else:
            r = q / p
            if p < 0:
                t0 = max(t0, r)
            else:
                t1 = min(t1, r)
            if t0 > t1:
                return False
    return True


def route_around_goal(frm, to, box=GOAL_BOX, margin=0.45):
    """Siguiente punto para ir de frm a to sin atravesar el arco (todo relativo al centro del arco).
    Si el camino directo esta libre devuelve to; si no, la esquina de afuera del arco por donde rodearlo."""
    if in_goal_box(frm, box):
        return (box[0] - margin, max(box[2] + 0.4, min(box[3] - 0.4, frm[1])))  # adentro del arco: salgo por el frente
    if not segment_hits_box(frm, to, box):
        return to
    side = 1.0 if (frm[1] + to[1]) >= 0 else -1.0
    corners = [(box[0] - margin, side * (box[3] + margin)), (box[1] + margin, side * (box[3] + margin))]
    # la esquina en la que ya estoy no cuenta (si no, me quedo parado en ella)
    corners = [c for c in corners if math.hypot(c[0] - frm[0], c[1] - frm[1]) > 0.3] or corners
    # Preferencia: una esquina a la que llego directo y desde la que ya llego directo al destino; si no hay,
    # una a la que llego directo y que me deja mas cerca del destino (asi siempre avanzo, sin ir y volver)
    free = [c for c in corners if not segment_hits_box(frm, c, box)]
    last_hop = [c for c in free if not segment_hits_box(c, to, box)]
    if last_hop:
        return min(last_hop, key=lambda c: math.hypot(c[0] - frm[0], c[1] - frm[1]) + math.hypot(c[0] - to[0], c[1] - to[1]))
    return min(free or corners, key=lambda c: math.hypot(c[0] - to[0], c[1] - to[1]))


class GoalTracker:
    def __init__(self, half_width=1.3, near_range=7.0, init_cone=1.0, gate=2.5, memory=10.0):
        self.memory = memory  # s que recuerdo cada poste visto de cerca
        self.half_width = half_width
        self.near_range = near_range  # postes mas cerca que esto: confio en la distancia
        self.init_cone = init_cone  # rad alrededor del rumbo de ataque para registrar el arco por primera vez
        self.gate = gate  # m: postes a menos de esto (o proporcional a la distancia) son de este arco
        self.reset()

    def reset(self):
        self.c = None  # centro del arco en odometria (x, y)
        self.t = -1e9  # ultima vez que lo vi
        self.t_near = -1e9  # ultima vez que lo vi de cerca (distancia confiable)
        self.t_pair = -1e9  # ultima vez que vi postes de los dos lados
        self.near_posts = []  # postes vistos de cerca (x, y, t) en odometria, de los ultimos segundos
        self.far_b = None  # antes de ubicarlo de cerca: direccion al arco (angulo en odometria)

    def update(self, posts, odom, field_dir, t):
        """posts: [(x, y, conf, distancia_confiable)] en el marco del robot. odom: (x, y, theta).
        field_dir: angulo del eje x de la cancha (hacia el arco rival) en el marco de odometria."""
        if not posts:
            return
        ox, oy, oth = odom
        c, s = math.cos(oth), math.sin(oth)
        fy = (-math.sin(field_dir), math.cos(field_dir))  # eje y de la cancha (a lo ancho del arco)
        meas = []
        for px, py, _, reliable in posts:
            r = math.hypot(px, py)
            wx, wy = ox + c * px - s * py, oy + s * px + c * py
            meas.append((wx, wy, r, math.atan2(wy - oy, wx - ox), reliable))

        if self.c is None:
            # Todavia no lo tengo ubicado: solo acepto postes hacia el lado de ataque (asi no tomo el arco propio)
            fwd = [m for m in meas if abs(wrap(m[3] - field_dir)) < self.init_cone]
            if not fwd:
                return
            # De lejos solo guardo la DIRECCION (la distancia lejana sale hasta ~50 % mal)
            b = math.atan2(sum(math.sin(m[3]) for m in fwd), sum(math.cos(m[3]) for m in fwd))
            self.far_b = b if self.far_b is None else wrap(self.far_b + 0.5 * wrap(b - self.far_b))
            self.t = t
            # La POSICION la registro recien de cerca, con un par de postes confiables separados ~el ancho del arco
            near = [m for m in fwd if m[4]]
            fx = (math.cos(field_dir), math.sin(field_dir))
            for i in range(len(near)):
                for j in range(i + 1, len(near)):
                    a, q = near[i], near[j]
                    dl = abs((a[0] - q[0]) * fy[0] + (a[1] - q[1]) * fy[1])  # separacion a lo ancho del arco
                    dd = abs((a[0] - q[0]) * fx[0] + (a[1] - q[1]) * fx[1])  # y en profundidad
                    if abs(dl - 2 * self.half_width) < 0.5 and dd < 0.9:
                        self.c = ((a[0] + q[0]) / 2, (a[1] + q[1]) / 2)
                        self.t_near = self.t_pair = t
                        return
            return

        cx, cy = self.c
        crng = math.hypot(cx - ox, cy - oy)
        cb = math.atan2(cy - oy, cx - ox)
        mine = []
        for m in meas:
            close = math.hypot(m[0] - cx, m[1] - cy) < max(self.gate, 0.4 * m[2])
            same_dir = abs(wrap(m[3] - cb)) < math.atan2(self.half_width + 1.0, max(crng, 0.5)) + 0.1
            if close or (same_dir and not m[4]):
                mine.append(m)
        if not mine:
            return
        self.t = t
        near = [m for m in mine if m[4]]  # distancia confirmada (proyeccion y tamano del poste coinciden)
        self.last_used = (mine, near)  # para diagnostico
        if near:
            # Distancia confiable. Junto los postes vistos de cerca en los ultimos segundos (en odometria) y los
            # agrupo: con postes de los DOS lados, el centro lateral es el punto medio; con un solo lado no se si
            # es el poste izquierdo o el derecho, asi que solo corrijo la profundidad (no el lateral).
            self.near_posts = [p for p in self.near_posts if t - p[2] < self.memory] + [(m[0], m[1], t) for m in near]
            fx = (math.cos(field_dir), math.sin(field_dir))
            clusters = []
            for px, py, _ in self.near_posts:
                for cl in clusters:
                    if math.hypot(px - cl[0] / cl[2], py - cl[1] / cl[2]) < 0.7:
                        cl[0] += px; cl[1] += py; cl[2] += 1
                        break
                else:
                    clusters.append([px, py, 1])
            pts = [(cl[0] / cl[2], cl[1] / cl[2]) for cl in clusters if cl[2] >= 2] or [(m[0], m[1]) for m in near]
            lat = [p[0] * fy[0] + p[1] * fy[1] for p in pts]
            dep = [p[0] * fx[0] + p[1] * fx[1] for p in pts]
            c_lat = cx * fy[0] + cy * fy[1]
            c_dep = cx * fx[0] + cy * fx[1]
            # Uso la forma del arco: el centro lateral sale de un PAR de postes separados ~el ancho del arco y a
            # casi la misma profundidad. Asi una deteccion falsa (p. ej. algo detras de la red) no lo corre.
            best = None
            for i in range(len(pts)):
                for j in range(i + 1, len(pts)):
                    dl, dd = abs(lat[i] - lat[j]), abs(dep[i] - dep[j])
                    miss = abs(dl - 2 * self.half_width)
                    if miss < 0.5 and dd < 0.9 and (best is None or miss < best[0]):
                        best = (miss, (lat[i] + lat[j]) / 2, (dep[i] + dep[j]) / 2)
            if best is not None:
                new_lat = c_lat + 0.5 * (best[1] - c_lat)
                new_dep = c_dep + 0.4 * (best[2] - c_dep)
                self.t_pair = t
            else:
                # Un solo lado: no se si es el poste izquierdo o el derecho -> solo la profundidad, y solo con
                # postes que caen dentro del ancho del arco (descarto los que estan muy al costado)
                inside = [d for d, l in zip(dep, lat) if abs(l - c_lat) < self.half_width + 0.5]
                new_lat = c_lat
                new_dep = c_dep + 0.4 * (sum(inside) / len(inside) - c_dep) if inside else c_dep
            self.c = (new_dep * fx[0] + new_lat * fy[0], new_dep * fx[1] + new_lat * fy[1])
            self.t_near = t
        else:
            # De lejos la distancia no sirve (y sale sesgada): solo corrijo la direccion
            b = math.atan2(sum(math.sin(m[3]) for m in mine), sum(math.cos(m[3]) for m in mine))
            nb = cb + 0.3 * wrap(b - cb)
            self.c = (ox + crng * math.cos(nb), oy + crng * math.sin(nb))

    def relative(self, odom):
        """(direccion en el marco del robot, distancia o None si aun no lo ubique de cerca), o None."""
        ox, oy, oth = odom
        if self.c is None:
            return None if self.far_b is None else (wrap(self.far_b - oth), None)
        dx, dy = self.c[0] - ox, self.c[1] - oy
        return wrap(math.atan2(dy, dx) - oth), math.hypot(dx, dy)
