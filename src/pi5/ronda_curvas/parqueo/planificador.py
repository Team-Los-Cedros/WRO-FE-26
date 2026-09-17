"""Busqueda Ackermann en la bahia: marcha adelante/atras y cuerpo completo.

Marco canonico: x a lo largo del carril, y desde el muro hacia el carril.
El hueco ocupa [-largo/2,largo/2] x [0,profundidad]. theta CCW en ese marco.
No hay puertos ni sensores aqui; el seguidor contrasta la ruta con medidas.

Cambios respecto a la version anterior:
- Margen de 8 mm (en vez de 16) sobre la envolvente real; 16 era mas del
  doble de la tolerancia WRO y estrangulaba la bahia.
- Heuristica con campo de distancias 2D desde la meta, rodeando los
  delimitadores; la euclidiana anterior penalizaba toda rotacion y expandia
  nodos a lo largo del carril sin entrar en la bahia.
- Cinco curvaturas (0, +-1/R, +-0.5/R) en vez de tres: suaviza el volante
  y genera transiciones admisibles que tres arcos discretos no alcanzan.
- Meta volumetrica: cuerpo entero dentro de la bahia con margen y angulo
  menor a 6 grados; antes exigia 14 mm a un unico punto exacto.
- Factor de heuristica 1.8 en lugar de 2.5: acepta rutas un 80% mas largas
  que el optimo 2D, pero converge con menos de 20k nodos en < 4 s.
"""
import heapq
import math
import time


def mover(pose, distancia, curvatura):
    x, y, a = pose
    b = a + distancia * curvatura
    if abs(curvatura) < 1e-9:
        return x + distancia * math.cos(a), y + distancia * math.sin(a), b
    return (x + (math.sin(b) - math.sin(a)) / curvatura,
            y + (math.cos(a) - math.cos(b)) / curvatura, b)


class PlanificadorBahia:
    def __init__(self, largo=330, profundidad=200, largo_robot=222,
                 ancho_robot=140, voladizo=60, radio_pos=260, radio_neg=360,
                 margen=8.0):
        self.largo, self.profundidad = largo, profundidad
        self.frente = largo_robot - voladizo
        self.cola, self.semi = voladizo, ancho_robot / 2
        self.radios = (radio_pos, radio_neg)
        self.margen = margen
        self.goal = (-(self.frente - self.cola) / 2, profundidad / 2, 0)
        self.rectangulos = [(-largo/2-20, -largo/2, 0, profundidad),
                           (largo/2, largo/2+20, 0, profundidad)]
        self.nodos = 0
        self._h2d = None
        self._h2d_res = 15.0
        self._h2d_xmin = -500.0
        self._h2d_ymin = 0.0
        self._h2d_nx = 0
        self._h2d_ny = 0

    def _precalcular_h2d(self):
        """Campo de distancias 2D desde la meta, rodeando delimitadores."""
        res = self._h2d_res
        xmin, xmax = self._h2d_xmin, 500.0
        ymin, ymax = self._h2d_ymin, 450.0
        nx = int((xmax - xmin) / res) + 1
        ny = int((ymax - ymin) / res) + 1
        self._h2d_nx, self._h2d_ny = nx, ny
        INF = float('inf')
        grid = [INF] * (nx * ny)
        obst_margin = self.semi + self.margen
        for ix in range(nx):
            for iy in range(ny):
                x = xmin + ix * res
                y = ymin + iy * res
                if y < obst_margin:
                    continue
                blocked = False
                if 0 <= y <= self.profundidad + obst_margin:
                    for x0, x1, _y0, _y1 in self.rectangulos:
                        if x0 - obst_margin <= x <= x1 + obst_margin:
                            blocked = True
                            break
                if blocked:
                    continue
                grid[ix * ny + iy] = 1e9
        gix = int(round((self.goal[0] - xmin) / res))
        giy = int(round((self.goal[1] - ymin) / res))
        if 0 <= gix < nx and 0 <= giy < ny:
            grid[gix * ny + giy] = 0.0
        q = [(0.0, gix, giy)]
        while q:
            d, ix, iy = heapq.heappop(q)
            idx = ix * ny + iy
            if d > grid[idx]:
                continue
            for dx, dy in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                nix, niy = ix + dx, iy + dy
                if 0 <= nix < nx and 0 <= niy < ny:
                    nidx = nix * ny + niy
                    if grid[nidx] > 1e8:
                        step = res * math.hypot(dx, dy)
                        nd = d + step
                        grid[nidx] = nd
                        heapq.heappush(q, (nd, nix, niy))
        self._h2d = grid

    def _heuristica(self, pose):
        if self._h2d is None:
            self._precalcular_h2d()
        res = self._h2d_res
        ix = int(round((pose[0] - self._h2d_xmin) / res))
        iy = int(round((pose[1] - self._h2d_ymin) / res))
        if 0 <= ix < self._h2d_nx and 0 <= iy < self._h2d_ny:
            val = self._h2d[ix * self._h2d_ny + iy]
            if val < 1e8:
                h = val
            else:
                h = math.hypot(pose[0] - self.goal[0], pose[1] - self.goal[1])
        else:
            h = math.hypot(pose[0] - self.goal[0], pose[1] - self.goal[1])
        if pose[1] <= self.profundidad + 20:
            h += 30.0 * abs(pose[2])
        return h

    def libre(self, pose):
        x, y, a = pose
        c, s = math.cos(a), math.sin(a)
        m = self.margen
        puntos = [(x + u*c - v*s, y + u*s + v*c)
                  for u in (-self.cola-m, self.frente+m)
                  for v in (-self.semi-m, self.semi+m)]
        xs, ys = [p[0] for p in puntos], [p[1] for p in puntos]
        if min(ys) <= 0 or max(ys) >= 800 or min(xs) < -800 or max(xs) > 800:
            return False
        # SAT sobre los cuatro ejes: incluye los cantos, no solo esquinas.
        for x0, x1, y0, y1 in self.rectangulos:
            rect = ((x0,y0),(x0,y1),(x1,y0),(x1,y1))
            separado = False
            for ax, ay in ((1,0),(0,1),(c,s),(-s,c)):
                p = [px*ax+py*ay for px,py in puntos]
                q = [px*ax+py*ay for px,py in rect]
                if max(p) < min(q) or max(q) < min(p):
                    separado = True
                    break
            if not separado:
                return False
        return True

    def llegado(self, pose):
        """Meta volumetrica: todo el cuerpo dentro de la bahia, paralelo."""
        if abs(pose[2]) > math.radians(6.0):
            return False
        x, y, a = pose
        c, s = math.cos(a), math.sin(a)
        m = self.margen
        for u in (-self.cola - m, self.frente + m):
            for v in (-self.semi - m, self.semi + m):
                px = x + u * c - v * s
                py = y + u * s + v * c
                if not (0 <= py <= self.profundidad and
                        -self.largo / 2 <= px <= self.largo / 2):
                    return False
        return self.libre(pose)

    def buscar(self, inicio, max_segundos=6.0):
        """Devuelve [(pose, distancia_firmada, curvatura)] o None."""
        trabajo = self.buscar_iter(inicio, max_segundos)
        while True:
            try:
                next(trabajo)
            except StopIteration as fin:
                return fin.value

    def buscar_iter(self, inicio, max_segundos=8.0):
        """Cede cada ~35 ms para que el control siga leyendo sensores parado."""
        if not self.libre(inicio):
            return None
        if self._h2d is None:
            self._precalcular_h2d()
        t0 = time.monotonic()
        consumido = 0.0
        paso = 25.0
        curvas = (0.0,
                  1/self.radios[0], -1/self.radios[1],
                  0.5/self.radios[0], -0.5/self.radios[1])
        def clave(p, direccion):
            return (round(p[0]/10), round(p[1]/8),
                    round(math.degrees(p[2])/3.5), direccion)
        nodos = [(inicio, -1, 0.0, 0.0, 0)]
        cola = [(self._heuristica(inicio), 0.0, 0)]
        costes = {clave(inicio,0): 0.0}
        while cola:
            if time.monotonic()-t0 > .035:
                consumido += time.monotonic()-t0
                yield None
                t0 = time.monotonic()
            if len(nodos) >= 90000 or consumido > max_segundos:
                self.nodos = len(nodos)
                return None
            _, coste, indice = heapq.heappop(cola)
            pose, padre, d0, k0, dir0 = nodos[indice]
            if coste > costes.get(clave(pose,dir0), float('inf')) + 1e-6:
                continue
            if self.llegado(pose):
                ruta=[]
                while indice:
                    p, padre, d, k, _ = nodos[indice]
                    ruta.append((p,d,k)); indice=padre
                self.nodos = len(nodos)
                return list(reversed(ruta))
            for direccion in (-1,1):
                for curva in curvas:
                    distancia = direccion*paso
                    nueva = mover(pose,distancia,curva)
                    if abs(nueva[2]) > math.radians(75):
                        continue
                    if not all(self.libre(mover(pose,distancia*f,curva)) for f in (.33,.67,1)):
                        continue
                    g = coste + paso + (20 if dir0 and direccion != dir0 else 0)
                    g += 2 if curva != k0 else 0
                    key=clave(nueva,direccion)
                    if g >= costes.get(key,float('inf')):
                        continue
                    costes[key]=g
                    nodos.append((nueva,indice,distancia,curva,direccion))
                    heapq.heappush(cola,(g+1.8*self._heuristica(nueva),g,len(nodos)-1))
        self.nodos=len(nodos)
        return None
