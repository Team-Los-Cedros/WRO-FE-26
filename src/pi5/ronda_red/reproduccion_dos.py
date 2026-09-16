"""Registro y reproduccion determinista del observador, sin dispositivos."""
import atexit
from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace

from dos_pilares import Caja, Cuadro, SiguientePilar


class RegistroReproduccion:
    def __init__(self, ruta):
        self.f = open(ruta, 'x', encoding='utf-8')
        self.n = 0
        self.activo = True
        atexit.register(self.f.close)

    def guardar(self, cuadro, clusters, actual, giro, avance, ahora, en_seccion,
                estado, barrido=None):
        if not self.activo:
            return
        try:
            dato = {
                'cuadro': asdict(cuadro) if cuadro is not None else None,
                'clusters': clusters, 'barrido': barrido,
                'actual': {k: getattr(actual, k) for k in
                           ('id', 'activo', 'color', 'x', 'y', 'sigma', 'ambiguo')},
                'giro': giro, 'avance': avance, 'ahora': ahora, 'estado': estado,
                'secciones': [bool(en_seccion(x, y)) for x, y, _ in clusters],
            }
            self.f.write(json.dumps(dato, separators=(',', ':')) + '\n')
            self.n += 1
            if self.n % 10 == 0:
                self.f.flush()
        except Exception as e:
            self.activo = False
            print('[dos] registro de reproduccion detenido: %s' % e)


def reproducir(ruta):
    observador = SiguientePilar()
    with open(ruta, encoding='utf-8') as f:
        for linea in f:
            d = json.loads(linea)
            c = d['cuadro']
            cuadro = None if c is None else Cuadro(
                c['secuencia'], c['captura_mono'], c['captura_unix'],
                tuple(Caja(**caja) for caja in c['cajas']))
            secciones = {(x, y): s for (x, y, _), s in zip(d['clusters'], d['secciones'])}
            observador.actualizar(cuadro, d['clusters'], SimpleNamespace(**d['actual']),
                d['giro'], d['avance'], d['ahora'], lambda x, y: secciones.get((x, y), False))
            yield dict(t=d['ahora'], estado=d['estado'], color=observador.color,
                       x=observador.x, y=observador.y, sigma=observador.sigma,
                       fuente=observador.fuente, motivo=observador.motivo,
                       edad=None if observador.t is None else d['ahora'] - observador.t)


if __name__ == '__main__':
    import argparse
    from collections import Counter
    p = argparse.ArgumentParser()
    p.add_argument('registro')
    args = p.parse_args()
    filas = list(reproducir(args.registro))
    destino = Path(args.registro).with_suffix('.reproducido.json')
    destino.write_text(json.dumps(filas, indent=2), encoding='utf-8')
    print('Ciclos:', len(filas))
    print('Fuentes:', Counter(f['fuente'] for f in filas))
    print('Salida:', destino)
