"""Pose por LiDAR/IMU y seguimiento del plan Ackermann de la bahia."""
import math
import numpy as np

import geometria_evasion as gev
import geometria_robot as geo
from .modelos import Consigna
from .planificador import PlanificadorBahia, mover


class SeguidorBahia:
    def __init__(self, retorno, heading, ahora):
        self.retorno = retorno
        self.lado = retorno.lado
        p, h = retorno.paredes, retorno.hueco
        lateral = p.izquierda if self.lado < 0 else p.derecha
        a = math.radians(p.frontal.angulo_deg)
        self.rumbo_ref = heading - p.frontal.angulo_deg
        theta = self.lado * a
        sensor_x = -(self.lado*h.distancia_lateral_mm*math.sin(a)
                     + h.centro_y_mm*math.cos(a))
        self.pose = (sensor_x-geo.LIDAR_X*math.cos(theta)-geo.LIDAR_Y*math.sin(a),
                     lateral.distancia_mm-geo.LIDAR_X*math.sin(theta)
                     + self.lado*geo.LIDAR_Y*math.cos(theta), theta)
        pos = gev.radio_de_comando(gev.COMANDO_MAX_IZQ)
        neg = gev.radio_de_comando(-gev.COMANDO_MAX_DER)
        self.planificador = PlanificadorBahia(radio_pos=pos if self.lado>0 else neg,
                                               radio_neg=neg if self.lado>0 else pos)
        # Rectas de las caras fisicas de los delimitadores y del muro.
        self.segmentos = [(-2000, 0, 2000, 0)]
        for x0,x1,y0,y1 in self.planificador.rectangulos:
            self.segmentos.extend([(x0,y0,x1,y0),(x0,y1,x1,y1),
                                    (x0,y0,x0,y1),(x1,y0,x1,y1)])
        frente = sensor_x + p.frontal.distancia_mm
        self.segmentos.append((frente, 0, frente, 3000))
        self._segmentos = np.asarray(self.segmentos, dtype=float)
        self.estado = "APROXIMAR_BAHIA"
        self.inicio = self.ultimo_t = ahora
        self.vel = self.cmd = 0
        self.ruta = None
        self.indice = 0
        self._sin_pose = 0
        self._verifica = 0
        self._t_verificar = ahora
        self._replanes = 0
        self._busqueda = None
        self.razon = "bahia localizada"

    def _actualizar_pose(self, heading, ahora):
        dt = max(0, min(.3, ahora-self.ultimo_t))
        self.ultimo_t = ahora
        k = self.lado * (1 if self.cmd>0 else -1) / gev.radio_de_comando(self.cmd)
        pred = mover(self.pose, self.vel*4.0*dt, k)
        a = self.lado*math.radians(heading-self.rumbo_ref)
        scan = self.retorno.percepcion._mascarar(self.retorno.scan,0)
        if len(scan)<15:
            return False
        polar = np.asarray(scan,dtype=float)
        rad=np.radians(polar[:,0]); dist=polar[:,1]
        xr=dist*np.sin(rad)+geo.LIDAR_Y
        yr=dist*np.cos(rad)+geo.LIDAR_X
        x=pred[0]+yr*math.cos(a)+self.lado*xr*math.sin(a)
        y=pred[1]+yr*math.sin(a)-self.lado*xr*math.cos(a)
        dx_total=dy_total=0.0
        seg=self._segmentos
        horizontal=np.abs(seg[:,1]-seg[:,3])<1e-6
        for _ in range(2):
            px=x[:,None]; py=y[:,None]
            ex=np.clip(px,np.minimum(seg[:,0],seg[:,2]),np.maximum(seg[:,0],seg[:,2]))
            ey=np.clip(py,np.minimum(seg[:,1],seg[:,3]),np.maximum(seg[:,1],seg[:,3]))
            costes=(ex-px)**2+(ey-py)**2
            ids=np.argmin(costes,axis=1)
            cerca=np.min(costes,axis=1)<60.0**2
            ix=cerca & ~horizontal[ids]
            iy=cerca & horizontal[ids]
            # Las caras de los separadores fijan x con mas precision que
            # el muro lejano usado para aproximarse desde el carril.
            locales = ix & (np.abs(seg[ids,0]) < 300)
            if np.sum(locales) >= 3:
                ix = locales
            if np.sum(ix)<3 or np.sum(iy)<5:
                return False
            dx=float(np.median(seg[ids[ix],0]-x[ix]))
            dy=float(np.median(seg[ids[iy],1]-y[iy]))
            if abs(dx)>40 or abs(dy)>40:
                return False
            x+=dx; y+=dy; dx_total+=dx; dy_total+=dy
        self.pose=(pred[0]+dx_total,pred[1]+dy_total,a)
        return True

    def _salida(self, velocidad, cmd, razon, terminado=False, verificado=False):
        self.vel,self.cmd=velocidad,cmd
        self.razon=razon
        self.retorno.control.estado=self.estado
        self.retorno.control._razon=razon
        return Consigna(velocidad,cmd,self.estado,razon,terminado,verificado)

    def fallar(self, razon):
        self.estado="FALLO"
        return self._salida(0,0,razon,True,False)

    def detener_para_replanear(self):
        self._replanes+=1
        if self._replanes>8:
            return self.fallar("no queda trayectoria con holgura")
        self.estado="PLANIFICAR"
        self._busqueda = None
        return self._salida(0,0,"obstaculo cercano: recalcular parado")

    def procesar(self, heading, us, ahora):
        if ahora-self.inicio>90:
            return self.fallar("tiempo total de parqueo agotado")
        if not self._actualizar_pose(heading,ahora):
            self._sin_pose+=1
            if self._sin_pose>=4:
                return self.fallar("bahia sin localizacion LiDAR fiable")
            return self._salida(0,0,"esperando paredes de la bahia")
        self._sin_pose=0
        x,y,a=self.pose
        if self.estado=="APROXIMAR_BAHIA":
            if x>=280:
                self.estado="PLANIFICAR"
                return self._salida(0,0,"aproximacion medida; planificar parado")
            # Objetivo con adelanto para acercarse suavemente al muro.
            look=max(220,min(450,300-x))
            lateral=300-y
            dx,dy=look,lateral
            cross=-dx*math.sin(a)+dy*math.cos(a)
            k=2*cross/max(dx*dx+dy*dy,1)
            cmd=self._comando(k)
            return self._salida(20,cmd,"aproximando a la cabecera de la bahia")
        if self.estado=="PLANIFICAR":
            # El ciclo anterior ya emitio motor=0. El watchdog de la Pico
            # recibe una parada antes de cualquier busqueda costosa.
            if self._busqueda is None:
                self._busqueda = self.planificador.buscar_iter(self.pose,max_segundos=10)
            try:
                next(self._busqueda)
                return self._salida(0,0,"calculando trayectoria con el robot parado")
            except StopIteration as fin:
                self.ruta = fin.value
                self._busqueda = None
            if not self.ruta:
                return self.fallar("no se encontro plan Ackermann libre")
            self.indice=0
            self.estado="SEGUIR_PLAN"
            return self._salida(0,0,"ruta de %d tramos" % len(self.ruta))
        if self.estado=="VERIFICAR":
            sobra=self.planificador.largo-(self.planificador.frente+self.planificador.cola)
            bien=(self.planificador.llegado(self.pose) and self.planificador.libre(self.pose)
                  and us is not None and us > 34 and 15<=us-34<=sobra-15)
            self._verifica=self._verifica+1 if bien else 0
            if self._verifica>=5:
                self.estado="LISTO"
                return self._salida(0,0,"aparcado por pose y ultrasonido",True,True)
            if self._verifica == 0 and ahora - self._t_verificar > 3.0:
                return self.detener_para_replanear()
            return self._salida(0,0,"verificando pose inmovil")
        if self.estado=="LISTO":
            return self._salida(0,0,"aparcado por pose y ultrasonido",True,True)
        if self.estado=="FALLO":
            return self._salida(0,0,self.razon,True,False)
        if self.planificador.llegado(self.pose):
            self.estado="VERIFICAR"
            self._t_verificar = ahora
            return self._salida(0,0,"pose final alcanzada")
        # Seguir puntos del mismo sentido. Frenar un ciclo en cada cambio.
        target,distancia,curva=self.ruta[self.indice]
        if math.hypot(target[0]-x,target[1]-y)<12:
            previo=distancia
            self.indice+=1
            if self.indice>=len(self.ruta):
                return self.detener_para_replanear()
            target,distancia,curva=self.ruta[self.indice]
            if previo*distancia<0:
                return self._salida(0,0,"cambio de sentido")
        if math.hypot(target[0]-x,target[1]-y)>100:
            return self.detener_para_replanear()
        mirar=self.indice
        while mirar+1<len(self.ruta) and self.ruta[mirar+1][1]*distancia>0:
            if math.hypot(self.ruta[mirar][0][0]-x,self.ruta[mirar][0][1]-y)>=65:
                break
            mirar+=1
        px,py,_=self.ruta[mirar][0]
        dx,dy=px-x,py-y
        cross=-dx*math.sin(a)+dy*math.cos(a)
        k=2*cross/max(25**2,dx*dx+dy*dy)
        cmd=self._comando(k)
        vel=20 if distancia>0 else -20
        if vel<0 and us-34<20:
            return self.detener_para_replanear()
        return self._salida(vel,cmd,"tramo %d/%d" % (self.indice+1,len(self.ruta)))

    def _comando(self,k):
        if abs(k)<1e-7:
            return 0.0
        return gev.comando_de_radio(1/abs(k),hacia_izquierda=k*self.lado>0)
