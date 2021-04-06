#!/usr/bin/env python2.7

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from std_msgs.msg import Float32

class wallFollower:

    #Inicialitzacio de la classe
    def __init__(self):

        rospy.init_node("wall_follower_P_controller", anonymous=False) #Inicialitzem el node amb un nom
        
        #Inicialitzacio de variables 
        #prototip: self._nomVar = rospy.get_param("~nomVar_des_del_launchFile", valorDefecte)

        #Variables de PID 
        self._kp = rospy.get_param("~kp",1) #constant de P
        self._ki = rospy.get_param("~ki",0) #constant de I (desactivada ja que nomes fem control P)
        self._kd = rospy.get_param("~kd",0) #constant de D (desactivada ja que nomes fem control P)

        self._pErr = 0 #Error de P. No demanem el parametre des del launch. Li donem directament un valor inicial

        #Variables de distancia (per implementar les equacions del control P)
        self._initialDistance = rospy.get_param("~initialDistance",0) #distancia inicial del robot

        self._projectedLongitude = rospy.get_param("~projectedLongitude",0.1) #Longitud de la projeccio
        self._theta = rospy.get_param("~theta",20.0) #Angle theta del laser
        self._forwardSpeed = rospy.get_param("forwardSpeed",0.1) #Velocitat del robot en moviment endevant
        self._error_z = 0 #Error del control P en l'eix z
        self._distRef = 0.01 #Distancia de referencia a on volem que s'alinii el robot
        #Definicio de topics i modes de rospy
        ##Definicio del missatge
        self._msg = Twist() #Definim msg com a un objecte de tipus Twist
        self._msg.linear.x = 0.1 #Inicialitzem el seu x = 0 (per a posar-lo a l'origen en l'eix x)
        self._msg.linear.z = 0 #Inicialitzem el seu z = 0 (per a posar-lo a l'origen en l'eix z)
        #self._msg.angular.z = 0.01 
        ##Definicio de la comanda de velocitat
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #Definim cmdVel com un publicador amb el mateix nom de tipus Twist i mida de cua = 10
        
        ##Definicio de la lectura del laser
        rospy.Subscriber("/scan", LaserScan, self.callbackLaser) #Definim un subscriptor que es diu scan de tipus LaserScan que crida la funcio callbackLaser
        
        ##Definicio de la variable d'apagat
        rospy.on_shutdown(self.shutdown)

        #Definicio de la frequencia del loop
        self._r = rospy.Rate(5)
    
    #Definicio de Funcions/metodes de la classe
    ##Funcio per enviar el msg a cmdVel
    def start(self): 
        while not rospy.is_shutdown(): #mentres no tinguem apagat el rospy
            #self._cmdVel.publish(self._msg) #publiquem a cmdVel el msg guardat a msg 
            self._r.sleep() #posem a dormir el rospy
    
    ##Funcio que fa el processat de dades quan es reb nova info del Laser
    def callbackLaser(self, scan): 
        #Recorrem la matriu de dades scan.ranges tant en files (val) com en columnes (idx). Si el valor al qual estem
        #hi esta dins dels rangs minims/maxims, li fem el minim i el guardem a closestDistance. Tambe guardem l'index.
        #En cas contrari mantenim el valor anterior a closestDistance i elementIndex
        #Amb aixo aconseguim tenir sempre la minima distancia del laser
        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max
        )
        
        correctionFactor = int(len(scan.ranges)/360) #Factor de correccio per quan connectem el lidar de veritat es recalculin els valors directament tot i tenir 720 graus enlloc de 360
        
        angleClosestDistance = self.__wrapAngle(elementIndex/correctionFactor) # only 360 points in simulated lidar!!!
        #angleClosestDistance = self.__wrapAngle(elementIndex / 2) # Real YDLidar with 720 points!!! S'hauria d'activar si no tinguessim el correctionFactor

        rospy.loginfo("Closest distance of %5.2f m at %5.1f degrees.",
                      closestDistance, angleClosestDistance) #printem la distancia mes propera i el seu angle per consola
        rospy.loginfo("correctionFactor: %i" ,correctionFactor)
        
        
        #Desenvolupament matematic-trigonometric del calcul de la velocitat angular i l'error (per al control P)
        a = scan.ranges[int(self.__unwrapAngle(-90+self._theta) *correctionFactor)] #a sera la longitud en els graus calculats a -90 + la theta 
        b = scan.ranges[int(self.__unwrapAngle(-90)*correctionFactor)] #i b la longitud a -90 graus
        rospy.loginfo("pene: %5.2f", a)
        
        if (-10<angleClosestDistance<10):
            # self._msg.angular.z = 0.01
            self._msg.linear.x = 0.1
            rospy.loginfo('giro ')
        else:
            theta = np.deg2rad(self._theta) #passem theta a radians per poder fer les operacions trigonometriques mitjancant numpy correctament
            #rospy.loginfo("theta: %5.2f", theta)
            alpha = np.arctan((a * np.cos(theta)-b) / (a * np.sin(theta))) #alpha val aixo
            self._initialDistance = b * np.sin(alpha)

            potentialDistance = self._initialDistance + self._projectedLongitude * np.sin(alpha)

            self._error_z = self._distRef - potentialDistance

            self._msg.angular.z = self._kp * self._error_z

            if(abs(self._msg.angular.z)>0.03):
                self._msg.linear.x = 0.1*self._forwardSpeed
            else:
                self._msg.linear.x = self._forwardSpeed
        
            rospy.loginfo("initialDist: %5.2f m, potentialDistance: %5.2f m, alpha: %5.2f rad, angular: %5.2f, error: %5.2f", self._initialDistance,potentialDistance,alpha,self._msg.angular.z,self._error_z)

        #self._msg.linear.x = 0.1
        self._cmdVel.publish(self._msg)
    ##Conversio d'angles
    ###Marca el signe del valor
    def __sign(self, val):
        if val >= 0:
            return 1
        else:
            return -1
    ###Passa d'angles absoluts a angles positius/negatius
    def __wrapAngle(self, angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360
    ###Passa d'angles positius/negatius a angles absoluts
    def __unwrapAngle(self,angle):
        if angle < 0:
            return angle + 360
        else:
            return angle

    ##Apagat del sistema
    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)



if __name__ == '__main__':
    try:
        robot = wallFollower()
        robot.start()
        rospy.spin()

    except rospy.ROSInterruptException: pass
