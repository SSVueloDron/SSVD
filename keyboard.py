#!/usr/bin python3
# -*- coding: utf-8 -*-

from ctypes.wintypes import LANGID
from platform import release
import queue
from re import S
from selectors import SelectorKey
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import Empty

#Finally in the GUI libraries
from PySide2 import QtCore, QtWidgets

GUI_UPDATE_PERIOD = 100 #ms

#Aqui definimos el mapeo de teclas para el control
class KeyMapping(object):
    PitchForward        =QtCore.Qt.Key.Key_W
    PitchBackward       =QtCore.Qt.Key.Key_S
    RollLeft            =QtCore.Qt.Key.Key_A
    RollRight           =QtCore.Qt.Key.Key_D
    YawLeft             =QtCore.Qt.Key.Key_Q
    YawRight            =QtCore.Qt.Key.Key_E
    IncreaseAltitude    =QtCore.Qt.Key.Key_Up
    DecreaseAltitude    =QtCore.Qt.Key.Key_Down
    Takeoff             =QtCore.Qt.Key.Key_T
    Land                =QtCore.Qt.Key.Key_Space
    Hovering            =QtCore.Qt.Key.Key_H
    Release             =QtCore.Qt.Key.Key_R

    CamUp               =QtCore.Qt.Key.Key_I
    CamDown             =QtCore.Qt.Key.Key_K
    CamRight            =QtCore.Qt.Key.Key_J
    CamLeft             =QtCore.Qt.Key.Key_L

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow,self).__init__()
        self.setWindowTitle('Bebop interfaz')
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.update)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.tilt = 0
        self.cam_pan = 0
        self.z_velocity = 0
        self.arm = False
        self.toff = False
        self.ld = False
        self.override = False

        self.pubCommandP = rospy.Publisher('/keyboard/cmd_vel',Twist,queue_size=10)
        self.pubLand = rospy.Publisher('/keyboard/land',Int8,queue_size = 10)
        self.pubTakeoff = rospy.Publisher('/keyboard/takeoff',Int8,queue_size = 10)
        self.pubOverride = rospy.Publisher('/keyboard/override',Int8,queue_size = 10)
        self.keyb = Twist()


    #Agregamos un controlador de teclado a DroneVideoDisplay para reaccionar a las pulsaciones de teclas

    def keyPressEvent(self, event):
        self.override = 1
        self.pubOverride.publish(self.override)
        key = event.key()
        #si tenemos un constructor, el controlador del dron y la key no se generan a partir de repetición automática
        #si no hay key, event.isAutoRepeat():

        if not event.isAutoRepeat():

            if key == KeyMapping.Hovering:
                self.keyb.linear.x = 0.0
                self.keyb.linear.y = 0.0
                self.keyb.linear.z = 0.0
                self.keyb.angular.z = 0.0
                self.pubCommandP.publish(self.keyb)
                rospy.loginfo("Hovering")

            elif key== KeyMapping.Takeoff:
                self.toff = not self.toff
                if self.toff == True:
                    #self.pubTakeoff.publish(Empty())
                    self.pubTakeoff.publish(1)
                    self.toff = False
                rospy.loginfo("LANDING")

            else:
                #Ahora manejamos el movimiento, observe que la sección es lo opuesto (=+) de la sección keyrelease
                if key == KeyMapping.YawLeft:
                    self.yaw_velocity += 0.3
                    rospy.loginfo("YAW_LEFT")

                elif key == KeyMapping.YawRight:
                    self.yaw_velocity += -0.3
                    rospy.loginfo("YAW_RIGHT")

                elif key == KeyMapping.PitchForward:
                    self.pitch += 0.3
                    rospy.loginfo("FORWARD")

                elif key == KeyMapping.PitchBackward:
                    self.pitch += -0.3
                    rospy.loginfo("BACKWARD")

                elif key == KeyMapping.RollLeft:
                    self.roll += 0.3
                    rospy.loginfo("LEFT")

                elif key == KeyMapping.RollRight:
                    self.roll += -0.3
                    rospy.loginfo("RIGHT")

                elif key == KeyMapping.IncreaseAltitude:
                    self.z_velocity += 0.3
                    rospy.loginfo("UP")

                elif key == KeyMapping.IncreaseAltitude:
                    self.z_velocity += -0.3
                    rospy.loginfo("DOWN")

                elif key == KeyMapping.CamUp:
                    self.tilt += 5
                elif key == KeyMapping.CamDown:
                    self.tilt += -5
                elif key == KeyMapping.CamLeft:
                    self.cam_pan += 5
                elif key == KeyMapping.CamRight:
                    self.cam_pan += -5

            self.keyb.linear.x = self.pitch
            self.keyb.linear.y = self.roll
            self.keyb.linear.z = self.z_velocity
            self.keyb.angular.x = self.tilt
            self.keyb.angular.y = self.cam_pan
            self.keyb.angular.z = self.yaw_velocity
            self.pubCommandP.publish(self.keyb)
            rospy.loginfo(self.roll)
            rospy.loginfo(self.pitch)
            rospy.loginfo(self.z_velocity)
            rospy.loginfo(self.yaw_velocity)
        else:
            self.pubCommandP.publish(self.keyb)
   
    def KeyReleaseEvent(self,event):
        key = event.key()
        #tenemos que construir el control del dron
        #si no hay key, repetimos, event.isAutoRepeat():
        if not event.isAutoRepeat():
            if key == KeyMapping.YawLeft:
                self.yaw_velocity -= 0.3

            elif key == KeyMapping.YawRight:
                self.yaw_velocity -= -0.3

            elif key == KeyMapping.PitchForward:
                self.pitch -= 0.3

            elif key == KeyMapping.PitchBackward:
                self.pitch -= -0.3
           
            elif key == KeyMapping.RollLeft:
                self.roll -= 0.3

            elif key == KeyMapping.RollRight:
                self.roll -= -0.3

            elif key == KeyMapping.IncreaseAltitude:
                self.z_velocity -= 0.3
           
            elif key == KeyMapping.DecreaseAltitude:
                self.z_velocity -= -0.3

            elif key == KeyMapping.Release:
                self.override = 0
                self.pubCommandP.publish(self.override)

            self.keyb.linear.x  = self.pitch
            self.keyb.linear.y  = self.roll
            self.keyb.linear.z  = self.z_velocity
            self.keyb.angular.x = self.tilt
            self.keyb.angular.y = self.cam_pan
            self.keyb.angular.z = self.yaw_velocity
            self.pubCommandP.publish(self.keyb)

if __name__=='__main__':
    import sys
    rospy.init_node('keyboard')
    app = QtWidgets.QApplication(sys.argv)
    #Primer iniciamos el nodo de Ros
    #rospy.init_node('bebopKeyController')
    #Ahora  construimos nuestra Qt Application y asociamos
    #t1=SoloCamera()
    #t1.start()
    #controller = BasicBebopController()
    #rate = rospy.Rate(10)

    display = MainWindow()
    display.show()
    status = app.exec_()

    #el proceso se hace una vez hasta que la aplicacion...
    rospy.signal_shutdown('Great Flying')
    sys.exit(status)
   
