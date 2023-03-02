#!/usr/bin python3
import roslib; roslib.load_manifest('bebop_driver')
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Int8
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

COMMAND_PERIOD = 20 #ms
class DroneController(object):
    def __init__(self):
        #mantenemos el status del Dron
        self.status = -1
        #mantenemos el controlador de /bebop/takeoff, asentamos y reseteamos topics
        self.pubLand    = rospy.Publisher('/bebop/land',Empty,queue_size=1000)
        self.pubTakeoff = rospy.Publisher('/bebop/takeoff',Empty,queue_size=1000)
        self.pubCommandPilot = rospy.Publisher('/bebop/cmd_vel',Twist,queue_size=1000)
        self.pubCommandCamera = rospy.Publisher('/bebop/camera_control',Twist,queue_size=1000)

        #Setup regualatr publishing of control packets
        self.command = Twist()
        self.command2 = Twist()

        #Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

    def SendTakeoff(self):
        #Send a takeoff message to the bebop driver Note we only send a take off message if... not good!!
          #if(self.status ==DoneStatus.Landed):
          self.pubTakeoff.publish(Empty())

    def SendLand(self):
        #Send a landing message to the driver Note we send this in all states, landing
        self.pubLand.publish(Empty())
   
    def SendEmergency(self):
        #Send and emergency (or reset) message to the bebop driver
        self.pubReset.publish(Empty())

    def SetCommandPilot(self,roll,pitch,yaw_velocity,z_velocity):
        #Called by the main program to set the currrent command
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity
        self.pubCommandPilot.publish(self.command)

    def SenfCommand(self,event):
        #The previous set command is then sent out periodically is the drone is flying
        #if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover
        self.pubCommandPilot.publish(self.command)
        self.pubCommandCamera.publish(self.command)

    def SetCommandCamare(self, tilt=0, cam_pan=0):
        self.command2.angular.y = tilt
        self.command2.angulat.z = cam_pan
        self.pubCommandCamera.publish(self.command2)

    def callback(self,cmd):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo(cmd)
        print (self.Msg.Bat)
        #rospy.loginfo("Linear Components: [%f,%f,%f]"%(cmd.linear.x,cmd.linear.y))..
        #rospy.loginfo("Angular Components: [%f,%f,%f]"%(cmd.angular.x,cmd.angular.y))..
        drone.SetCommadPilot(cmd.linear.y,cmd.linear.x,cmd.angular.z,cmd.linear.z)
        drone.SetCommandCamera(cmd.angular.x, cmd.angular.y)
        #drone.SendCommmand()

        #comm.SendCommand(env)
        #pubLand    = rospy.Publisher('/bebop/land',Empty,queue_size=1000)
        #self.pubTakeoff = rospy.Publisher('/bebop/land',Empty,queue_size=1000)
        #self.pubCommandPilot = rospy.Publisher('/bebop/cmd_vel',Twist,queue_size=1000)
        # #command = Twist()
        # self.command.linear.x   = msg.linear.x
        # self.command.linear.y   = msg.linear.y
        # self.command.linear.z   = msg.linear.z
        # self.command.angular.z  = msg.angular.z
        # pubCommand.publish(command)
       
    def takeoff(self,toff):
        #rospy.Subscriber("/keyboard/takeoff",Int8, takeoff)
        rospy.loginfo(toff.data)
        #if toff.data == True:
        if toff.data == 1:
            drone.SendTakeoff()
            rospy.loginfo("TAKEOFF")

    def land(self,id):
        #rospy.Subscriber("/kayboard/land",int8,land)
        rospy.loginfo(id.data)
        #if id.data == True:
        if id.data ==True:
            drone.SendLand()
            rospy.loginfo("LAND")
            #print "LAND"

    def battery_status(self,data):
        #self.statusBar(),showMessage(data.percent)
        #rospy.loginfo("NIVEL DE BATERIA %s",str(data.percent))
        self.Msg.Bat = "Bateria:"+str(data.percent) + "%"
        print (self.Msg.Bat)

def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/keyoard/cmd_vel",Twist,drone.callback)
    rospy.Subscriber("/keyboard/takeoff",Int8, drone.takeoff)
    rospy.Subscriber("/keyboard/land",Int8,drone.land)
    rospy.Subscriber('bebop/states/common/CommonState/BatteryStateChanged',CommonCommonStateBatteryStateChanged,drone.battery_status)

    #pubReset = rospy.Publisher('/bebop/reset',Empty)
    #Allow the controller to publish to the /cmd_vel topic and thus control the dron SetUp regular publishing of..
    rospy.spin()

if __name__ == '__main__':
    drone = DroneController()
    listener()


