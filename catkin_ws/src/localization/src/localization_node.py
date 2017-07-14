#!/usr/bin/env python
import rospy
#from apriltags_ros.msg import AprilTagDetectionArray
from duckietown_msgs.msg import AprilTagsWithInfos
import tf2_ros
from tf2_msgs.msg import TFMessage
import tf.transformations as tr
from geometry_msgs.msg import Transform, TransformStamped
import numpy as np
from localization import PoseAverage
from visualization_msgs.msg import Marker
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
import time
import math

# Localization Node
# Author: Teddy Ort
# Inputs: apriltags/duckietown_msgs/AprilTags - A list of april tags in a camera frame
# Outputs: pose2d/duckietown_msgs/Pose2dStamped - The estimated pose of the robot in the world frame in 2D coordinates
#          pose3d/geometry_msgs/PoseStamped - The estimated pose of the robot in the world frame in 3D coordinates

class LocalizationNode(object):
	
	def __init__(self):
        self.node_name = 'localization_node'

        # Constants
        self.world_frame = "world"
        self.duckiebot_frame = "duckiebot"
        self.duckiebot_lifetime = self.setupParam("~duckiebot_lifetime", 5) # The number of seconds to keep the duckiebot alive bewtween detections
        self.highlight_lifetime = self.setupParam("~highlight_lifetime", 3) # The number of seconds to keep a sign highlighted after a detection
		self.lista=list()
		self.Tn=None

        # Setup the publishers and subscribers
        self.sub_april = rospy.Subscriber("~apriltags", AprilTagsWithInfos, self.tag_callback)
		self.sub_wheels=rospy.Subscriber('/duckiebot/wheels_driver_node/car_cmd', Twist2DStamped, self.pos_callback)
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1, latch=True)
        self.pub_rviz = rospy.Publisher("/sign_highlights", Marker, queue_size=1, latch=True)

        # Setup the transform listener
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        # Use a timer to make the duckiebot disappear
        self.lifetimer = rospy.Time.now()
        self.publish_duckie_marker()

        rospy.loginfo("[%s] has started", self.node_name)
	
	#delta: msg(Twist2DStamped) -> None
	#Guarda una lista con la velocidad lineal, angular y el tiempo actual en el objeto self.lista
	def delta(self,msg):
		i=len(self.lista)
		if i>50: #Si el objeto self.lista tiene mas de 50 elementos, lo reducimos al ultimo elemento (los anteriores no sirven)
			L=list()
			L.append(self.lista[i-1])
			self.lista=L[:]
		#Guardamos las velocidades y tiempo actual en el objeto self.lista
    	self.lista.append([msg.v,msg.omega,rospy.get_time()])


	#pos_callback: msg(Twist2DStamped) -> None
	#Calcula el cambio de posicion que hubo entre la posición en la que estaba el robot entre el comando (del joystick) anterior y actual
	#luego, publica esa posicion el mapa.
    def pos_callback(self,msg):
		#Guardamos las velocidades y tiempo actual en el objeto self.lista
		self.delta(msg)
		#si es el primer comando que se recibe del joystick, no hay comando anterior por lo que se termina la función
        if len(self.lista)==1:
			return
		#si no es el primer comando que se recibe del joystick y ya se tiene una posición inicial
    	elif len(self.lista)>1 and self.Tn!=None:
    		i=len(self.lista)
    		deltaT=self.lista[i-1][2]-self.lista[i-2][2]
    		Omega=self.lista[i-2][1]
    		v=self.lista[i-2][0]
			#se crea una lista con la desplazamiento lineal y angular, notese que 0.56 es el k del que se habla en el reporte
			#y 0.43 es el k'. (Factores que relacionan el desplazamiento lineal y angular, teórico con el real, respectivamente)
    		l=[float(deltaT)*float(v)*float(0.56),float(deltaT)*float(Omega)*float(0.43)]			
			a=math.cos(l[1])
			b=math.sin(l[1])
			#Se convierte la información de posición anterior a una matriz
			M=self.transform_to_matrix(self.Tn)
			#Se define una matriz que tiene la información del cambio de posición y orientación 
			Mov=np.matrix([[a,-b,0,l[0]],[b,a,0,0],[0, 0,1,0],[0,0,0,1]])
			#Se multiplica la matriz que tiene el desplazamiento a la de la última posición guardada
			Mr_w=np.dot(M,Mov)
			#Se guarda la matriz anterior en un formato útil
			P=self.matrix_to_transform(Mr_w)
			#Se guarda la nueva posición y orientación
			self.Tn=P
			T= TransformStamped()
			T.transform = P
			T.header.frame_id = self.world_frame
			T.header.stamp = rospy.Time.now()
			T.child_frame_id = self.duckiebot_frame
			#Se publica la posición, orientación y otros detalles en el mapa
			self.pub_tf.publish(TFMessage([T]))
			self.lifetimer = rospy.Time.now()



	#tag_callback: msg_tag -> None
	#Estima la posicion cuando ve un Apriltag.
    def tag_callback(self, msg_tag):
        # Listen for the transform of the tag in the world
        avg = PoseAverage.PoseAverage()
        for tag in msg_tag.detections:
            try:
                Tt_w = self.tfbuf.lookup_transform(self.world_frame, "tag_{id}".format(id=tag.id), rospy.Time(), rospy.Duration(1))
                Mtbase_w=self.transform_to_matrix(Tt_w.transform)
                Mt_tbase = tr.concatenate_matrices(tr.translation_matrix((0,0,0.17)), tr.euler_matrix(0,0,np.pi))
                Mt_w = tr.concatenate_matrices(Mtbase_w,Mt_tbase)
                Mt_r=self.pose_to_matrix(tag.pose)
                Mr_t=np.linalg.inv(Mt_r)
                Mr_w=np.dot(Mt_w,Mr_t)
                Tr_w = self.matrix_to_transform(Mr_w)
                avg.add_pose(Tr_w)
                self.publish_sign_highlight(tag.id)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                rospy.logwarn("Error looking up transform for tag_%s", tag.id)
                rospy.logwarn(ex.message)
        
        Tr_w =  avg.get_average() # Average of the opinions

        # Broadcast the robot transform
        if Tr_w is not None:
            # Set the z translation, and x and y rotations to 0
            Tr_w.translation.z = 0
            rot = Tr_w.rotation
            rotz=tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[2]
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_euler(0, 0, rotz)
            T = TransformStamped()
            #Guardamos la Transformada
	    	self.Tn=Tr_w
            T.transform = Tr_w
            T.header.frame_id = self.world_frame
            T.header.stamp = rospy.Time.now()
            T.child_frame_id = self.duckiebot_frame
            self.pub_tf.publish(TFMessage([T]))
            self.lifetimer = rospy.Time.now()

	def publish_duckie_marker(self):
        # Publish a duckiebot transform far away unless the timer was reset
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if rospy.Time.now() - self.lifetimer > rospy.Duration(self.duckiebot_lifetime):
                T = TransformStamped()
                T.transform.translation.z = 1000    # Throw it 1km in the air
                T.transform.rotation.w = 1
                T.header.frame_id = self.world_frame
                T.header.stamp = rospy.Time.now()
                T.child_frame_id = self.duckiebot_frame
                self.pub_tf.publish(TFMessage([T]))

    def publish_sign_highlight(self, id):
        # Publish a highlight marker on the sign that is seen by the robot
        m = Marker()
        m.header.frame_id="tag_{id}".format(id=id)
        m.header.stamp = rospy.Time.now()
        m.id=id
        m.lifetime = rospy.Duration(self.highlight_lifetime)
        m.type = Marker.CYLINDER
        p = m.pose.position
        o = m.pose.orientation
        c = m.color
        s = m.scale
        s.x, s.y, s.z = (0.1, 0.1, 0.3)
        p.z = 0.15
        c.a, c.r, c.g, c.b = (0.2, 0.9, 0.9, 0.0)
        o.w = 1
        self.pub_rviz.publish(m)

    def pose_to_matrix(self, p):
        # Return the 4x4 homogeneous matrix for a PoseStamped.msg p from the geometry_msgs
        trans = (p.pose.position.x, p.pose.position.y, p.pose.position.z)
        rot = (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
        return np.dot(tr.translation_matrix(trans), tr.quaternion_matrix(rot))

    def transform_to_matrix(self, T):
        # Return the 4x4 homogeneous matrix for a TransformStamped.msg T from the geometry_msgs
        trans = (T.translation.x, T.translation.y, T.translation.z)
        rot = (T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w)
        return np.dot(tr.translation_matrix(trans), tr.quaternion_matrix(rot))

    def matrix_to_transform(self, M):
        # Return a TransformStamped.msg T from the geometry_msgs from a 4x4 homogeneous matrix
        T=Transform()
        (T.translation.x, T.translation.y, T.translation.z) = tr.translation_from_matrix(M)
        (T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w) = tr.quaternion_from_matrix(M)
        return T

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('localization_node', anonymous=False)
    localization_node = LocalizationNode()
    rospy.spin()
