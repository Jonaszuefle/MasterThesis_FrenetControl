import Sofa
from Sofa.constants import *
import numpy as np
import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

# Initialize a Node
def init(nodeName = "Sofa"):
    if not rclpy.ok():
       rclpy.init()
       
    node = rclpy.create_node(nodeName)
    node.get_logger().info('Created node')
    return node

# Class for detecting the position of the TCP of the surgical incision tool
class ContactDetectionController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, *kwargs)

        self.rootNode = kwargs.get("rootNode")
        self.mech_obj = kwargs.get("MechanicalObject")
        self.modifier = kwargs.get("modifier")
        self.state = kwargs.get("state")
        
        self.radius = 0.78
      
        ## incision tool
        self.incision_tool = self.rootNode.getChild("CuttingTool")        
        self.incision_tool_mech = self.mech_obj.getObject("cuttingToolMechs")
        self.incision_tool_visual = self.incision_tool.getChild("VisualModel")
        self.incision_tool_visual_model = self.incision_tool_visual.getObject("tool_visual_model")

        print("Init Contact modul is done")
           
    def onAnimateBeginEvent(self, e):

        incision_tool_pos = self.incision_tool_mech.findData('position').value  # Take the tip point from the mechanical drawing (it is only the tip point)       
                                                                                # Option2: incision_tool_visual = self.incision_tool_visual_model.findData('position').value

        if np.linalg.norm(incision_tool_pos[0][0:3]) < self.radius:             # If the tip point is smaller than the radius of the cornea, draw the point  
            with self.state.position.writeable() as state:                      
                i = len(state)-1
                state[i] = np.array(incision_tool_pos[0][0:3])                  # Option2: incision_tool_visual[8076] 

            self.modifier.addPoints(1, True)
        

        