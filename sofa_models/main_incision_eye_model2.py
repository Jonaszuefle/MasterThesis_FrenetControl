import Sofa
from contact_detection_image_pub import ContactDetectionController
import os

# Choose in your script to activate or not the GUI
USE_GUI = True
# Chose which visual model should be used
OLD_EYE_MODEL = False
NEW_EYE_MODEL = False

def main():
    # Import Sofa library to control the runtime
    import SofaRuntime
    # Import Sofa library to control the graphical interface
    import Sofa.Gui
    # Make sure to load all SOFA libraries
    SofaRuntime.importPlugin("Sofa.Component.StateContainer")
    
    #SofaRuntime.importPlugin("SofaOpenglVisual") --> will be removed with version 23.06
    SofaRuntime.importPlugin("Sofa.GL.Component.Rendering2D")
    SofaRuntime.importPlugin("Sofa.GL.Component.Rendering3D")
    SofaRuntime.importPlugin("Sofa.GL.Component.Shader")

    #SofaRuntime.importPlugin("SofaBoundaryCondition")--> will be removed with version 23.06
    SofaRuntime.importPlugin("Sofa.Component.Constraint.Projective")
    SofaRuntime.importPlugin("Sofa.Component.MechanicalLoad")
    
    SofaRuntime.importPlugin("ROS2Plugin")

    #Create the root node
    root = Sofa.Core.Node("root")
    
    # Call the below 'createScene' function to create the scene graph
    createScene(root)
    Sofa.Simulation.init(root)
    
    
    

    if not USE_GUI:
        for iteration in range(100):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        # Find out the supported GUIs
        print ("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
        # Launch the GUI (qt or qglviewer)

        desired_gui_fps = 30  # Adjust as needed
        time_step = 1.0 / desired_gui_fps  # Calculate the desired time step
        
        Sofa.Gui.GUIManager.Init("myscene", "qt")   #qglviewer
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        # Initialization of the scene will be done here
        Sofa.Gui.GUIManager.MainLoop(root)
           
    
        Sofa.Gui.GUIManager.closeGUI()
        print("GUI was closed")

    print("Simulation is done.")


# Function called when the scene graph is being created
def createScene(root):
    
    root.gravity=[0.0,-9.81,0.0]        # Define the gravity
    root.dt = 0.1                       # Define the time step
    
    # Scene must now include a VisualLoop (Create a rendering loop to display in the graphic)
    root.addObject('DefaultVisualManagerLoop')

    # Scene must now include a AnimationLoop (Define the loop for the simulation after cliking the animation button)
    root.addObject('DefaultAnimationLoop')
    
    # Add the required Sofa plugins    
    root.addObject('RequiredPlugin', name="loadSOFAModules", \
        pluginName="Sofa.Component.LinearSolver.Iterative Sofa.Component.Mass \
                    Sofa.Component.MechanicalLoad Sofa.Component.StateContainer \
                    Sofa.Component.ODESolver.Backward \
                    Sofa.Component.Engine.Transform \
                    Sofa.Component.Engine.Analyze \
                    Sofa.Component.Engine.Generate \
                    Sofa.Component.Engine.Select \
                    Sofa.Component.Engine.Transform \
                    Sofa.GL.Component.Engine \
                    Sofa.Component.ODESolver.Backward \
                    Sofa.Component.Constraint.Projective Sofa.Component.MechanicalLoad \
                    Sofa.Component.SolidMechanics.FEM.Elastic \
                    Sofa.Component.Collision.Detection.Algorithm \
                    Sofa.Component.Collision.Detection.Intersection")
    
    # For non static object add the EulerImplicitSolver and CGLinearSolver
    root.addObject("EulerImplicitSolver", name="ODEsolver", rayleighStiffness="0.1", rayleighMass="0.1")
    root.addObject("CGLinearSolver", iterations="100", tolerance="1e-5", threshold="1e-5")
    
    # For Meshing
    root.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False)
    root.addObject('RequiredPlugin', name="Sofa.Component.Visual", printLog=False)
    root.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    
    # Path of the meshes (add a subscriber to obtain the position of the cutting tool)
    absolute_path_main = os.path.dirname(__file__)
        
    root.addObject("ROS2Context",name="ros2Context")
    
    root.addObject("ROS2Subscriber",name="pose_sub", template="RosRigid", \
        nodeName="pose_subscriber", topicName="/sofa/in/tool_pose", draw="1")
        
    ### Cutting Tool ###
    # Create the Cutting tool
    marker_node = root.addChild("CuttingTool")

    # Mechanical model (First start publishing position from the Publisher otherwise the position of the object is none -> start before of the simulation!)
    marker_node.addObject("MechanicalObject", name="cuttingToolMechs", template="Rigid3d", position="@../pose_sub.output", rotation="0 0 -90", showObject=0, showObjectScale=0.2)
        
    # Visual model
    incision_toll_mesh_path = os.path.join(absolute_path_main, 'eye_models/scalpel_cataract_v4.obj')                    # load the file of the visual model
        
    tool_visual = marker_node.addChild("VisualModel")                                                                   # add the visual model subnode
    tool_visual.loader = tool_visual.addObject('MeshOBJLoader', name="loader", filename=incision_toll_mesh_path)        # load the visual model from the file
    tool_visual.addObject('OglModel', name="tool_visual_model", src="@loader", scale3d=[0.05]*3, updateNormals=False)   # draw the visual model
    tool_visual.addObject('RigidMapping', input="@..", output="@.", index="0")                                          # Connect the visual model and the mechanical model


    #### End new Eye model ####
    
    ### Cornea ###
    cornea = root.addChild("Cornea")                                                                                    # Create the Cornea
    cornea.addObject('MeshSTLLoader', name = "STLLoader", \
        filename = os.path.join(absolute_path_main, 'eye_models/zeiss_models/Cornea.stl'))                              # load the visual model from the file
    # VisualModel
    cornea.addObject('OglModel', name="Visual", src='@STLLoader', alphaBlend="false", rotation="-90 90 0", \
        # feat Rebekka
        material="Default Diffuse 1 1 1 1 0.2 Ambient 1 1 1 1 1 Specular 0 0 0 0 0 Emissive 1 1 1 1 1 Shininess 1 100") # draw the visual model


    ### Iris ##
    iris = root.addChild("Iris")                                                                                        # Create the Iris
    iris.addObject('MeshSTLLoader', name = "STLLoader", \
        filename = os.path.join(absolute_path_main, 'eye_models/zeiss_models/Iris.stl'))                                # load the visual model from the file
    # VisualModel
    iris.addObject('OglModel', name="Visual", src = '@STLLoader', rotation="-90 90 0", color=[0, 0, 0.35])              # draw the visual model ([0.45, 0.23, 0] brown)


    ### Lense ###
    lense = root.addChild("Lense")                                                                                      # Create the Lense
    lense.addObject('MeshSTLLoader', name = "STLLoader", flipNormals = '0', \
        filename = os.path.join(absolute_path_main, 'eye_models/zeiss_models/Lense.stl'))                               # load the visual model from the file
    # Visual model 
    lense.addObject('OglModel', name="Visual", src = '@STLLoader', rotation="-90 90 0", color=[0.4, 0.4, 0.4])          # draw the visual model


    ### Sclera ###
    sclera = root.addChild("Sclera")                                                                                    # Create the Scera
    sclera.addObject('MeshSTLLoader', name = "STLLoader", flipNormals = "0", \
        filename = os.path.join(absolute_path_main, 'eye_models/zeiss_models/Sclera.stl'))                              # load the visual model from the file
    # Visual Model
    sclera.addObject('OglModel', name = "Visual", src = '@STLLoader', rotation="-90 90 0", color = "white")             # draw the visual model
    
    #### End new Eye model ####
    
    

    ###### Add a desk for reference ######

    desk = root.addChild("Desk")                                                                                        # Create the Desk
    desk.addObject('MeshOBJLoader', name = "Loader1", \
        filename = os.path.join(absolute_path_main, 'eye_models/floor.obj'))                                            # load the visual model from the file
    # draw the visual model    
    desk.addObject('OglModel', name = "Visual", src = '@Loader1', rotation="0 0 0", translation="0 -0.5 0", color=[0.2, 0.4, 0.4], scale3d=[0.1]*3, \
                   material="Default Diffuse 1 0.5 0.5 0.8 1 Ambient 1 0.1 0.1 0.1 1 Specular 0 0.5 0.5 0.5 1 Emissive 0 0.5 0.5 0.5 1 Shininess 0 45 No texture linked to the material No bump texture linked to the material ")
    
    # ######################################

    # built-in Ros2 publisher
    root.addObject("ROS2Publisher",name="point_pub", template="RosRigid", nodeName="point_publisher", topicName="sofa/out/point", canPublish="true", \
            #input="@TranslationEngine.output_position", draw="1")
            input="@EyePos/eyeMechObj.position", draw="1")
    
    # Collision Detection part
    container = root.addObject("PointSetTopologyContainer", name="CutSurfaceContainer", points=[[0, 0, 0]])     # load the point Topology to be able to draw it
    modifier = root.addObject("PointSetTopologyModifier", name="CutSurfaceModifier")                            # load the topology changes container. It defines the basic operations (add or remove)

    state = root.addObject("MechanicalObject", name="CutSurfaceMechObj", template="Vec3d", showObject=True, showObjectScale=5)
    root.addObject(ContactDetectionController(name="ContactDetectionSurface",rootNode=root, MechanicalObject=marker_node, modifier=modifier, state=state, container=container))

    return root

              
# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
