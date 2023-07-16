bl_info = {
    "name": "Export XFlr5",
    "author": "Ian Huish",
    "version": (2, 15, 2),
    "blender": (3, 00, 0),
    "location": "Export > XFlr5",
    "description": "Export Plane model to XFlr5",
    "warning": "",
    "wiki_url": "https://github.com/nerk987/export_xflr5",
    "tracker_url": "http://github.com/nerk987/export_xflr5/issues",
    "category": "Export",
}

#v1.02 Handle fin

import bpy
import bmesh
from bpy_extras.object_utils import AddObjectHelper
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty, FloatProperty
import mathutils.geometry
import xml.etree.ElementTree as ET
import os
import platform
from math import *
from bpy.props import (
        FloatProperty,
        IntProperty,
        )

#********************************************************
#  Sample Curve
#********************************************************

def dist(point1, point2):
    return ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2 + (point1[2]-point2[2])**2)**0.5

def interp(point1, point2, factor, thickness):
    return([(point2[0]-point1[0])*factor+point1[0],(point2[1]-point1[1])*factor+point1[1],((point2[2]-point1[2])*factor+point1[2]) * thickness])

def Vec2co(vec):
    return [vec.x, vec.y, vec.z]

def SampleCurve(curve, fraction, start):
    if curve == None:
        return [[0,0,0],0]
    #determine resolution
    res = 100
    
    #get SegData [len,..]for approx lengths of each segment
    spline = curve.data.splines[0]
    SegData = []
    TotalAproxLen = 0
    segments = len(spline.bezier_points)-1
    for i in range(segments):
        AproxLen = dist(spline.bezier_points[i].co, spline.bezier_points[i+1].co)
        SegData.append(AproxLen)
        TotalAproxLen = TotalAproxLen + AproxLen
    
    #get SegLen [[len, co1, co2], ..] for cumlative length of each mini segment
    SegLen = []
    AccumLen = 0.0
    for i in range(segments):
        inext = (i + 1) % len(spline.bezier_points)

        knot1 = spline.bezier_points[i].co
        handle1 = spline.bezier_points[i].handle_right
        handle2 = spline.bezier_points[inext].handle_left
        knot2 = spline.bezier_points[inext].co

        r = int(res * SegData[i] / TotalAproxLen)
        if r < 5:
            r = 5
        points = mathutils.geometry.interpolate_bezier(knot1, handle1, handle2, knot2, r)
        for j in range(len(points)-1):
            MiniLen = dist(Vec2co(points[j]), Vec2co(points[j+1]))
            if j >= 1:
                prev = j-1
            else:
                prev = j
            SegLen.append([AccumLen, Vec2co(points[j]), Vec2co(points[prev])])
            AccumLen = AccumLen + MiniLen

    #get TotalLen
    TotalLen = AccumLen
    
    #from start, move forward until correct len/TotalLen > fraction found
    i = start
    for i in range(start, len(SegLen)): 
        if SegLen[i][0]/TotalLen >= fraction:
            break
        
    #interpolate current and previous points
    if i >= 1:
        SegFract = (fraction - SegLen[i-1][0]/TotalLen)/((SegLen[i][0] - SegLen[i-1][0])/TotalLen)
        #print("SegFract, i, SegLen[i][1], SegLen[i-1][0], fraction, TotalLen: ", SegFract, i, SegLen[i][0], SegLen[i-1][0], fraction, TotalLen)
    else:
        SegFract = 0.0
    coord = interp(SegLen[i][2], SegLen[i][1], SegFract, 1.0)
    #print("Coords: ", coord, SegLen[i][1])
    #return [co,location]
    return [coord, i]

#********************************************************
#  Write airfoil file
#********************************************************

def WriteDatFile(filepath, root, tip, factor, thickness, i):
    suffix = "{0:04.0f}".format(i)
    airfoilName = os.path.basename(filepath)[:-4]+suffix
    
    filename = filepath[:-4]+suffix +".dat"
    #print("Filepath: ", filename)
    f = open(filename, 'w', encoding='utf-8')
    f.write(airfoilName + "\n")
    j = 0
    #Check that the root and tip airfoils have the same number of vertices
    if len(root.data.vertices) != len(tip.data.vertices):
        return -1
    for vert in root.data.vertices:
        co_root = root.data.vertices[j].co
        co_tip = tip.data.vertices[j].co
        co_merge = interp(co_root, co_tip, factor, thickness)
        f.write("{:.7f}".format(co_merge[1]) + " " + "{:.7f}".format(co_merge[2]) + "\n")
        j = j+1
    f.close()
    return 0    



#********************************************************
#  Operator
#********************************************************

class ExportXFlr5(bpy.types.Operator, ExportHelper):
    """Export to XFlr5"""
    bl_idname = "mesh.export_xflr5"
    bl_label = "Export to XFlr5"
    bl_options = {'REGISTER', 'UNDO'}

     # ImportHelper mixin class uses this
    filename_ext = ".xml"

    filter_glob: StringProperty(
        default="*.xml",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )
    
    def execute(self, context):
        
        #Check the correct object type is selected and finish with an error if not
        ob = context.active_object
        err = False
        isFin = False
        if ob == None:
            err = True
        elif "GeometryNodes" not in ob.modifiers:
            err = True
        elif ob.modifiers["GeometryNodes"].node_group == None:
            err = True
        elif ob.modifiers["GeometryNodes"].node_group.name not in ["WingV2", "FinV2", "WingV3", "FinV3"]:
            err = True
            
        if err:        
            self.report({"WARNING"}, "Couldn't find a Geo Node modifier with node 'WingV2' or 'FinV2' on the currently selected object")
            return {'FINISHED'}
        
        #Define span panels per meter etc
        SpanPanels = context.scene.xFlr5_props.SpanPanels
        ChordPanels = context.scene.xFlr5_props.ChordPanels
        MinSpanPanels = 2
        MinChordPanels = 2
        
        #Get data from the main Geo Node interface
        surface = context.object
        RootAirfoil = surface.modifiers["GeometryNodes"]["Input_4"]
        try:
            TipAirfoil = surface.modifiers["GeometryNodes"]["Input_5"]
        except:
            TipAirfoil = RootAirfoil
        Leading = surface.modifiers["GeometryNodes"]["Input_6"]
        Trailing = surface.modifiers["GeometryNodes"]["Input_7"]
        try:
            Twist = surface.modifiers["GeometryNodes"]["Input_10"]
        except:
            Twist = None
        try:
            TwistCenter = surface.modifiers["GeometryNodes"]["Input_11"]
        except:
            TwistCenter = None
        try:
            Interpolate = surface.modifiers["GeometryNodes"]["Input_13"]
        except:
            Interpolate = None
        RibCount = surface.modifiers["GeometryNodes"]["Input_2"]
        TipRibCount = surface.modifiers["GeometryNodes"]["Input_8"]
        TipFraction = surface.modifiers["GeometryNodes"]["Input_9"]
        try:
            RootCount = surface.modifiers["GeometryNodes"]["Input_22"]
            RootFraction = surface.modifiers["GeometryNodes"]["Input_23"]
        except:
            RootCount = -1
            RootFraction = 0.0
        try:
            Thickness = surface.modifiers["GeometryNodes"]["Input_18"]
        except:
            Thickness = None
        
        
        #Get path of wing.xml file included with the addon
        if platform.system() == 'Windows':
            directory = os.path.join(os.path.dirname(__file__), "xml")
            if directory and os.path.exists(directory):
                src_file = directory + "\\wing.xml"
            else:
                return {'FINISHED'}
        else:
            directory = os.path.join(os.path.dirname(__file__), "xml")
            if directory and os.path.exists(directory):
                src_file = directory + "/wing.xml"
            else:
                return {'FINISHED'}
            

        #Access the Plane node XML file
        flrTree = ET.parse(src_file)
        flrRoot = flrTree.getroot()
        flrPlane = flrRoot.find("xplane")
                
        #Find a reference to the wing and update type
        wing = flrRoot.find("wing")
        if wing == None:
            return {'FINISHED'}
        
        #handle fin
        if ob.modifiers["GeometryNodes"].node_group.name[:3] == "Fin":
            print("fin!")
            wing.find("isFin").text = "true"
            isFin = True
        
        #record wing location, now including leading edge data
        [LeadingCoord, LeadingStart]  = SampleCurve(Leading, 0.0,0)
        wing.find("Position").text = "{0:.2f}, {1:.2f}, {2:.2f}".format(ob.location[1], ob.location[0], ob.location[2] + LeadingCoord[2])
        
        #Delete all the Sections
        sections = wing.find("Sections")
        for i in range(len(sections)):
             sections.remove(sections[0])
        
        #Initialise variables for loop
        RootInc = 0
        RibInc = 0
        TipInc = 0
        LeadingStart = 0
        TrailingStart = 0
        
        #Handle version 1 of the Wing Object
        if RootCount < 2:
            RootCountTot = 0
            RootFraction = 0.0
        else:
            RootCountTot = RootCount - 1
        

        #For each rib, add a new Section
        for i in range(RootCountTot + RibCount + TipRibCount):
            #Determine location of this section
            WingFract = 1.0 -((RootFraction)*RootInc/(RootCount-1) + (1-TipFraction-RootFraction)*RibInc/(RibCount) + TipFraction*TipInc/(TipRibCount))
            
            print("")
            print("Tip Count, Rib Count, Root Count, i: ", TipInc, RibInc, RootInc, i)
            #print("Adding section: ", WingFract, RibInc, TipInc, RootInc)
            if RootInc <= RootCount-1.5:
                RootInc = RootInc + 1
            elif RibInc <= RibCount-0.5:
                RibInc = RibInc + 1
            else:
                TipInc = TipInc + 1

            #Look up the guiding curves for parameters
            [LeadingCoord, LeadingStart]  = SampleCurve(Leading, WingFract,0)
            [TrailingCoord, TrailingStart]  = SampleCurve(Trailing, WingFract, 0)
            [InterpCoord, InterpStart]  = SampleCurve(Interpolate, WingFract, 0)
            [TwistCoord, TwistStart]  = SampleCurve(Twist, WingFract, 0)
            [ThicknessCoord, ThicknessStart]  = SampleCurve(Thickness, WingFract, 0)
            ThicknessVal = ThicknessCoord[2] * 25.0 + 1.0
            if isFin:
                temp = LeadingCoord[0]
                LeadingCoord[0] = LeadingCoord[2]
                LeadingCoord[2] = temp

            #Look at the next section for dihedral and panel count
            if i < (RootCountTot + RibCount + TipRibCount):
                WingFractNext = 1.0 -((RootFraction)*RootInc/(RootCount-1) + (1-TipFraction-RootFraction)*RibInc/(RibCount) + TipFraction*TipInc/(TipRibCount))
                [LeadingCoordNext, LeadingStartNext]  = SampleCurve(Leading, WingFractNext,0)
                [TrailingCoordNext, TrailingStartNext]  = SampleCurve(Trailing, WingFractNext,0)
                if isFin:
                    temp = LeadingCoordNext[0]
                    LeadingCoordNext[0] = LeadingCoordNext[2]
                    LeadingCoordNext[2] = temp
                    
                Dihed = degrees(atan(abs(LeadingCoordNext[2]-LeadingCoord[2])/abs(LeadingCoordNext[0]-LeadingCoord[0])))
                xpanels = max(MinChordPanels, int(ChordPanels * abs(LeadingCoord[1]-TrailingCoord[1])))
                ypanels = max(MinSpanPanels, int(SpanPanels * abs(LeadingCoordNext[0] - LeadingCoord[0])))
                print("Xpanels, ypanels: ", xpanels, ypanels)
                print("Spans: ", LeadingCoordNext[0], LeadingCoord[0], abs(LeadingCoordNext[0] - LeadingCoord[0]), int(SpanPanels * abs(LeadingCoordNext[0] - LeadingCoord[0])))
                print("Chords: ", abs(LeadingCoord[1]-TrailingCoord[1]))
            else:
                Dihed = 0.0
                
            #Add a new section
            print("WingFraction next Lead, Trail, Chord: ", WingFract, WingFractNext, LeadingCoord[1], TrailingCoord[1], abs(LeadingCoord[1] - TrailingCoord[1]))
            print("Y Value: ", LeadingCoord[0])
            newSection = ET.SubElement(sections, "Section")
            ET.SubElement(newSection, "y_position").text = "{0:.3f}".format(abs(LeadingCoord[0]))
            ET.SubElement(newSection, "Chord").text = "{0:.3f}".format(abs(LeadingCoord[1] - TrailingCoord[1]))
            ET.SubElement(newSection, "xOffset").text = "{0:.3f}".format(LeadingCoord[1])
            ET.SubElement(newSection, "Dihedral").text = "{0:.3f}".format(Dihed)
            ET.SubElement(newSection, "Twist").text = "{0:.3f}".format(-TwistCoord[2]*1000.0)
            ET.SubElement(newSection, "x_number_of_panels").text = "{0:.0f}".format(xpanels)
            ET.SubElement(newSection, "x_panel_distribution").text = "COSINE"
            ET.SubElement(newSection, "y_number_of_panels").text = "{0:.0f}".format(ypanels)
            ET.SubElement(newSection, "y_panel_distribution").text = "INVERSE SINE"
            
            #Handle airfoils
            MultiAirfoil = ((RootAirfoil.name != TipAirfoil.name) and (RootAirfoil.data.name != TipAirfoil.data.name))
            airfoilPrefix = os.path.basename(self.filepath)[:-4]
            if not MultiAirfoil: #Only one airfoil used
                airfoilname1 = airfoilPrefix + '0000'
                airfoilname2 = airfoilname1
            else:
                suffix = "{0:04.0f}".format(i)
                airfoilname1 = airfoilPrefix + suffix
                if i < (RootCountTot + RibCount + TipRibCount - 1): #Repeat the last airfoil because it's not a real section
                    print("Writing Last Airfoil")
                    suffix = "{0:04.0f}".format(i+1)
                airfoilname2 = airfoilPrefix + suffix
                    
            ET.SubElement(newSection, "Left_Side_FoilName").text = airfoilname1
            ET.SubElement(newSection, "Right_Side_FoilName").text = airfoilname2

            print("Ready to write an airfoil dat file:", self.filepath,i)
            #Write an Airfoil Dat file
            if (i == 0) or MultiAirfoil:
                print("Write an airfoil dat file:", self.filepath,i)
                if WriteDatFile(self.filepath, RootAirfoil, TipAirfoil, InterpCoord[2], ThicknessVal, i) != 0:
                    self.report({"WARNING"}, "Root and tip airfoils must have the same number of vertices")
                    return {'FINISHED'}        
                print("Thickness Value: ", ThicknessVal)

#Write the airfoil for the tip
#        WriteDatFile(self.filepath, RootAirfoil, TipAirfoil, InterpCoord[2], i+1)


        #Write over the xml file    
        path2 = self.filepath[:-4] + ".xml"    
        flrTree.write(path2)
    
        return {'FINISHED'}
    
#****************************************
#* Panel properties
#*******************************************

class xFlr5Props(bpy.types.PropertyGroup):        
    SpanPanels: IntProperty(
            name="Span Panels",
            description="Span Panels/m",
            default=100,
            min=1,
            soft_max=1000
            )
    ChordPanels: IntProperty(
            name="Chord Panels",
            description="Chord Panels/m",
            default=100,
            min=1,
            soft_max=1000
            )

#****************************************
#* Panel
#*******************************************

class xFlr5Panel(bpy.types.Panel):
    """Creates a Panel in the scene context of the properties editor"""
    bl_label = "xFlr5 Export"
    bl_idname = "SCENE_PT_xflr5export"
    bl_space_type = "VIEW_3D"
    bl_context = "objectmode"
    bl_region_type = "UI"
    bl_category = "xFlr5"

    @classmethod
    def poll(cls, context):
        ob = bpy.context.active_object
        if ob == None:
            return False
        if ob.modifiers["GeometryNodes"] == None:
            return False
        if ob.modifiers["GeometryNodes"].node_group == None:
            return False
        if ob.modifiers["GeometryNodes"].node_group.name  not in ["WingV2", "FinV2", "WingV3", "FinV3"]:
            return False
        
        return True

    def draw(self, context):
        layout = self.layout

        scene = context.scene

        # Create a simple row.
        layout.label(text="Panels per metre")

        layout.prop(scene.xFlr5_props, "SpanPanels")
        layout.prop(scene.xFlr5_props, "ChordPanels")


def menu_func(self, context):
    self.layout.operator(ExportXFlr5.bl_idname, icon='MESH_CUBE')

# Register and add to the "add mesh" menu (required to use F3 search "Add Box" for quick access)
def register():
    bpy.utils.register_class(xFlr5Panel)
    bpy.utils.register_class(ExportXFlr5)
    bpy.utils.register_class(xFlr5Props)
    bpy.types.TOPBAR_MT_file_export.append(menu_func)
    bpy.types.Scene.xFlr5_props = bpy.props.PointerProperty(type=xFlr5Props)


def unregister():
    bpy.utils.unregister_class(xFlr5Props)
    bpy.utils.unregister_class(ExportXFlr5)
    bpy.utils.unregister_class(xFlr5Panel)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func)
    del bpy.types.Scene.xFlr5_props


if __name__ == "__main__":
    register()

    # test call
#    bpy.ops.mesh.airfoil_add()
