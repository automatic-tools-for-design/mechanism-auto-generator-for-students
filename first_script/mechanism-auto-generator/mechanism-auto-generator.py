from turtledemo.penrose import start

import adsk.core, adsk.fusion, traceback
import os
import json
import math


class link:
    def __init__(self, start,end, width,radius,thickness,rootComp):
        self.start=start
        self.end=end
        self.length = ((start[0]-end[0])**2+(start[1]-end[1])**2)**.5
        self.width = width
        self.radius = radius
        self.root=rootComp
        self.thickness=thickness


    def generate(self):
        xyPlane = self.root.xYConstructionPlane

        sketch = self.root.sketches.add(xyPlane)
        y_diff=self.end[1]-self.start[1]
        x_diff = self.end[0] - self.start[0]
        angle=math.atan2(y_diff,x_diff)+math.pi/2

        center_point=adsk.core.Point3D.create((self.start[0]+self.end[0])/2, (self.start[1]+self.end[1])/2, 0)

        corner_point1 = adsk.core.Point3D.create(self.start[0] + self.width * math.cos(angle) / 2,
                                                 self.start[1] + self.width * math.sin(angle) / 2, 0)
        corner_point2 = adsk.core.Point3D.create(self.start[0] - self.width * math.cos(angle) / 2,
                                                 self.start[1] - self.width * math.sin(angle) / 2, 0)
        corner_point3 = adsk.core.Point3D.create(self.end[0] - self.width * math.cos(angle) / 2,
                                                 self.end[1] - self.width * math.sin(angle) / 2, 0)


        # app = adsk.core.Application.get()
        # ui = app.userInterface
        # ui.messageBox(f'{[(self.start[0]+self.end[0])/2, (self.start[1]+self.end[1])/2, 0,self.start[0]+self.width*math.cos(angle), self.start[1]+self.width*math.sin(angle), 0]}')
        sketchLines = sketch.sketchCurves.sketchLines
        # point1 = adsk.core.Point3D.create(0, 0, 0)
        # point2 = adsk.core.Point3D.create(self.length, self.width, 0)
        #sketchLines.addTwoPointRectangle(corner_point1, corner_point3)


        sketchLines.addThreePointRectangle(corner_point1,corner_point2,corner_point3)

        sketchCircles = sketch.sketchCurves.sketchCircles
        center_1 = adsk.core.Point3D.create(self.start[0],self.start[1], 0)
        center_2 = adsk.core.Point3D.create(self.end[0], self.end[1], 0)
        sketchCircles.addByCenterRadius(center_1, self.width / 2)
        sketchCircles.addByCenterRadius(center_1, self.radius)
        sketchCircles.addByCenterRadius(center_2, self.width / 2)
        sketchCircles.addByCenterRadius(center_2, self.radius)

        profilesToExtrude = adsk.core.ObjectCollection.create()
        for idx, profile in enumerate(sketch.profiles):
            #todo figure out a better way to do this
            #if idx in [0,1,2,4,6]: #this does not appear to be consistent and depends on the location of the body
            if idx <10:
                profilesToExtrude.add(profile)

        # Get the extrude features collection
        extrudes = self.root.features.extrudeFeatures

        # Create an ExtrudeFeatureInput
        extrudeInput = extrudes.createInput(profilesToExtrude, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # Define the extrusion distance
        distance = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(self.thickness))  # 1 cm extrusion
        extrudeInput.setOneSideExtent(distance, adsk.fusion.ExtentDirections.PositiveExtentDirection)

        # Create the extrusion
        extrudes.add(extrudeInput)






def run(context):
    script_path = os.path.abspath(__file__)
    script_directory = os.path.dirname(script_path)
    os.chdir(script_directory)
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        
        # Get the active design
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active design', 'No Design')
            return

        # Get the root component
        rootComp = design.rootComponent

        file_path="linkage_params.json"
        with open(file_path, 'r') as file:
            data = json.load(file)
            num_links=data.get("num_links")


        # Draw some geometry on the sketch (e.g., a rectangle)
        width=3
        hole_radius=1
        thickness=1
        for i in range(num_links):
            linkage_data=data.get(f'Link{i}')
            start=linkage_data.get("start")
            end=linkage_data.get("end")
            link1=link(start,end,width,hole_radius,thickness,rootComp)
            link1.generate()



    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))