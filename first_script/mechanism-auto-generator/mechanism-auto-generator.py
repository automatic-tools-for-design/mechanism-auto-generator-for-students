import adsk.core, adsk.fusion, traceback

def run(context):
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

        # Define the sketch plane (e.g., XY plane)
        # You can also select a face or a construction plane
        xyPlane = rootComp.xYConstructionPlane

        # Create a new sketch on the selected plane
        sketch = rootComp.sketches.add(xyPlane)

        # Draw some geometry on the sketch (e.g., a rectangle)
        sketchLines = sketch.sketchCurves.sketchLines
        point1 = adsk.core.Point3D.create(0, 0, 0)
        point2 = adsk.core.Point3D.create(5, 3, 0)
        sketchLines.addTwoPointRectangle(point1, point2)

        # Optionally, add other geometry like circles, arcs, etc.
        # sketchCircles = sketch.sketchCurves.sketchCircles
        # centerPoint = adsk.core.Point3D.create(2.5, 1.5, 0)
        # sketchCircles.addByCenterRadius(centerPoint, 1)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))