import traceback, os, sys, platform

import adsk.core
import adsk.fusion
app = adsk.core.Application.get()
ui  = app.userInterface


try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    import core_obf

except:
    ui.messageBox("IMPORT FAILED:\n" + traceback.format_exc())
    raise



def run(context):
    try:
        if not app.activeProduct:
            ui.messageBox("No active product. Open a design or assembly first.")
            return

        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox("No active Fusion design open.")
            return

        # Cross-OS Downloads path
        if platform.system() == "Windows":
            downloads_path = os.path.join(os.getenv("USERPROFILE"), "Downloads")
        else:
            downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")

        # -------------------------------
        # STUDENT-EDITABLE SECTION (choose JSON)
        # -------------------------------
        script_dir = os.path.dirname(__file__)
        json_path = os.path.join(script_dir, "4BARMECH.json")
        #json_path = os.path.join(script_dir, "6BARMECH_WATT_I.json")
        #json_path = os.path.join(script_dir, "6BARMECH_STEPHENSON_II.json")
        #json_path = os.path.join(script_dir, "Theo_Jansen.json")
        # -------------------------------

        # Run the obfuscated core
        mech, root_comp = core_obf.run_with_json(json_path, theta_crank=0)


        ui.messageBox(
            "Mechanism parsed successfully!\n\n"
            "Links:   {}\n"
            "Joints:  {}\n"
            "Groups:  {}\n"
            "Crank:   {}\n".format(
                len(mech.links),
                len(mech.joints),
                len(mech.groups),
                "Yes" if mech.crank is not None else "No",
            )
        )

        result = ui.messageBox(
            "Do you want to Export STL files of your all parts to the Downloads folder?",
            "Export STL files",
            adsk.core.MessageBoxButtonTypes.YesNoButtonType
        )

        if result == adsk.core.DialogResults.DialogYes:
            core_obf.export_all_stl(root_comp, downloads_path)
            ui.messageBox("Files exported - script complete")
        else:
            ui.messageBox("Script complete")

    except:
        if ui:
            ui.messageBox("Error:\n{}".format(traceback.format_exc()))
