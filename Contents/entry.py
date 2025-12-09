import adsk.core
import adsk.fusion
import traceback
import os
import sys

# Asegurar que la carpeta del add-in esté en sys.path
this_dir = os.path.dirname(os.path.realpath(__file__))
if this_dir not in sys.path:
    sys.path.insert(0, this_dir)

# Importar módulo robot - manejar errores de importación
try:
    import robot
    from robot import RobotExporter, _log, _sanitize, ensure_dir
    IMPORT_SUCCESS = True
except ImportError as e:
    IMPORT_SUCCESS = False
    print(f"Error al importar módulo robot: {e}")
    print(f"Directorio actual: {this_dir}")
    print(f"Contenido del directorio: {os.listdir(this_dir)}")
except Exception as e:
    IMPORT_SUCCESS = False
    print(f"Error inesperado al importar: {e}")

# Mantener referencias a handlers para evitar GC
_handlers = []

# IDs básicos (puedes cambiarlos, pero que coincidan en todo el archivo)
CMD_ID = "URDFConverter_Command"
CMD_NAME = "URDF Converter"
CMD_TOOLTIP = "Export URDF + DAE with Low / High quality"
WORKSPACE_ID = "FusionSolidEnvironment"
PANEL_ID = "SolidScriptsAddinsPanel"
CONTROL_ID = CMD_ID  # normalmente igual al command id


# =========================================================
# Handlers
# =========================================================

class URDFCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args: adsk.core.CommandCreatedEventArgs):
        try:
            # Verificar que el módulo robot se importó correctamente
            if not IMPORT_SUCCESS:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox(
                    "Error: No se pudo cargar el módulo 'robot.py'. "
                    "Asegúrese de que el archivo existe en el mismo directorio.",
                    "Error - URDF Converter"
                )
                return
            
            cmd = args.command
            inputs = cmd.commandInputs

            # Evento Execute
            on_execute = URDFCommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)

            # --- Inputs UI ---

            # 1) Quality dropdown
            drop = inputs.addDropDownCommandInput(
                "quality_mode",
                "Mesh Quality",
                adsk.core.DropDownStyles.TextListDropDownStyle
            )
            drop.listItems.add("Very Low Quality (Fast)", True)
            drop.listItems.add("High Quality (DisplayMesh)", False)

            # 2) Robot name
            app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(app.activeProduct)

            default_name = "robot"
            if design and design.rootComponent and design.rootComponent.name:
                default_name = design.rootComponent.name

            inputs.addStringValueInput(
                "robot_name",
                "Robot Name",
                default_name
            )

            # 3) Información
            txt = inputs.addTextBoxCommandInput(
                "info_box",
                "",
                "The output folder will be chosen when you press OK.\n\n"
                "Very Low Quality: Fast export, optimized for robotics simulation.\n"
                "High Quality: Uses DisplayMesh for better visual fidelity.",
                3,
                True
            )
            txt.isFullWidth = True

        except Exception as e:
            # Log básico si falla algo al crear el comando
            try:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox(f"Error creating URDF command UI:\n{str(e)}\n\n{traceback.format_exc()}")
            except:
                print("Error in URDFCommandCreatedHandler:\n", traceback.format_exc())


class URDFCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args: adsk.core.CommandEventArgs):
        app = adsk.core.Application.get()
        ui = app.userInterface
        
        try:
            # Verificar que el módulo robot se importó correctamente
            if not IMPORT_SUCCESS:
                ui.messageBox(
                    "Error: El módulo 'robot.py' no se pudo cargar. "
                    "Reinicie Fusion 360 o verifique que el archivo existe.",
                    "Error - URDF Converter"
                )
                return
            
            cmd = args.command
            inputs = cmd.commandInputs

            # Leer modo de malla
            quality_input = adsk.core.DropDownCommandInput.cast(
                inputs.itemById("quality_mode")
            )
            if quality_input is None or quality_input.selectedItem is None:
                ui.messageBox("No quality mode selected.", "URDF Converter")
                return

            sel_name = quality_input.selectedItem.name
            if "Low" in sel_name:
                mesh_mode = "very_low"
                mode_display = "Very Low Quality"
            else:
                mesh_mode = "display"
                mode_display = "High Quality"

            # Nombre de robot
            name_input = adsk.core.StringValueCommandInput.cast(
                inputs.itemById("robot_name")
            )
            robot_name = name_input.value if name_input and name_input.value else "robot"
            
            # Sanitizar nombre
            try:
                robot_name = robot._sanitize(robot_name)
            except:
                # Fallback si no está disponible
                robot_name = "".join(c for c in robot_name if c.isalnum() or c in ('_', '-'))
                if not robot_name:
                    robot_name = "robot"

            # Elegir carpeta de salida
            folder_dlg = ui.createFolderDialog()
            folder_dlg.title = "Select output folder for URDF package"
            folder_dlg.initialDirectory = os.path.expanduser("~/Desktop")
            dlg_res = folder_dlg.showDialog()
            
            if dlg_res != adsk.core.DialogResults.DialogOK:
                return  # usuario canceló

            out_dir = folder_dlg.folder
            if not out_dir:
                ui.messageBox("Invalid output folder.", "URDF Converter")
                return

            # Obtener design
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox("No active Fusion design. Open a design and try again.", "URDF Converter")
                return

            # Mostrar diálogo de progreso
            progress_dlg = ui.createProgressDialog()
            progress_dlg.show(
                "URDF Export",
                f"Exporting {robot_name} with {mode_display}...",
                0,
                100,
                0
            )

            try:
                # Llamar a tu sistema de exportación
                progress_dlg.progressValue = 10
                
                # Usar logging del módulo robot si está disponible
                try:
                    robot._log(f"Starting URDF export to '{out_dir}' with mesh_mode='{mesh_mode}'")
                except Exception as log_err:
                    # Fallback si el logging falla
                    ui.palettes.itemById("TextCommands").writeText(
                        f"[URDFConverter] Starting export: {robot_name}, mode: {mesh_mode}\n"
                    )

                progress_dlg.progressValue = 30
                
                urdf_path = robot.RobotExporter.export_robot(
                    design,
                    out_dir,
                    robot_name,
                    mesh_mode=mesh_mode
                )

                progress_dlg.progressValue = 90
                progress_dlg.hide()

                if urdf_path and os.path.exists(urdf_path):
                    # Mostrar mensaje de éxito con opción para abrir carpeta
                    result = ui.messageBox(
                        f"✅ URDF export completed successfully!\n\n"
                        f"Robot: {robot_name}\n"
                        f"Quality: {mode_display}\n"
                        f"URDF file: {os.path.basename(urdf_path)}\n"
                        f"Location: {out_dir}\n\n"
                        f"Do you want to open the output folder?",
                        "Success - URDF Converter",
                        adsk.core.MessageBoxButtonTypes.YesNoButtonType
                    )
                    
                    if result == adsk.core.DialogResults.DialogYes:
                        # Abrir carpeta en explorador de archivos
                        if sys.platform == 'win32':
                            os.startfile(out_dir)
                        elif sys.platform == 'darwin':
                            os.system(f'open "{out_dir}"')
                        else:
                            os.system(f'xdg-open "{out_dir}"')
                else:
                    ui.messageBox(
                        "❌ URDF export failed.\n\n"
                        "Check the Text Commands palette (View -> Show Text Commands) "
                        "for detailed error messages.",
                        "Error - URDF Converter",
                        adsk.core.MessageBoxButtonTypes.OKButtonType,
                        adsk.core.MessageBoxIconTypes.CriticalIconType
                    )

            except Exception as export_err:
                progress_dlg.hide()
                ui.messageBox(
                    f"❌ Error during URDF export:\n\n{str(export_err)}\n\n"
                    "Check the Text Commands palette for more details.",
                    "Export Error - URDF Converter",
                    adsk.core.MessageBoxButtonTypes.OKButtonType,
                    adsk.core.MessageBoxIconTypes.CriticalIconType
                )
                # Log detallado en Text Commands
                ui.palettes.itemById("TextCommands").writeText(
                    f"[URDFConverter] Export error:\n{traceback.format_exc()}\n"
                )

        except Exception as e:
            try:
                ui.messageBox(
                    f"Unexpected error in URDF Converter:\n\n{str(e)}\n\n"
                    "Please check the Text Commands palette for details.",
                    "Unexpected Error - URDF Converter",
                    adsk.core.MessageBoxButtonTypes.OKButtonType,
                    adsk.core.MessageBoxIconTypes.CriticalIconType
                )
                ui.palettes.itemById("TextCommands").writeText(
                    f"[URDFConverter] Unexpected error:\n{traceback.format_exc()}\n"
                )
            except:
                print("Error in URDFCommandExecuteHandler:\n", traceback.format_exc())


# =========================================================
# run / stop
# =========================================================

def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        # Verificar importación
        if not IMPORT_SUCCESS:
            ui.messageBox(
                "Cannot load URDF Converter.\n\n"
                "Make sure 'robot.py' exists in the same folder as this script.",
                "URDF Converter - Import Error",
                adsk.core.MessageBoxButtonTypes.OKButtonType,
                adsk.core.MessageBoxIconTypes.CriticalIconType
            )
            return

        # Buscar o crear definición de comando
        cmd_defs = ui.commandDefinitions
        cmd_def = cmd_defs.itemById(CMD_ID)
        if not cmd_def:
            cmd_def = cmd_defs.addButtonDefinition(
                CMD_ID,
                CMD_NAME,
                CMD_TOOLTIP,
                ""  # ruta de iconos opcional
            )

        # Registrar handler de CommandCreated
        on_created = URDFCommandCreatedHandler()
        cmd_def.commandCreated.add(on_created)
        _handlers.append(on_created)

        # Añadir botón al panel de Scripts & Addins
        workspace = ui.workspaces.itemById(WORKSPACE_ID)
        panel = workspace.toolbarPanels.itemById(PANEL_ID)

        ctrl = panel.controls.itemById(CONTROL_ID)
        if not ctrl:
            panel.controls.addCommand(cmd_def, CONTROL_ID)
            # Mensaje de éxito en Text Commands
            try:
                ui.palettes.itemById("TextCommands").writeText(
                    f"[URDFConverter] ✅ Command '{CMD_NAME}' added to panel.\n"
                    f"[URDFConverter] Ready to export URDF models.\n"
                )
                print(f"[URDFConverter] Command '{CMD_NAME}' registered successfully.")
            except:
                print(f"[URDFConverter] Command '{CMD_NAME}' registered.")

    except Exception as e:
        try:
            ui.messageBox(
                f"Error starting URDF Converter:\n\n{str(e)}\n\n"
                "Check the console or Text Commands for more details.",
                "Startup Error - URDF Converter",
                adsk.core.MessageBoxButtonTypes.OKButtonType,
                adsk.core.MessageBoxIconTypes.CriticalIconType
            )
        except:
            print(f"Error in URDF Converter run():\n{traceback.format_exc()}")


def stop(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        # Quitar control del panel
        workspace = ui.workspaces.itemById(WORKSPACE_ID)
        panel = workspace.toolbarPanels.itemById(PANEL_ID)
        ctrl = panel.controls.itemById(CONTROL_ID)
        if ctrl:
            ctrl.deleteMe()

        # Borrar definición de comando
        cmd_def = ui.commandDefinitions.itemById(CMD_ID)
        if cmd_def:
            cmd_def.deleteMe()

        # Mensaje en Text Commands
        try:
            ui.palettes.itemById("TextCommands").writeText(
                f"[URDFConverter] Command '{CMD_NAME}' removed.\n"
            )
        except:
            pass

    except Exception as e:
        try:
            ui.messageBox(
                f"Error stopping URDF Converter:\n\n{str(e)}",
                "Stop Error - URDF Converter",
                adsk.core.MessageBoxButtonTypes.OKButtonType
            )
        except:
            print(f"Error in URDF Converter stop():\n{traceback.format_exc()}")