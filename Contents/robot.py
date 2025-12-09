# =========================================================
#  BLOQUE 1 / 8 - UTILIDADES BASE + TRANSFORMACIONES
#                 (SISTEMA CÓDIGO 1 + GENERALIZADO + DEBUG)
# =========================================================

import adsk.core
import adsk.fusion
import os
import math
import traceback
import struct
import zlib
import re
from typing import Optional, Tuple, List, Dict, Any, Union

_EPS = 1e-9

# ---------------------------------------------------------
#  FLAGS DE DEBUG GLOBAL
# ---------------------------------------------------------
# Si te inunda de mensajes, ponlos en False.
_DEBUG_TRANSFORMS = True   # Poses globales/relativas, origins, etc.
_DEBUG_MESH_TREE  = True   # Qué bodies/meshes cuelgan de cada occurrence
_DEBUG_LINK_JOINT = True   # Estructura de joints, quién es parent/child, etc.

# ---------------------------------------------------------
#  App / UI + Logging unificado
# ---------------------------------------------------------

def _get_app_ui() -> Tuple[Optional[adsk.core.Application], Optional[adsk.core.UserInterface]]:
    """SISTEMA EXACTO Código 1 - Versión robusta."""
    try:
        app = adsk.core.Application.get()
        if app is None:
            print("[URDFConverter] ERROR: No se pudo obtener la aplicación")
            return None, None
        ui = app.userInterface
        return app, ui
    except Exception as e:
        print(f"[URDFConverter] ERROR en _get_app_ui: {e}")
        return None, None


def _log(msg: str, also_messagebox: bool = False, level: str = "INFO"):
    """SISTEMA EXACTO Código 1 - Versión mejorada de logging."""
    app, ui = _get_app_ui()

    try:
        full = f"[URDFConverter][{level}] {msg}"
        print(full)

        # Log en la paleta de texto si está disponible
        if ui:
            try:
                palette = ui.palettes.itemById("TextCommands")
                if palette:
                    palette.writeText(full + "\n")
            except:
                pass

        # Mostrar messagebox si se solicita
        if also_messagebox and ui:
            try:
                if level == "ERROR":
                    ui.messageBox(
                        msg,
                        "Error - URDF Converter",
                        adsk.core.MessageBoxButtonTypes.OKButtonType,
                        adsk.core.MessageBoxIconTypes.CriticalIconType
                    )
                else:
                    ui.messageBox(msg, "URDF Converter")
            except:
                print(f"[URDFConverter] No se pudo mostrar messagebox: {msg}")
    except Exception as e:
        print(f"[URDFConverter][LOG-ERROR] Error al registrar: {e} - Mensaje original: {msg}")


# Alias sencillo para compatibilidad con otros módulos / entry.py
def log(msg: str):
    """Alias de _log a nivel INFO (compatibilidad)."""
    _log(msg, also_messagebox=False, level="INFO")


# ---------------------------------------------------------
#  Sanitizado de nombres
# ---------------------------------------------------------

def _sanitize(name: str) -> str:
    """SISTEMA EXACTO Código 1 para limpiar nombres de links/joints/meshes."""
    if not name:
        return "link"
    bad = '<>:"/\\|?* ,.'
    for c in bad:
        name = name.replace(c, '_')
    if name and name[0].isdigit():
        name = "l_" + name
    return name


def sanitize(name: str) -> str:
    """Alias público para compatibilidad con otros bloques."""
    return _sanitize(name)


# ---------------------------------------------------------
#  Utilidades de fichero / carpetas
# ---------------------------------------------------------

def ensure_dir(path: str) -> bool:
    """Crea carpeta si no existe."""
    if not path:
        return False
    try:
        if os.path.exists(path):
            if not os.path.isdir(path):
                return False
            return True
        os.makedirs(path, exist_ok=True)
        return True
    except Exception as e:
        _log(f"Error al crear directorio {path}: {e}", level="ERROR")
        return False


# ---------------------------------------------------------
#  Utilidades vectoriales
# ---------------------------------------------------------

def vec_length(v: Tuple[float, float, float]) -> float:
    """Longitud de un vector 3D."""
    try:
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    except Exception:
        return 0.0


def vec_normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Normaliza un vector 3D, con fallback robusto."""
    try:
        n = vec_length(v)
        if n < _EPS:
            return (0.0, 0.0, 1.0)
        return (v[0] / n, v[1] / n, v[2] / n)
    except Exception:
        return (0.0, 0.0, 1.0)


# ---------------------------------------------------------
#  Helpers de descripción / debug de entidades
# ---------------------------------------------------------

def _describe_entity(entity: Any) -> str:
    """
    Devuelve una descripción corta de una entidad Fusion:
      - tipo (objectType o type)
      - nombre
      - fullPathName si es occurrence
      - assemblyContext si es proxy
    """
    try:
        if entity is None:
            return "<None>"

        try:
            obj_type = getattr(entity, "objectType", type(entity).__name__)
        except:
            obj_type = type(entity).__name__

        name = ""
        try:
            name = getattr(entity, "name", "")
        except:
            pass

        occ_ctx = None
        try:
            occ_ctx = getattr(entity, "assemblyContext", None)
        except:
            occ_ctx = None

        ctx_str = ""
        if occ_ctx and occ_ctx is not entity:
            ctx_name = getattr(occ_ctx, "name", "")
            ctx_full = getattr(occ_ctx, "fullPathName", ctx_name or "?")
            ctx_str = f" [ctx_occ='{ctx_full}']"

        # Si es occurrence, añade fullPathName
        if hasattr(entity, "fullPathName"):
            fp = getattr(entity, "fullPathName", "")
            if fp:
                return f"{obj_type}('{name}', fullPath='{fp}'){ctx_str}"

        return f"{obj_type}('{name}'){ctx_str}"
    except Exception as e:
        return f"<{type(entity).__name__} (desc_error={e})>"


def _debug_matrix(label: str, m: Optional[adsk.core.Matrix3D]):
    """Imprime una matriz 4x4 (R + T) de forma legible."""
    if not _DEBUG_TRANSFORMS:
        return

    if m is None:
        _log(f"[TFM] {label}: Matrix = <None>", level="DEBUG")
        return

    try:
        p = m.translation
        r11, r12, r13 = m.getCell(0, 0), m.getCell(0, 1), m.getCell(0, 2)
        r21, r22, r23 = m.getCell(1, 0), m.getCell(1, 1), m.getCell(1, 2)
        r31, r32, r33 = m.getCell(2, 0), m.getCell(2, 1), m.getCell(2, 2)

        _log(
            "[TFM] {lbl}: T(cm)=({:.3f},{:.3f},{:.3f}) "
            "R=[[{:.4f},{:.4f},{:.4f}], "
            "   [{:.4f},{:.4f},{:.4f}], "
            "   [{:.4f},{:.4f},{:.4f}]]".format(
                p.x, p.y, p.z,
                r11, r12, r13,
                r21, r22, r23,
                r31, r32, r33
            ).format(lbl=label),
            level="DEBUG"
        )
    except Exception as e:
        _log(f"[TFM] {label}: error al imprimir matriz: {e}", level="DEBUG")


# ---------------------------------------------------------
#  Clave estable para occurrences
# ---------------------------------------------------------

def get_occurrence_key(occ: adsk.fusion.Occurrence) -> str:
    """
    SISTEMA EXACTO Código 1:
    Obtiene una clave estable para mapear occurrences -> links.
    """
    if not occ:
        return ""
    try:
        token = getattr(occ, "entityToken", None)
        if token:
            return token
    except:
        pass
    try:
        fp = getattr(occ, "fullPathName", None)
        if fp:
            return fp
    except:
        pass
    try:
        return f"{occ.name}_{id(occ)}"
    except:
        return str(id(occ))


# ---------------------------------------------------------
#  Matrices y conversiones a xyz / rpy (CÓDIGO 1)
# ---------------------------------------------------------

def _matrix_to_xyz_rpy(m: adsk.core.Matrix3D,
                       design: adsk.fusion.Design) -> Tuple[Tuple[float, float, float],
                                                            Tuple[float, float, float]]:
    """
    SISTEMA EXACTO Código 1:
    - Lee la matriz de Fusion (en cm)
    - Convierte traslación a metros
    - Calcula RPY (roll, pitch, yaw).
    """
    if m is None:
        if _DEBUG_TRANSFORMS:
            _log("[TFM] _matrix_to_xyz_rpy: matriz None → xyz=(0,0,0), rpy=(0,0,0)", level="DEBUG")
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    if _DEBUG_TRANSFORMS:
        _debug_matrix("_matrix_to_xyz_rpy.input", m)

    p = m.translation
    try:
        # Fusion internamente trabaja en cm, convertimos a m
        scale = design.unitsManager.convert(1.0, "cm", "m")
    except Exception:
        scale = 0.01

    x = p.x * scale
    y = p.y * scale
    z = p.z * scale

    r11, r12, r13 = m.getCell(0, 0), m.getCell(0, 1), m.getCell(0, 2)
    r21, r22, r23 = m.getCell(1, 0), m.getCell(1, 1), m.getCell(1, 2)
    r31, r32, r33 = m.getCell(2, 0), m.getCell(2, 1), m.getCell(2, 2)

    if abs(r31) < 1.0:
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)
    else:
        pitch = math.pi / 2 if r31 <= -1.0 else -math.pi / 2
        roll = 0.0
        yaw = math.atan2(-r12, r22)

    xyz = (x, y, z)
    rpy = (roll, pitch, yaw)

    if _DEBUG_TRANSFORMS:
        _log(f"[TFM] _matrix_to_xyz_rpy → xyz(m)={xyz}, rpy(rad)={rpy}", level="DEBUG")

    return xyz, rpy


class TransformUtils:
    """
    SISTEMA DE TRANSFORMACIONES GENERALIZADO basado en el Código 1.

    Objetivo:
      - Obtener pose ABSOLUTA de *cualquier* entidad de Fusion 360
        (occurrences, cuerpos BRep, MeshBodies, proxies, etc.)
      - Obtener pose RELATIVA entre dos entidades arbitrarias.
    """

    # -----------------------------
    #  Helpers internos
    # -----------------------------
    @staticmethod
    def _identity_matrix() -> adsk.core.Matrix3D:
        """Devuelve Matrix3D identidad."""
        try:
            m = adsk.core.Matrix3D.create()
            if _DEBUG_TRANSFORMS:
                _debug_matrix("identity_matrix", m)
            return m
        except Exception as e:
            _log(f"Error creando matriz identidad: {e}", level="ERROR")
            return None

    # -----------------------------
    #  Conversión Matrix3D -> xyz/rpy
    # -----------------------------
    @staticmethod
    def matrix_to_xyz_rpy(m: adsk.core.Matrix3D,
                          design: adsk.fusion.Design
                          ) -> Tuple[Tuple[float, float, float],
                                     Tuple[float, float, float]]:
        """Wrapper directo al sistema Código 1."""
        return _matrix_to_xyz_rpy(m, design)

    # -----------------------------
    #  Matriz global de una entidad
    # -----------------------------
    @staticmethod
    def global_matrix_for_entity(entity: Any) -> adsk.core.Matrix3D:
        """
        Devuelve una Matrix3D que representa la pose GLOBAL de la entidad.

        Soporta:
          - adsk.fusion.Occurrence (usa transform2 directamente)
          - BRepBody, MeshBody, etc. con assemblyContext
          - Cualquier objeto con .transform y/o .assemblyContext
        """
        try:
            if _DEBUG_TRANSFORMS:
                _log(f"[TFM] global_matrix_for_entity: entity={_describe_entity(entity)}", level="DEBUG")

            if entity is None:
                m_id = TransformUtils._identity_matrix()
                if _DEBUG_TRANSFORMS:
                    _log("[TFM] global_matrix_for_entity: entity=None → identidad", level="DEBUG")
                return m_id

            # Caso 1: Occurrence o entidad con transform2 (pose global directa)
            try:
                if hasattr(entity, "transform2"):
                    m = getattr(entity, "transform2", None)
                    if isinstance(m, adsk.core.Matrix3D):
                        m_copy = m.copy()
                        if _DEBUG_TRANSFORMS:
                            _log("[TFM] global_matrix_for_entity: usando transform2 directo", level="DEBUG")
                            _debug_matrix("global_matrix_for_entity.transform2", m_copy)
                        return m_copy
            except Exception as e:
                if _DEBUG_TRANSFORMS:
                    _log(f"[TFM] global_matrix_for_entity: error leyendo transform2: {e}", level="DEBUG")

            # Caso 2: Cualquier entidad con assemblyContext (pose global = parent_global * local)
            parent_m = None
            try:
                ctx = getattr(entity, "assemblyContext", None)
            except:
                ctx = None

            if ctx is not None and ctx is not entity:
                if _DEBUG_TRANSFORMS:
                    _log(f"[TFM] global_matrix_for_entity: entity tiene assemblyContext={_describe_entity(ctx)}",
                         level="DEBUG")
                parent_m = TransformUtils.global_matrix_for_entity(ctx)

            # Matriz local (si tiene .transform)
            local_m = None
            try:
                if hasattr(entity, "transform"):
                    t = getattr(entity, "transform", None)
                    if isinstance(t, adsk.core.Matrix3D):
                        local_m = t.copy()
                        if _DEBUG_TRANSFORMS:
                            _debug_matrix("global_matrix_for_entity.local_transform", local_m)
            except Exception as e:
                if _DEBUG_TRANSFORMS:
                    _log(f"[TFM] global_matrix_for_entity: error leyendo transform: {e}", level="DEBUG")
                local_m = None

            # Composición
            if parent_m is None:
                if local_m is not None:
                    if _DEBUG_TRANSFORMS:
                        _log("[TFM] global_matrix_for_entity: sin parent, usando solo transform local", level="DEBUG")
                    return local_m
                else:
                    if _DEBUG_TRANSFORMS:
                        _log("[TFM] global_matrix_for_entity: sin parent y sin local → identidad", level="DEBUG")
                    m_id = TransformUtils._identity_matrix()
                    return m_id
            else:
                m_total = parent_m.copy()
                if local_m is not None:
                    if _DEBUG_TRANSFORMS:
                        _log("[TFM] global_matrix_for_entity: componiendo parent * local", level="DEBUG")
                        _debug_matrix("global_matrix_for_entity.parent_before", parent_m)
                        _debug_matrix("global_matrix_for_entity.local_before", local_m)
                    m_total.transformBy(local_m)
                else:
                    if _DEBUG_TRANSFORMS:
                        _log("[TFM] global_matrix_for_entity: solo parent (sin local)", level="DEBUG")

                if _DEBUG_TRANSFORMS:
                    _debug_matrix("global_matrix_for_entity.result", m_total)
                return m_total

        except Exception as e:
            _log(f"Error en global_matrix_for_entity: {e}", level="ERROR")
            m_id = TransformUtils._identity_matrix()
            return m_id

    # -----------------------------
    #  Pose absoluta de una entidad
    # -----------------------------
    @staticmethod
    def entity_abs_pose(entity: Any,
                        design: adsk.fusion.Design
                        ) -> Tuple[Tuple[float, float, float],
                                   Tuple[float, float, float]]:
        """
        Pose ABSOLUTA (xyz_m, rpy) de cualquier entidad de Fusion.
        """
        try:
            m = TransformUtils.global_matrix_for_entity(entity)
            xyz, rpy = _matrix_to_xyz_rpy(m, design)

            if _DEBUG_TRANSFORMS:
                _log(f"[TFM] entity_abs_pose: entity={_describe_entity(entity)} → xyz(m)={xyz}, rpy(rad)={rpy}",
                     level="DEBUG")
            return xyz, rpy
        except Exception as e:
            _log(f"Error en entity_abs_pose: {e}", level="ERROR")
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    # -----------------------------
    #  Pose absoluta de una occurrence
    # -----------------------------
    @staticmethod
    def occ_abs_pose(occ: adsk.fusion.Occurrence,
                     design: adsk.fusion.Design
                     ) -> Tuple[Tuple[float, float, float],
                                Tuple[float, float, float]]:
        """
        SISTEMA EXACTO Código 1, pero apoyado en el sistema GENERAL.
        """
        if _DEBUG_TRANSFORMS:
            _log(f"[TFM] occ_abs_pose: occ={_describe_entity(occ)}", level="DEBUG")
        return TransformUtils.entity_abs_pose(occ, design)

    # -----------------------------
    #  Pose relativa parent → child (ENTIDADES GENÉRICAS)
    # -----------------------------
    @staticmethod
    def relative_pose(parent_entity: Optional[Any],
                      child_entity: Any,
                      design: adsk.fusion.Design
                      ) -> Tuple[Tuple[float, float, float],
                                 Tuple[float, float, float]]:
        """
        Calcula la pose RELATIVA del child respecto al parent usando matrices completas.

            T_world^P = global_matrix_for_entity(parent_entity)
            T_world^C = global_matrix_for_entity(child_entity)
            T_P^C     = (T_world^P)^-1 * T_world^C
        """
        try:
            if _DEBUG_TRANSFORMS:
                _log(
                    "[TFM] relative_pose: parent={}; child={}".format(
                        _describe_entity(parent_entity),
                        _describe_entity(child_entity)
                    ),
                    level="DEBUG"
                )

            m_child = TransformUtils.global_matrix_for_entity(child_entity)

            if parent_entity is None:
                if _DEBUG_TRANSFORMS:
                    _log("[TFM] relative_pose: parent=None → usamos pose absoluta del child", level="DEBUG")
                xyz, rpy = _matrix_to_xyz_rpy(m_child, design)
                if _DEBUG_TRANSFORMS:
                    _log(f"[TFM] relative_pose (parent=None) → xyz(m)={xyz}, rpy(rad)={rpy}", level="DEBUG")
                return xyz, rpy

            m_parent = TransformUtils.global_matrix_for_entity(parent_entity)

            if _DEBUG_TRANSFORMS:
                _debug_matrix("relative_pose.parent_global", m_parent)
                _debug_matrix("relative_pose.child_global", m_child)

            m_rel = m_parent.copy()
            m_rel.invert()
            m_rel.transformBy(m_child)

            if _DEBUG_TRANSFORMS:
                _debug_matrix("relative_pose.result", m_rel)

            xyz, rpy = _matrix_to_xyz_rpy(m_rel, design)
            if _DEBUG_TRANSFORMS:
                _log(f"[TFM] relative_pose result → xyz(m)={xyz}, rpy(rad)={rpy}", level="DEBUG")

            return xyz, rpy

        except Exception as e:
            _log(f"Error en relative_pose: {e}", level="ERROR")
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    # -----------------------------
    #  Body nativo (no proxy) para BRep/Mesh
    # -----------------------------
    @staticmethod
    def native_body(body: Any) -> Any:
        """
        Devuelve el body nativo (no proxy) cuando existe .nativeObject.
        Si no aplica o falla, devuelve el mismo objeto.

        Se usa para:
          - Geometría / malla (MeshUtils, ColorUtils, etc.).
        """
        if body is None:
            return None
        try:
            if hasattr(body, "nativeObject"):
                native = getattr(body, "nativeObject", None)
                if native:
                    if _DEBUG_TRANSFORMS:
                        _log(
                            f"[TFM] native_body: proxy={_describe_entity(body)} "
                            f"→ native={_describe_entity(native)}",
                            level="DEBUG"
                        )
                    return native
        except Exception as e:
            if _DEBUG_TRANSFORMS:
                _log(f"[TFM] native_body: error al obtener nativeObject: {e}", level="DEBUG")
        if _DEBUG_TRANSFORMS:
            _log(f"[TFM] native_body: body ya es nativo: {_describe_entity(body)}", level="DEBUG")
        return body

    # -----------------------------
    #  Mapeo de joints de Fusion -> tipo URDF + eje + límites
    # -----------------------------
    @staticmethod
    def map_fusion_joint(joint):
        """SISTEMA EXACTO Código 1 + DEBUG."""
        try:
            if not joint:
                return "fixed", (0.0, 0.0, 1.0), None

            if _DEBUG_TRANSFORMS:
                jname = getattr(joint, "name", "<sin_nombre>")
                _log(f"[JNT] map_fusion_joint: joint='{jname}'", level="DEBUG")

            motion = getattr(joint, "jointMotion", None)
            if not motion:
                if _DEBUG_TRANSFORMS:
                    _log("[JNT] jointMotion es None → fixed", level="DEBUG")
                return "fixed", (0.0, 0.0, 1.0), None

            jtype = "fixed"
            axis = (0.0, 0.0, 1.0)
            limit = None

            try:
                from adsk.fusion import JointTypes
                jt = getattr(motion, "jointType", None)

                if JointTypes and jt is not None:
                    try:
                        if jt == JointTypes.RigidJointType:
                            jtype = "fixed"
                        elif jt == JointTypes.RevoluteJointType:
                            jtype = "revolute"
                        elif jt == JointTypes.SliderJointType:
                            jtype = "prismatic"
                    except:
                        pass
            except:
                pass

            try:
                if motion and hasattr(motion, "rotationAxisVector"):
                    av = motion.rotationAxisVector
                    axis = (av.x, av.y, av.z)
                elif hasattr(joint, "geometry") and joint.geometry and hasattr(joint.geometry, "primaryAxisVector"):
                    av = joint.geometry.primaryAxisVector
                    axis = (av.x, av.y, av.z)
            except:
                pass

            # Normalizar eje
            length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            if length > 1e-6:
                axis = (axis[0] / length, axis[1] / length, axis[2] / length)

            # Límites para revolute
            if jtype == "revolute":
                rl = getattr(motion, "rotationLimits", None)
                if rl:
                    lo = hi = None
                    if getattr(rl, "isMinimumValueEnabled", False):
                        lo = rl.minimumValue
                    if getattr(rl, "isMaximumValueEnabled", False):
                        hi = rl.maximumValue
                    if lo is not None and hi is not None and hi > lo:
                        limit = (lo, hi)
                if limit is None:
                    jtype = "continuous"

            # Límites para prismatic
            elif jtype == "prismatic":
                sl = getattr(motion, "slideLimits", None)
                if sl:
                    lo = hi = None
                    if getattr(sl, "isMinimumValueEnabled", False):
                        lo = sl.minimumValue
                    if getattr(sl, "isMaximumValueEnabled", False):
                        hi = sl.maximumValue
                    if lo is not None and hi is not None and hi > lo:
                        limit = (lo, hi)
                if limit is None:
                    limit = (-0.1, 0.1)

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[JNT] map_fusion_joint result: type={jtype}, axis={axis}, limit={limit}",
                    level="DEBUG"
                )

            return jtype, axis, limit

        except Exception as e:
            _log(f"Error en map_fusion_joint: {e}", level="ERROR")
            return "fixed", (0.0, 0.0, 1.0), None

    # -----------------------------
    #  Origen de joint (xyz/rpy global)
    # -----------------------------
    @staticmethod
    def joint_origin(joint, design: adsk.fusion.Design
                     ) -> Tuple[Tuple[float, float, float],
                                Tuple[float, float, float]]:
        """SISTEMA EXACTO Código 1 + DEBUG."""
        try:
            if not joint:
                return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

            geo = getattr(joint, "geometryOrOriginTwo", None)
            if geo and hasattr(geo, "transform"):
                m = geo.transform
                if m:
                    if _DEBUG_TRANSFORMS:
                        jname = getattr(joint, "name", "<sin_nombre>")
                        _log(f"[JNT] joint_origin: usando geometryOrOriginTwo de '{jname}'", level="DEBUG")
                        _debug_matrix("joint_origin.matrix", m)
                    return _matrix_to_xyz_rpy(m, design)

        except Exception as e:
            _log(f"Error en joint_origin: {e}", level="ERROR")

        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    # -----------------------------
    #  Heurística parent/child para joints
    # -----------------------------
    @staticmethod
    def pick_parent_child(name1: str, name2: str,
                          existing_joints: List[Dict]) -> Tuple[str, str]:
        """SISTEMA EXACTO Código 1 + DEBUG."""
        try:
            if not name1 or not name2:
                return name1 or "unknown", name2 or "unknown"

            children = {j["child"] for j in existing_joints if "child" in j}

            if name1 in children and name2 not in children:
                parent, child = name2, name1
            elif name2 in children and name1 not in children:
                parent, child = name1, name2
            else:
                parent, child = ((name1, name2) if name1 < name2 else (name2, name1))

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[JNT] pick_parent_child: candidates=({name1}, {name2}) → "
                    f"parent='{parent}', child='{child}'",
                    level="DEBUG"
                )

            return parent, child

        except Exception as e:
            _log(f"Error en pick_parent_child: {e}", level="ERROR")
            return name1, name2






















# =========================================================
#  BLOQUE 2 / 8 - COLORES, TEXTURAS Y COLECTA DE BODIES
#                  (SISTEMA CÓDIGO 1 + DEBUG ÁRBOL)
# =========================================================

class ColorUtils:
    """SISTEMA EXACTO de colores del Código 1 (con hooks de debug ligeros)."""

    @staticmethod
    def _get_color_from_appearance(app):
        """SISTEMA EXACTO Código 1."""
        if not app:
            return None

        try:
            props = app.appearanceProperties
        except:
            return None

        # 1) propiedad "Color" explícita
        try:
            p = props.itemByName("Color")
            if p:
                colProp = adsk.core.ColorProperty.cast(p)
                if colProp and colProp.value:
                    c = colProp.value
                    r = c.red / 255.0
                    g = c.green / 255.0
                    b = c.blue / 255.0
                    a_raw = getattr(c, "opacity", 255)
                    a = a_raw / 255.0
                    return (r, g, b, a)
        except:
            pass

        # 2) cualquier ColorProperty
        try:
            for i in range(props.count):
                prop = props.item(i)
                colProp = adsk.core.ColorProperty.cast(prop)
                if colProp and colProp.value:
                    c = colProp.value
                    r = c.red / 255.0
                    g = c.green / 255.0
                    b = c.blue / 255.0
                    a_raw = getattr(c, "opacity", 255)
                    a = a_raw / 255.0
                    return (r, g, b, a)
        except:
            pass

        return None

    @staticmethod
    def extract_color_from_body(body):
        """SISTEMA EXACTO Código 1."""
        if not body:
            return None

        body = TransformUtils.native_body(body)

        # 1) body.appearance
        try:
            app = getattr(body, "appearance", None)
            if app:
                col = ColorUtils._get_color_from_appearance(app)
                if col:
                    return col
        except:
            pass

        # 2) material.appearance
        try:
            mat = getattr(body, "material", None)
            if mat and getattr(mat, "appearance", None):
                app = mat.appearance
                col = ColorUtils._get_color_from_appearance(app)
                if col:
                    return col
        except:
            pass

        return None

    @staticmethod
    def extract_color_for_link(body_or_mesh, occ):
        """SISTEMA EXACTO Código 1."""
        # 1) directo desde body/mesh
        col = ColorUtils.extract_color_from_body(body_or_mesh)
        if col:
            return col

        # 2) appearance directo de la occurrence
        try:
            if occ and getattr(occ, "appearance", None):
                col = ColorUtils._get_color_from_appearance(occ.appearance)
                if col:
                    return col
        except:
            pass

        # 3) appearance del componente
        try:
            if occ and getattr(occ, "component", None) and occ.component.appearance:
                col = ColorUtils._get_color_from_appearance(occ.component.appearance)
                if col:
                    return col
        except:
            pass

        # 4) material del componente
        try:
            if occ and getattr(occ, "component", None):
                mat = getattr(occ.component, "material", None)
                if mat and getattr(mat, "appearance", None):
                    col = ColorUtils._get_color_from_appearance(mat.appearance)
                    if col:
                        return col
        except:
            pass

        return None

    @staticmethod
    def extract_color_for_face(face, body, occ):
        """SISTEMA EXACTO Código 1."""
        if face:
            try:
                app = getattr(face, "appearance", None)
                if app:
                    col = ColorUtils._get_color_from_appearance(app)
                    if col:
                        return col
            except:
                pass

        col = ColorUtils.extract_color_for_link(body, occ)
        if col:
            return col

        return (0.7, 0.7, 0.7, 1.0)


class TextureUtils:
    """SISTEMA EXACTO de texturas del Código 1."""

    @staticmethod
    def _png_write_rgba(filepath: str, width: int, height: int, pixels_rgba: bytes):
        """SISTEMA EXACTO Código 1."""
        try:
            raw = bytearray()
            stride = width * 4
            for y in range(height):
                raw.append(0)  # filter type 0
                start = y * stride
                raw.extend(pixels_rgba[start:start + stride])

            compressed = zlib.compress(bytes(raw), 9)

            def chunk(chunk_type: str, data: bytes) -> bytes:
                length = struct.pack(">I", len(data))
                ctype = chunk_type.encode("ascii")
                crc = zlib.crc32(ctype + data) & 0xffffffff
                crc_bytes = struct.pack(">I", crc)
                return length + ctype + data + crc_bytes

            ihdr_data = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
            png = b"\x89PNG\r\n\x1a\n"
            png += chunk("IHDR", ihdr_data)
            png += chunk("IDAT", compressed)
            png += chunk("IEND", b"")

            with open(filepath, "wb") as f:
                f.write(png)
        except Exception as e:
            _log(f"Error al escribir PNG {filepath}: {e}", level="ERROR")

    @staticmethod
    def build_faces_atlas_png(png_path: str, face_colors: list, cell_px: int = 128):
        """SISTEMA EXACTO Código 1."""
        n = len(face_colors)
        if n <= 0:
            return None, None, []

        cols = max(1, int(math.ceil(math.sqrt(n))))
        rows = int(math.ceil(n / cols))

        width = cols * cell_px
        height = rows * cell_px

        pixels = bytearray(width * height * 4)

        # Pintamos cada celda
        for idx, col in enumerate(face_colors):
            if col is None:
                r = g = b = 0.7
                a = 1.0
            else:
                r, g, b, a = col

            ir = max(0, min(255, int(round(r * 255))))
            ig = max(0, min(255, int(round(g * 255))))
            ib = max(0, min(255, int(round(b * 255))))
            ia = max(0, min(255, int(round(a * 255))))

            c = idx % cols
            r_row = idx // cols

            x0 = c * cell_px
            y0 = r_row * cell_px

            for yy in range(y0, min(y0 + cell_px, height)):
                for xx in range(x0, min(x0 + cell_px, width)):
                    off = (yy * width + xx) * 4
                    pixels[off:off + 4] = bytes((ir, ig, ib, ia))

        TextureUtils._png_write_rgba(png_path, width, height, bytes(pixels))

        # UV centers con V invertida
        uv_centers = []
        for idx in range(n):
            c = idx % cols
            r_row = idx // cols

            # U tal cual
            u0 = c / cols
            u1 = (c + 1) / cols
            u = 0.5 * (u0 + u1)

            # Flip vertical
            v0_img = r_row / rows
            v1_img = (r_row + 1) / rows
            v_img_center = 0.5 * (v0_img + v1_img)
            v = 1.0 - v_img_center

            uv_centers.append((u, v))

        return width, height, uv_centers

    @staticmethod
    def build_solid_png(png_path: str, color: tuple, size: int = 4):
        """SISTEMA EXACTO Código 1."""
        if color is None:
            r = g = b = 0.7
            a = 1.0
        else:
            r, g, b, a = color

        ir = max(0, min(255, int(round(r * 255))))
        ig = max(0, min(255, int(round(g * 255))))
        ib = max(0, min(255, int(round(b * 255))))
        ia = max(0, min(255, int(round(a * 255))))

        pixels = bytes((ir, ig, ib, ia)) * (size * size)
        TextureUtils._png_write_rgba(png_path, size, size, pixels)


# =========================================================
#  COLECTA DE BODIES (BRep y Mesh) POR OCCURRENCE
# =========================================================

class BodyCollector:
    """SISTEMA EXACTO de colecta de bodies del Código 1 + DEBUG ÁRBOL."""

    @staticmethod
    def _debug_list_entities(label: str, entities: List[Any]):
        """Pequeño helper para listar entities con su contexto y tipo."""
        if not _DEBUG_MESH_TREE:
            return
        try:
            descs = []
            for e in entities:
                descs.append(_describe_entity(e))
            _log(f"[TREE] {label} (count={len(entities)}):", level="DEBUG")
            for d in descs:
                _log(f"[TREE]    - {d}", level="DEBUG")
        except Exception as e:
            _log(f"[TREE] Error en _debug_list_entities({label}): {e}", level="DEBUG")

    @staticmethod
    def collect_bodies_for_occurrence(occ: adsk.fusion.Occurrence):
        """
        Devuelve:
          comp_brep_export, occ_brep_export, comp_mesh_export, occ_mesh_export

        - Toma TODOS los BRepBodies y MeshBodies convertibles (no filtramos por visibilidad
          para respetar tu requisito de 'todos los elementos convertibles').
        - Prioriza bodies en la occurrence; si no hay, usa los del componente.

        DEBUG:
        - Imprime de dónde viene cada body/mesh:
            * occ.bRepBodies vs comp.bRepBodies
            * occ.meshBodies vs comp.meshBodies
        - Marca si son proxies (assemblyContext) y a qué occurrence están ligados.
        """
        if not occ:
            return [], [], [], []

        comp = occ.component

        if _DEBUG_MESH_TREE:
            key = get_occurrence_key(occ)
            _log(
                f"[TREE] collect_bodies_for_occurrence: OCC={_describe_entity(occ)} "
                f"(key='{key}')",
                level="DEBUG"
            )

        # --- BREP ---
        comp_brep_all = list(comp.bRepBodies)
        occ_brep_all = list(occ.bRepBodies)

        if _DEBUG_MESH_TREE:
            BodyCollector._debug_list_entities("comp.bRepBodies (todos)", comp_brep_all)
            BodyCollector._debug_list_entities("occ.bRepBodies (todos)", occ_brep_all)

        comp_brep_export = []
        occ_brep_export = []

        # Primero bodies de la occurrence
        for b in occ_brep_all:
            try:
                _ = getattr(b, "isSolid", None)
                _ = getattr(b, "isVisible", None)
            except:
                pass

            # Aquí podrías filtrar por algún criterio si quisieras
            export = True
            if export:
                occ_brep_export.append(b)
                if _DEBUG_MESH_TREE:
                    _log(
                        f"[TREE]   → BRep en OCC seleccionado para export: {_describe_entity(b)}",
                        level="DEBUG"
                    )

        # Si no hay en occurrence, usamos los del componente
        if not occ_brep_export:
            for b in comp_brep_all:
                try:
                    _ = getattr(b, "isSolid", None)
                    _ = getattr(b, "isVisible", None)
                except:
                    pass

                export = True
                if export:
                    comp_brep_export.append(b)
                    if _DEBUG_MESH_TREE:
                        _log(
                            f"[TREE]   → BRep en COMPONENT seleccionado para export: {_describe_entity(b)}",
                            level="DEBUG"
                        )

        # --- MESH BODIES ---
        comp_mesh_all = list(getattr(comp, "meshBodies", []))
        occ_mesh_all = list(getattr(occ, "meshBodies", []))

        if _DEBUG_MESH_TREE:
            BodyCollector._debug_list_entities("comp.meshBodies (todos)", comp_mesh_all)
            BodyCollector._debug_list_entities("occ.meshBodies (todos)", occ_mesh_all)

        comp_mesh_export = []
        occ_mesh_export = []

        # Primero meshBodies de la occurrence
        for m in occ_mesh_all:
            try:
                _ = getattr(m, "isVisible", None)
            except:
                pass
            export = True
            if export:
                occ_mesh_export.append(m)
                if _DEBUG_MESH_TREE:
                    _log(
                        f"[TREE]   → MeshBody en OCC seleccionado para export: {_describe_entity(m)}",
                        level="DEBUG"
                    )

        # Si no hay mesh en occurrence, usamos los del componente
        if not occ_mesh_export:
            for m in comp_mesh_all:
                try:
                    _ = getattr(m, "isVisible", None)
                except:
                    pass
                export = True
                if export:
                    comp_mesh_export.append(m)
                    if _DEBUG_MESH_TREE:
                        _log(
                            f"[TREE]   → MeshBody en COMPONENT seleccionado para export: {_describe_entity(m)}",
                            level="DEBUG"
                        )

        if _DEBUG_MESH_TREE:
            _log(
                "[TREE] RESUMEN collect_bodies_for_occurrence:"
                f" occ_brep_export={len(occ_brep_export)},"
                f" comp_brep_export={len(comp_brep_export)},"
                f" occ_mesh_export={len(occ_mesh_export)},"
                f" comp_mesh_export={len(comp_mesh_export)}",
                level="DEBUG"
            )

            # Extra: ver si algo sospechoso está pasando (todo viene solo de comp.*, por ejemplo)
            if not occ_brep_export and occ_brep_all:
                _log(
                    "[TREE][WARN] La occurrence tenía BRepBodies pero ninguno se puso en occ_brep_export "
                    "(revisa criterio de export).",
                    level="WARNING"
                )
            if not occ_mesh_export and occ_mesh_all:
                _log(
                    "[TREE][WARN] La occurrence tenía MeshBodies pero ninguno se puso en occ_mesh_export "
                    "(revisa criterio de export).",
                    level="WARNING"
                )

        return comp_brep_export, occ_brep_export, comp_mesh_export, occ_mesh_export

































# =========================================================
#  BLOQUE 3 / 8 - FÍSICA (SISTEMA CÓDIGO 1 + DEBUG)
# =========================================================

class PhysicsUtils:
    """Utilidades para cálculo de propiedades físicas."""

    @staticmethod
    def get_body_physical_properties(body):
        """
        Devuelve propiedades físicas de un body.

        Retorna: (mass_kg, com_global_m, inertia_global_kgm2)

        Nota:
        - Usa el sistema CÓDIGO 1 (mismas fórmulas que el sistema que
          ya te funcionaba bien).
        - COM e inercia se calculan en el sistema global del diseño
          (Fusion los da en cm, aquí se convierten a m).

        DEBUG:
        - Imprime si el body es proxy o nativo.
        - Imprime mass, COM e inercia calculada, para cazar cosas como:
            * mass = 0 en bodies que deberían pesar algo
            * COM muy lejos del origen esperado
            * inercia basura (todo en 0, etc.)
        """
        empty_result = (0.0,
                        (0.0, 0.0, 0.0),
                        (1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0))

        if not body:
            if _DEBUG_TRANSFORMS:
                _log("[PHYS] get_body_physical_properties: body=None → empty_result", level="DEBUG")
            return empty_result

        # Debug de body original vs nativo
        if _DEBUG_TRANSFORMS:
            _log(f"[PHYS] get_body_physical_properties: body(IN)={_describe_entity(body)}", level="DEBUG")

        # OJO: las props físicas deben pedirse sobre el body nativo
        native = TransformUtils.native_body(body)
        if _DEBUG_TRANSFORMS and native is not body:
            _log(
                f"[PHYS] get_body_physical_properties: body es proxy, usando native={_describe_entity(native)}",
                level="DEBUG"
            )
        body = native

        try:
            props = getattr(body, "physicalProperties", None)
            if not props:
                if _DEBUG_TRANSFORMS:
                    _log(
                        "[PHYS] physicalProperties es None → empty_result "
                        "(suele pasar en MeshBodies sin material o cuerpos de construcción).",
                        level="DEBUG"
                    )
                return empty_result

            # Masa (kg)
            mass = getattr(props, "mass", 0.0)

            # Centro de masa (cm → m)
            com = getattr(props, "centerOfMass", None)
            if not com:
                if _DEBUG_TRANSFORMS:
                    _log(
                        "[PHYS] centerOfMass es None → empty_result "
                        "(cuerpo sin props bien definidas).",
                        level="DEBUG"
                    )
                return empty_result

            cx = com.x * 0.01
            cy = com.y * 0.01
            cz = com.z * 0.01
            com_global = (cx, cy, cz)

            # Momentos principales (kg·cm² → kg·m²)
            Ixx = Iyy = Izz = 0.0
            try:
                if hasattr(props, "getXYZMomentsOfInertia"):
                    Ixx, Iyy, Izz = props.getXYZMomentsOfInertia()
            except Exception as e:
                if _DEBUG_TRANSFORMS:
                    _log(f"[PHYS] getXYZMomentsOfInertia lanzó excepción: {e}", level="DEBUG")

            # Productos de inercia (kg·cm² → kg·m²)
            Ixy = Ixz = Iyz = 0.0
            try:
                if hasattr(props, "getProductsOfInertia"):
                    Ixy, Ixz, Iyz = props.getProductsOfInertia()
            except Exception as e:
                if _DEBUG_TRANSFORMS:
                    _log(f"[PHYS] getProductsOfInertia lanzó excepción: {e}", level="DEBUG")

            # Conversión kg·cm² → kg·m² (1 cm = 0.01 m → 10^-4)
            scale = 1e-4
            Ixx *= scale
            Iyy *= scale
            Izz *= scale
            Ixy *= scale
            Ixz *= scale
            Iyz *= scale

            inertia_global = (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)

            if _DEBUG_TRANSFORMS:
                _log(
                    "[PHYS] body={desc} → mass={mass:.6g} kg, "
                    "COM_global(m)={com}, "
                    "Inertia_global(kg·m²)=(Ixx={ixx:.3e}, Iyy={iyy:.3e}, Izz={izz:.3e}, "
                    "Ixy={ixy:.3e}, Ixz={ixz:.3e}, Iyz={iyz:.3e})".format(
                        desc=_describe_entity(body),
                        mass=mass,
                        com=com_global,
                        ixx=Ixx, iyy=Iyy, izz=Izz,
                        ixy=Ixy, ixz=Ixz, iyz=Iyz
                    ),
                    level="DEBUG"
                )

            # Si la masa sale 0, dejar el registro explícito (suele explicar links “fantasma”)
            if _DEBUG_TRANSFORMS and mass <= 0:
                _log(
                    "[PHYS][WARN] mass=0 para body={desc} → este link aportará masa nula al URDF".format(
                        desc=_describe_entity(body)
                    ),
                    level="WARNING"
                )

            return mass, com_global, inertia_global

        except Exception as e:
            _log(f"Error en get_body_physical_properties: {e}", level="ERROR")
            return empty_result

    @staticmethod
    def combine_bodies_physics(phys_list):
        """
        Combina propiedades físicas de múltiples bodies (regla de Steiner).

        phys_list: lista de tuplas:
            (mass_kg, com_global_m, inertia_global_kgm2)

        Retorna:
            (mass_total, com_total_global, inertia_total_global)

        DEBUG:
        - Imprime listado de cada cuerpo antes de combinar.
        - Muestra masa total, COM resultante e inercia resultante.
        """
        empty_result = (0.0,
                        (0.0, 0.0, 0.0),
                        (1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0))

        if not phys_list:
            if _DEBUG_TRANSFORMS:
                _log("[PHYS] combine_bodies_physics: phys_list vacío → empty_result", level="DEBUG")
            return empty_result

        try:
            if _DEBUG_TRANSFORMS:
                _log(f"[PHYS] combine_bodies_physics: n_bodies={len(phys_list)}", level="DEBUG")
                for i, (mass_i, com_i, inertia_i) in enumerate(phys_list):
                    _log(
                        f"[PHYS]   body[{i}]: mass={mass_i:.6g}, COM(m)={com_i}, "
                        f"Inertia={inertia_i}",
                        level="DEBUG"
                    )

            # 1) Masa total
            total_mass = 0.0
            for mass, com, inertia in phys_list:
                if mass > 0:
                    total_mass += mass

            if total_mass <= 0:
                if _DEBUG_TRANSFORMS:
                    _log(
                        "[PHYS][WARN] combine_bodies_physics: masa total <= 0 → empty_result "
                        "(todos los cuerpos tenían masa 0?)",
                        level="WARNING"
                    )
                return empty_result

            # 2) COM global total
            cx_sum, cy_sum, cz_sum = 0.0, 0.0, 0.0
            for mass, com, inertia in phys_list:
                if mass > 0:
                    cx_sum += mass * com[0]
                    cy_sum += mass * com[1]
                    cz_sum += mass * com[2]

            cx = cx_sum / total_mass
            cy = cy_sum / total_mass
            cz = cz_sum / total_mass
            com_total = (cx, cy, cz)

            # 3) Inercia total trasladada al COM total (paralell axis)
            Ixx = Iyy = Izz = 0.0
            Ixy = Ixz = Iyz = 0.0

            for mass, com, inertia in phys_list:
                if mass <= 0:
                    continue

                px, py, pz = com
                dx = px - cx
                dy = py - cy
                dz = pz - cz

                Ixx_b, Iyy_b, Izz_b, Ixy_b, Ixz_b, Iyz_b = inertia

                # Momentos trasladados
                Ixx += Ixx_b + mass * (dy * dy + dz * dz)
                Iyy += Iyy_b + mass * (dx * dx + dz * dz)
                Izz += Izz_b + mass * (dx * dx + dy * dy)

                # Productos trasladados
                Ixy += Ixy_b - mass * dx * dy
                Ixz += Ixz_b - mass * dx * dz
                Iyz += Iyz_b - mass * dy * dz

            inertia_total = (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)

            if _DEBUG_TRANSFORMS:
                _log(
                    "[PHYS] combine_bodies_physics RESULT → "
                    "mass_total={mt:.6g} kg, COM_total(m)={com}, "
                    "Inertia_total(kg·m²)=(Ixx={ixx:.3e}, Iyy={iyy:.3e}, Izz={izz:.3e}, "
                    "Ixy={ixy:.3e}, Ixz={ixz:.3e}, Iyz={iyz:.3e})".format(
                        mt=total_mass,
                        com=com_total,
                        ixx=Ixx, iyy=Iyy, izz=Izz,
                        ixy=Ixy, ixz=Ixz, iyz=Iyz
                    ),
                    level="DEBUG"
                )

            return total_mass, com_total, inertia_total

        except Exception as e:
            _log(f"Error en combine_bodies_physics: {e}", level="ERROR")
            return empty_result




















# =========================================================
#  BLOQUE 4 / 8 - MANEJO DE MALLAS (VERY LOW + DISPLAYMESH)
#                  (CON DEBUG DE ÁRBOL Y MALLA)
# =========================================================

class MeshUtils:
    """Utilidades para extracción de mallas (BRep + MeshBody)."""

    # -----------------------------------------------------
    #  MALLA BÁSICA POR CUERPO (BRep o MeshBody)
    # -----------------------------------------------------
    @staticmethod
    def _get_mesh_triangles_from_body_basic(
        body,
        design: adsk.fusion.Design,
        mesh_quality_mode: str = "very_low_optimized"
    ):
        """
        Devuelve (vertices_m, indices) para un body BRep o MeshBody.

        - BRep: usa meshManager + TriangleMeshCalculator o displayMeshes.
        - MeshBody: usa displayMesh/mesh.
        - Siempre devuelve vértices en METROS.

        DEBUG:
        - Imprime si el body es BRep o MeshBody.
        - Muestra si estamos usando displayMesh o TriangleMeshCalculator.
        - Enseña el diámetro estimado y la tolerancia usada.
        - Cuenta final de vértices e índices.
        """
        if _DEBUG_MESH_TREE:
            _log(
                f"[MESH] _get_mesh_triangles_from_body_basic: body={_describe_entity(body)}, "
                f"mode='{mesh_quality_mode}'",
                level="DEBUG"
            )

        # Siempre trabajar con el body nativo (sin assemblyContext)
        original = body
        body = TransformUtils.native_body(body)
        if _DEBUG_MESH_TREE and original is not body:
            _log(
                f"[MESH]   body era proxy → usando nativo={_describe_entity(body)}",
                level="DEBUG"
            )

        is_brep = hasattr(body, "meshManager")
        is_mesh_body = hasattr(body, "mesh") or hasattr(body, "displayMesh")

        coords = []
        indices = []

        try:
            # ------------------------
            #  BREP BODY
            # ------------------------
            if is_brep:
                if _DEBUG_MESH_TREE:
                    _log("[MESH]   detectado BRepBody (usa meshManager)", level="DEBUG")

                mesh_mgr = body.meshManager
                mesh = None

                # Si estamos en modo display, intentamos usar displayMeshes primero
                if mesh_quality_mode == "display_mesh":
                    try:
                        dm = getattr(mesh_mgr, "displayMeshes", None)
                        if dm and dm.count > 0:
                            mesh = dm.item(0)
                            if _DEBUG_MESH_TREE:
                                _log(
                                    "[MESH]   usando displayMeshes[0] para BRep (DisplayMesh directo)",
                                    level="DEBUG"
                                )
                    except Exception as e:
                        mesh = None
                        if _DEBUG_MESH_TREE:
                            _log(
                                f"[MESH]   error usando displayMeshes en BRep: {e} → fallback TriangleMeshCalculator",
                                level="DEBUG"
                            )

                # Si no hay displayMesh o estamos en very_low, usamos TriangleMeshCalculator
                if mesh is None:
                    try:
                        calc = mesh_mgr.createMeshCalculator()

                        # Calcular diámetro aproximado para ajustar tolerancia
                        try:
                            bbox = body.boundingBox
                            dx = bbox.maxPoint.x - bbox.minPoint.x
                            dy = bbox.maxPoint.y - bbox.minPoint.y
                            dz = bbox.maxPoint.z - bbox.maxPoint.z + (bbox.maxPoint.z - bbox.minPoint.z)
                        except:
                            bbox = None
                            dx = dy = dz = 1.0

                        if bbox is not None:
                            dx = bbox.maxPoint.x - bbox.minPoint.x
                            dy = bbox.maxPoint.y - bbox.minPoint.y
                            dz = bbox.maxPoint.z - bbox.minPoint.z
                            diameter = math.sqrt(dx * dx + dy * dy + dz * dz)
                        else:
                            diameter = 1.0

                        if mesh_quality_mode == "very_low_optimized":
                            pow_ = 6        # malla más gruesa
                        elif mesh_quality_mode == "display_mesh":
                            pow_ = 12       # malla más fina
                        else:
                            pow_ = 10       # intermedio

                        try:
                            tol = diameter / (2.0 ** pow_)
                            calc.surfaceTolerance = tol
                        except:
                            # Fallback si surfaceTolerance no acepta ese valor
                            if mesh_quality_mode == "very_low_optimized":
                                tol = 0.5
                                calc.surfaceTolerance = 0.5
                            elif mesh_quality_mode == "display_mesh":
                                tol = 0.02
                                calc.surfaceTolerance = 0.02
                            else:
                                tol = 0.1
                                calc.surfaceTolerance = 0.1

                        if _DEBUG_MESH_TREE:
                            _log(
                                f"[MESH]   TriangleMeshCalculator BRep: diameter≈{diameter:.3g} cm, "
                                f"tol≈{tol:.3g} (pow={pow_})",
                                level="DEBUG"
                            )

                        mesh = calc.calculate()
                    except Exception as e:
                        if _DEBUG_MESH_TREE:
                            _log(
                                f"[MESH]   error en TriangleMeshCalculator BRep: {e} → fallback displayMeshes",
                                level="DEBUG"
                            )
                        # Fallback: intentar displayMeshes
                        try:
                            dm = getattr(mesh_mgr, "displayMeshes", None)
                            if dm and dm.count > 0:
                                mesh = dm.item(0)
                                if _DEBUG_MESH_TREE:
                                    _log(
                                        "[MESH]   fallback displayMeshes[0] en BRep tras fallo en calc.calculate()",
                                        level="DEBUG"
                                    )
                        except:
                            mesh = None

                if not mesh:
                    if _DEBUG_MESH_TREE:
                        _log("[MESH][WARN]   BRepBody sin mesh resultante → devuelve malla vacía", level="WARNING")
                    return [], []

                try:
                    coords = list(mesh.nodeCoordinatesAsFloat)
                    if _DEBUG_MESH_TREE:
                        _log("[MESH]   usando mesh.nodeCoordinatesAsFloat en BRep", level="DEBUG")
                except:
                    coords = list(mesh.nodeCoordinates)
                    if _DEBUG_MESH_TREE:
                        _log("[MESH]   usando mesh.nodeCoordinates (no float) en BRep", level="DEBUG")

                indices = list(mesh.nodeIndices)

            # ------------------------
            #  MESH BODY IMPORTADO
            # ------------------------
            elif is_mesh_body:
                if _DEBUG_MESH_TREE:
                    _log("[MESH]   detectado MeshBody (no BRep)", level="DEBUG")

                mb = body
                mesh = None

                # Si se pide display_mesh, priorizamos displayMesh
                if mesh_quality_mode == "display_mesh":
                    try:
                        mesh = mb.displayMesh
                        if mesh and _DEBUG_MESH_TREE:
                            _log("[MESH]   usando mb.displayMesh (DisplayMesh) en MeshBody", level="DEBUG")
                    except Exception as e:
                        mesh = None
                        if _DEBUG_MESH_TREE:
                            _log(
                                f"[MESH]   error leyendo mb.displayMesh: {e} → intentar mb.mesh",
                                level="DEBUG"
                            )

                if mesh is None:
                    try:
                        mesh = mb.mesh
                        if mesh and _DEBUG_MESH_TREE:
                            _log("[MESH]   usando mb.mesh en MeshBody", level="DEBUG")
                    except Exception as e:
                        mesh = None
                        if _DEBUG_MESH_TREE:
                            _log(f"[MESH]   error leyendo mb.mesh: {e}", level="DEBUG")

                # Fallback: si existe displayMesh, úsalo
                if mesh is None:
                    try:
                        mesh = mb.displayMesh
                        if mesh and _DEBUG_MESH_TREE:
                            _log(
                                "[MESH]   fallback: usando mb.displayMesh en MeshBody tras otros fallos",
                                level="DEBUG"
                            )
                    except:
                        pass

                if mesh:
                    try:
                        coords = list(mesh.nodeCoordinatesAsFloat)
                        if _DEBUG_MESH_TREE:
                            _log("[MESH]   usando mesh.nodeCoordinatesAsFloat en MeshBody", level="DEBUG")
                    except:
                        coords = list(mesh.nodeCoordinates)
                        if _DEBUG_MESH_TREE:
                            _log("[MESH]   usando mesh.nodeCoordinates (no float) en MeshBody", level="DEBUG")

                    indices = list(mesh.nodeIndices)
                else:
                    if _DEBUG_MESH_TREE:
                        _log(
                            "[MESH][WARN]   MeshBody sin mesh disponible (ni mesh ni displayMesh) → malla vacía",
                            level="WARNING"
                        )
                    return [], []

            else:
                # No es BRep ni MeshBody convertible
                if _DEBUG_MESH_TREE:
                    _log(
                        f"[MESH][WARN]   _get_mesh_triangles_from_body_basic: "
                        f"body={_describe_entity(body)} no es BRep ni MeshBody → malla vacía",
                        level="WARNING"
                    )
                return [], []

        except Exception as e:
            _log(f"Error en _get_mesh_triangles_from_body_basic (body={_describe_entity(body)}): {e}",
                 level="ERROR")
            return [], []

        # -------------------------------------------------
        #  Conversión de cm -> m
        # -------------------------------------------------
        try:
            scale = design.unitsManager.convert(1.0, "cm", "m")
        except:
            scale = 0.01

        verts_m = []
        for i in range(0, len(coords), 3):
            x = coords[i] * scale
            y = coords[i + 1] * scale
            z = coords[i + 2] * scale
            verts_m.extend([x, y, z])

        if _DEBUG_TRANSFORMS:
            n_verts = len(verts_m) // 3
            n_tris = len(indices) // 3
            _log(
                f"[MESH]   RESULT _get_mesh_triangles_from_body_basic: "
                f"verts={n_verts}, tris={n_tris}, indices_len={len(indices)}",
                level="DEBUG"
            )

        return verts_m, indices

    # -----------------------------------------------------
    #  BREP / MESH + TEXTURAS (atlas por cara para display)
    # -----------------------------------------------------
    @staticmethod
    def brep_mesh_with_per_face_texture(
        body,
        occ,
        design: adsk.fusion.Design,
        texture_dir: str,
        geom_id: str,
        mesh_quality_mode: str = "very_low_optimized"
    ):
        """
        Genera malla + UV + textura PNG para un body (BRep o MeshBody).

        Retorna:
            vertices_m, indices, uv_coords, texture_name (str o None)

        DEBUG:
        - Imprime qué combinación body/occ/geom_id se está exportando.
        - Dice si se trata como MeshBody o BRep.
        - Indica si el atlas por cara se pudo generar y cuántas caras tiene.
        """
        if _DEBUG_MESH_TREE:
            _log(
                f"[MESH] brep_mesh_with_per_face_texture: body={_describe_entity(body)}, "
                f"occ={_describe_entity(occ)}, geom_id='{geom_id}', mode='{mesh_quality_mode}'",
                level="DEBUG"
            )

        # Trabajar SIEMPRE con el body nativo (sin assemblyContext)
        original = body
        body = TransformUtils.native_body(body)
        if _DEBUG_MESH_TREE and original is not body:
            _log(
                f"[MESH]   body era proxy en brep_mesh_with_per_face_texture → nativo={_describe_entity(body)}",
                level="DEBUG"
            )

        # Detectar tipo
        is_brep = hasattr(body, "meshManager")
        is_mesh_body = (hasattr(body, "mesh") or hasattr(body, "displayMesh")) and not is_brep

        if _DEBUG_MESH_TREE:
            _log(
                f"[MESH]   tipo detectado: is_brep={is_brep}, is_mesh_body={is_mesh_body}",
                level="DEBUG"
            )

        # -------------------------------------------------
        #  CASO 1: MESH BODY IMPORTADO
        #         (no hay caras BRep para atlas)
        # -------------------------------------------------
        if is_mesh_body:
            if _DEBUG_MESH_TREE:
                _log("[MESH]   MODO MeshBody: sin atlas por cara (un color sólido por body)", level="DEBUG")

            verts, idxs = MeshUtils._get_mesh_triangles_from_body_basic(
                body, design, mesh_quality_mode
            )
            if not verts or not idxs:
                if _DEBUG_MESH_TREE:
                    _log("[MESH][WARN]   MeshBody sin malla al exportar → devuelve vacío", level="WARNING")
                return [], [], [], None

            tex_name = geom_id + ".png"
            tex_path = os.path.join(texture_dir, tex_name)

            color = ColorUtils.extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            TextureUtils.build_solid_png(tex_path, color, size=4)

            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[MESH]   MeshBody geom_id='{geom_id}' → verts={nverts}, tris={len(idxs)//3}, "
                    f"tex='{tex_name}' (color sólido)",
                    level="DEBUG"
                )

            return verts, idxs, uvs, tex_name

        # -------------------------------------------------
        #  CASO 2: NO ES BREP NI MESH BODY
        # -------------------------------------------------
        if not is_brep:
            # No sabemos qué es, intentamos malla básica genérica
            if _DEBUG_MESH_TREE:
                _log(
                    "[MESH][WARN]   body no es BRep ni MeshBody claro → usando malla básica genérica",
                    level="WARNING"
                )

            verts, idxs = MeshUtils._get_mesh_triangles_from_body_basic(
                body, design, mesh_quality_mode
            )
            if not verts or not idxs:
                if _DEBUG_MESH_TREE:
                    _log("[MESH][WARN]   malla genérica vacía → sin textura/mesh", level="WARNING")
                return [], [], [], None

            tex_name = geom_id + ".png"
            tex_path = os.path.join(texture_dir, tex_name)
            color = ColorUtils.extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            TextureUtils.build_solid_png(tex_path, color, size=4)

            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[MESH]   body genérico geom_id='{geom_id}' → verts={nverts}, tris={len(idxs)//3}, "
                    f"tex='{tex_name}' (color sólido)",
                    level="DEBUG"
                )

            return verts, idxs, uvs, tex_name

        # -------------------------------------------------
        #  CASO 3: BREP BODY
        # -------------------------------------------------
        if _DEBUG_MESH_TREE:
            _log("[MESH]   MODO BRepBody", level="DEBUG")

        # -------------------------
        #  VERY LOW OPTIMIZED
        # -------------------------
        if mesh_quality_mode == "very_low_optimized":
            if _DEBUG_MESH_TREE:
                _log("[MESH]   very_low_optimized: sin atlas, color sólido por body", level="DEBUG")

            verts, idxs = MeshUtils._get_mesh_triangles_from_body_basic(
                body, design, "very_low_optimized"
            )
            if not verts or not idxs:
                if _DEBUG_MESH_TREE:
                    _log("[MESH][WARN]   BRep very_low_optimized sin malla → vacío", level="WARNING")
                return [], [], [], None

            tex_name = geom_id + ".png"
            tex_path = os.path.join(texture_dir, tex_name)
            color = ColorUtils.extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            TextureUtils.build_solid_png(tex_path, color, size=4)

            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[MESH]   BRep very_low geom_id='{geom_id}' → verts={nverts}, "
                    f"tris={len(idxs)//3}, tex='{tex_name}' (color sólido)",
                    level="DEBUG"
                )

            return verts, idxs, uvs, tex_name

        # -------------------------
        #  DISPLAY MESH (ALTA CALIDAD CON ATLAS POR CARA)
        # -------------------------
        faces = getattr(body, "faces", None)
        if not faces or faces.count == 0:
            # Fallback: no hay caras, usar malla básica + color sólido
            if _DEBUG_MESH_TREE:
                _log(
                    "[MESH][WARN]   BRep sin faces o faces.count=0 → "
                    "fallback malla básica + textura sólida",
                    level="WARNING"
                )

            verts, idxs = MeshUtils._get_mesh_triangles_from_body_basic(
                body, design, "display_mesh"
            )
            if not verts or not idxs:
                if _DEBUG_MESH_TREE:
                    _log("[MESH][WARN]   fallback display_mesh también vacío → sin malla", level="WARNING")
                return [], [], [], None

            tex_name = geom_id + ".png"
            tex_path = os.path.join(texture_dir, tex_name)
            color = ColorUtils.extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            TextureUtils.build_solid_png(tex_path, color, size=4)

            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[MESH]   BRep sin faces (fallback basic) geom_id='{geom_id}' → "
                    f"verts={nverts}, tris={len(idxs)//3}, tex='{tex_name}' (color sólido)",
                    level="DEBUG"
                )

            return verts, idxs, uvs, tex_name

        # -------------------------
        #  DISPLAY MESH CON ATLAS POR CARA
        # -------------------------
        if _DEBUG_MESH_TREE:
            _log(
                f"[MESH]   display_mesh con atlas por cara: faces.count={faces.count}",
                level="DEBUG"
            )

        # 1) Colores por cara
        face_colors = []
        for i in range(faces.count):
            f = faces.item(i)
            col = ColorUtils.extract_color_for_face(f, body, occ)
            face_colors.append(col)

        tex_name = geom_id + ".png"
        tex_path = os.path.join(texture_dir, tex_name)

        # 2) Construir atlas PNG
        w, h, uv_centers = TextureUtils.build_faces_atlas_png(
            tex_path, face_colors, cell_px=128
        )

        if _DEBUG_TRANSFORMS:
            _log(
                f"[MESH]   atlas por cara geom_id='{geom_id}' → "
                f"faces={len(face_colors)}, w={w}, h={h}, uv_centers={len(uv_centers)}",
                level="DEBUG"
            )

        if not uv_centers:
            # Fallback a color sólido
            if _DEBUG_MESH_TREE:
                _log(
                    "[MESH][WARN]   uv_centers vacío tras atlas → fallback a color sólido + basic mesh",
                    level="WARNING"
                )

            color = ColorUtils.extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            TextureUtils.build_solid_png(tex_path, color, size=4)
            verts, idxs = MeshUtils._get_mesh_triangles_from_body_basic(
                body, design, "display_mesh"
            )
            if not verts or not idxs:
                if _DEBUG_MESH_TREE:
                    _log("[MESH][WARN]   fallback basic mesh vacío → sin malla", level="WARNING")
                return [], [], [], None

            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[MESH]   BRep display (fallback) geom_id='{geom_id}' → "
                    f"verts={nverts}, tris={len(idxs)//3}, tex='{tex_name}' (color sólido)",
                    level="DEBUG"
                )

            return verts, idxs, uvs, tex_name

        verts_m = []
        indices = []
        uvs = []

        try:
            scale = design.unitsManager.convert(1.0, "cm", "m")
        except:
            scale = 0.01

        faces_with_mesh = 0

        # 3) Triangulación por cara, asignando UV del centro de su celda
        for i in range(faces.count):
            f = faces.item(i)
            uv_u, uv_v = uv_centers[i]

            try:
                mesh_mgr = f.meshManager
                calc = mesh_mgr.createMeshCalculator()

                # Calidad decente para display
                try:
                    bbox = f.boundingBox
                    dx = bbox.maxPoint.x - bbox.minPoint.x
                    dy = bbox.maxPoint.y - bbox.minPoint.y
                    dz = bbox.maxPoint.z - bbox.minPoint.z
                except:
                    bbox = None
                    dx = dy = dz = 1.0

                if bbox is not None:
                    diameter = math.sqrt(dx * dx + dy * dy + dz * dz)
                else:
                    diameter = 1.0

                pow_ = 10

                try:
                    tol = diameter / (2.0 ** pow_)
                    calc.surfaceTolerance = tol
                except:
                    tol = 0.1
                    calc.surfaceTolerance = 0.1

                if _DEBUG_MESH_TREE:
                    _log(
                        f"[MESH]   face[{i}] mesh calc: diameter≈{diameter:.3g} cm, tol≈{tol:.3g}",
                        level="DEBUG"
                    )

                mesh = calc.calculate()
                if not mesh:
                    if _DEBUG_MESH_TREE:
                        _log(f"[MESH][WARN]   face[{i}] sin mesh calculado", level="WARNING")
                    continue
            except Exception as e:
                if _DEBUG_MESH_TREE:
                    _log(f"[MESH][WARN]   error calculando mesh para face[{i}]: {e}", level="WARNING")
                continue

            try:
                coords = list(mesh.nodeCoordinatesAsFloat)
            except:
                coords = list(mesh.nodeCoordinates)

            idxs_face = list(mesh.nodeIndices)

            if not coords or not idxs_face:
                if _DEBUG_MESH_TREE:
                    _log(f"[MESH][WARN]   face[{i}] mesh vacío (coords/idxs)", level="WARNING")
                continue

            faces_with_mesh += 1
            base_index = len(verts_m) // 3

            for k in range(0, len(coords), 3):
                x = coords[k] * scale
                y = coords[k + 1] * scale
                z = coords[k + 2] * scale
                verts_m.extend([x, y, z])
                uvs.extend([uv_u, uv_v])

            for idx in idxs_face:
                indices.append(base_index + int(idx))

        # 4) Fallback si algo salió mal
        if faces_with_mesh == 0 or not verts_m or not indices:
            if _DEBUG_MESH_TREE:
                _log(
                    "[MESH][WARN]   atlas display: ninguna face dio mesh útil → fallback basic + sólido",
                    level="WARNING"
                )

            verts, idxs = MeshUtils._get_mesh_triangles_from_body_basic(
                body, design, "display_mesh"
            )
            if not verts or not idxs:
                if _DEBUG_MESH_TREE:
                    _log("[MESH][WARN]   fallback basic display vacío → sin malla", level="WARNING")
                return [], [], [], None

            color = ColorUtils.extract_color_for_link(body, occ) or (0.7, 0.7, 0.7, 1.0)
            TextureUtils.build_solid_png(tex_path, color, size=4)
            nverts = len(verts) // 3
            uvs = [0.5, 0.5] * nverts

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[MESH]   BRep display fallback basic geom_id='{geom_id}' → "
                    f"verts={nverts}, tris={len(idxs)//3}, tex='{tex_name}' (color sólido)",
                    level="DEBUG"
                )

            return verts, idxs, uvs, tex_name

        if _DEBUG_TRANSFORMS:
            n_verts = len(verts_m) // 3
            n_tris = len(indices) // 3
            _log(
                f"[MESH]   BRep display ATLAS OK geom_id='{geom_id}' → "
                f"faces_with_mesh={faces_with_mesh}, verts={n_verts}, tris={n_tris}, tex='{tex_name}'",
                level="DEBUG"
            )

        return verts_m, indices, uvs, tex_name



























# =========================================================
#  BLOQUE 5 / 8 - ESCRITOR DAE (COLLADA) + DEBUG
# =========================================================

class DAEWriter:
    """Genera archivos COLLADA (.dae) simples, compatibles con el viewer HTML."""

    @staticmethod
    def _compute_bbox(flat_vertices: List[float]):
        """Pequeño helper para bounding box, solo para debug."""
        if not flat_vertices:
            return None
        xs = flat_vertices[0::3]
        ys = flat_vertices[1::3]
        zs = flat_vertices[2::3]
        return (
            (min(xs), min(ys), min(zs)),
            (max(xs), max(ys), max(zs)),
        )

    @staticmethod
    def write_dae(
        filepath: str,
        vertices: List,
        indices: List,
        uv_coords: List,
        texture_path: Optional[str],
        geom_name: str = "mesh"
    ) -> bool:
        """
        Escribe un archivo COLLADA sencillo con:
          - una geometría
          - una malla con triángulos
          - opcionalmente una textura (PNG) asociada

        vertices:
            - lista plana [x0,y0,z0,x1,y1,z1,...] o
            - lista de tuplas [(x,y,z), ...]
        indices:
            - lista de índices (múltiplo de 3)
        uv_coords:
            - lista [u0,v0,u1,v1,...] (2 por vértice)
        texture_path:
            - ruta absoluta del PNG (puede ser None → se genera COLLADA sin <image>)

        DEBUG:
        - Logea cuántos vértices / triángulos entran y salen.
        - Indica si se ha tenido que corregir UVs o recortar índices.
        - Imprime bounding box de la malla (en metros).
        - Muestra nombre de textura y ruta relativa esperada.
        """
        if not vertices or not indices:
            _log(
                f"[DAE][WARN] write_dae: geometría vacía para '{geom_name}' → "
                f"verts={len(vertices)}, indices={len(indices)} → no se escribe {filepath}",
                level="WARNING"
            )
            return False

        try:
            # -------------------------------------------------
            #  Normalizar vertices a lista plana
            # -------------------------------------------------
            if vertices and isinstance(vertices[0], (list, tuple)):
                if _DEBUG_TRANSFORMS:
                    _log(
                        f"[DAE] write_dae: vertices para '{geom_name}' vienen como lista de tuplas "
                        f"(len={len(vertices)}) → se aplanan",
                        level="DEBUG"
                    )
                flat_vertices = []
                for v in vertices:
                    if len(v) == 3:
                        flat_vertices.extend(v)
            else:
                flat_vertices = list(vertices)
                if _DEBUG_TRANSFORMS:
                    _log(
                        f"[DAE] write_dae: vertices para '{geom_name}' ya venían planos "
                        f"(len={len(flat_vertices)})",
                        level="DEBUG"
                    )

            vert_count = len(flat_vertices) // 3
            if vert_count == 0:
                _log(
                    f"[DAE][WARN] write_dae: vert_count=0 para '{geom_name}' → no se escribe {filepath}",
                    level="WARNING"
                )
                return False

            # -------------------------------------------------
            #  Validar y normalizar UVs
            # -------------------------------------------------
            if not uv_coords or len(uv_coords) != vert_count * 2:
                if _DEBUG_TRANSFORMS:
                    _log(
                        f"[DAE][UV][WARN] geom='{geom_name}': "
                        f"len(uv_in)={len(uv_coords)} != 2*vert_count({vert_count}) → "
                        "se asigna UV=(0.5,0.5) a todos los vértices.",
                        level="WARNING"
                    )
                uv_coords = [0.5, 0.5] * vert_count
            else:
                if _DEBUG_TRANSFORMS:
                    _log(
                        f"[DAE][UV] geom='{geom_name}': UVs OK (len={len(uv_coords)} = 2*{vert_count})",
                        level="DEBUG"
                    )
                uv_coords = list(uv_coords)

            # Asegurar que el número de índices sea múltiplo de 3
            tri_count = len(indices) // 3
            if tri_count <= 0:
                _log(
                    f"[DAE][WARN] write_dae: tri_count<=0 para '{geom_name}' "
                    f"(indices_len={len(indices)}) → no se escribe {filepath}",
                    level="WARNING"
                )
            indices = indices[:tri_count * 3]

            # Información de textura
            tex_filename = ""
            if texture_path:
                tex_filename = os.path.basename(texture_path)

            # -------------------------------------------------
            #  DEBUG de geometría de entrada
            # -------------------------------------------------
            if _DEBUG_TRANSFORMS:
                bbox = DAEWriter._compute_bbox(flat_vertices)
                if bbox:
                    (xmin, ymin, zmin), (xmax, ymax, zmax) = bbox
                    _log(
                        f"[DAE] INPUT geom='{geom_name}': "
                        f"verts={vert_count}, tris={tri_count}, "
                        f"bbox_min=({xmin:.4g},{ymin:.4g},{zmin:.4g}), "
                        f"bbox_max=({xmax:.4g},{ymax:.4g},{zmax:.4g}), "
                        f"tex='{tex_filename or 'NONE'}'",
                        level="DEBUG"
                    )
                else:
                    _log(
                        f"[DAE] INPUT geom='{geom_name}': "
                        f"verts={vert_count}, tris={tri_count}, "
                        f"bbox=NONE, tex='{tex_filename or 'NONE'}'",
                        level="DEBUG"
                    )

            # -------------------------------------------------
            #  IDs internos y strings
            # -------------------------------------------------
            geom_id = _sanitize(geom_name)
            pos_id = f"{geom_id}-positions"
            pos_array_id = f"{pos_id}-array"
            uv_id = f"{geom_id}-uv"
            uv_array_id = f"{uv_id}-array"

            positions_str = " ".join(f"{v:.9g}" for v in flat_vertices)
            uvs_str = " ".join(f"{uv:.6f}" for uv in uv_coords)
            indices_str = " ".join(str(int(i)) for i in indices)

            # -------------------------------------------------
            #  Secciones COLLADA
            #  (unit meter, Z_UP para que sea consistente con el viewer)
            # -------------------------------------------------

            # Parte de imágenes/materiales/effects (solo si hay textura)
            if tex_filename:
                if _DEBUG_MESH_TREE:
                    _log(
                        f"[DAE] geom='{geom_name}': usando textura '{tex_filename}' en COLLADA",
                        level="DEBUG"
                    )

                images_block = f"""
  <library_images>
    <image id="{geom_id}-image" name="{geom_id}-image">
      <init_from>{tex_filename}</init_from>
    </image>
  </library_images>

  <library_effects>
    <effect id="mat-effect">
      <profile_COMMON>
        <newparam sid="surface0">
          <surface type="2D">
            <init_from>{geom_id}-image</init_from>
          </surface>
        </newparam>
        <newparam sid="sampler0">
          <sampler2D>
            <source>surface0</source>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <phong>
            <diffuse>
              <texture texture="sampler0" texcoord="TEX0"/>
            </diffuse>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>

  <library_materials>
    <material id="mat" name="mat">
      <instance_effect url="#mat-effect"/>
    </material>
  </library_materials>
"""
                material_ref = """
        <triangles count="{tri_count}" material="mat">
          <input semantic="VERTEX" source="#{geom_id}-vertices" offset="0"/>
          <input semantic="TEXCOORD" source="#{uv_id}" offset="0" set="0"/>
          <p>{indices_str}</p>
        </triangles>
"""
                bind_material_block = """
        <instance_geometry url="#{geom_id}">
          <bind_material>
            <technique_common>
              <instance_material symbol="mat" target="#mat">
                <bind_vertex_input semantic="TEX0" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
"""
            else:
                # Sin textura: geometría pura, sin library_images/materials
                if _DEBUG_MESH_TREE:
                    _log(
                        f"[DAE] geom='{geom_name}': sin textura, COLLADA solo con geometría.",
                        level="DEBUG"
                    )

                images_block = ""
                material_ref = """
        <triangles count="{tri_count}">
          <input semantic="VERTEX" source="#{geom_id}-vertices" offset="0"/>
          <p>{indices_str}</p>
        </triangles>
"""
                bind_material_block = """
        <instance_geometry url="#{geom_id}"/>
"""

            # Construir XML COLLADA completo
            collada_xml = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>URDFConverter DAEWriter</authoring_tool>
    </contributor>
    <unit name="meter" meter="1.0"/>
    <up_axis>Z_UP</up_axis>
  </asset>{images_block}
  <library_geometries>
    <geometry id="{geom_id}" name="{geom_id}">
      <mesh>
        <source id="{pos_id}">
          <float_array id="{pos_array_id}" count="{len(flat_vertices)}">{positions_str}</float_array>
          <technique_common>
            <accessor source="#{pos_array_id}" count="{vert_count}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <source id="{uv_id}">
          <float_array id="{uv_array_id}" count="{len(uv_coords)}">{uvs_str}</float_array>
          <technique_common>
            <accessor source="#{uv_array_id}" count="{vert_count}" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <vertices id="{geom_id}-vertices">
          <input semantic="POSITION" source="#{pos_id}"/>
        </vertices>{material_ref.format(
            tri_count=tri_count,
            geom_id=geom_id,
            uv_id=uv_id,
            indices_str=indices_str
        )}
      </mesh>
    </geometry>
  </library_geometries>

  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="{geom_id}-node" name="{geom_id}">{bind_material_block.format(geom_id=geom_id)}
      </node>
    </visual_scene>
  </library_visual_scenes>

  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>'''

            # -------------------------------------------------
            #  Crear directorio y escribir archivo
            # -------------------------------------------------
            output_dir = os.path.dirname(filepath)
            if output_dir and not ensure_dir(output_dir):
                _log(
                    f"[DAE][ERROR] write_dae: no se pudo crear el directorio '{output_dir}' "
                    f"para geom='{geom_name}'",
                    level="ERROR"
                )
                return False

            with open(filepath, "w", encoding="utf-8") as f:
                f.write(collada_xml)

            if _DEBUG_TRANSFORMS or _DEBUG_MESH_TREE:
                _log(
                    f"[DAE] OK geom='{geom_name}' → file='{filepath}', "
                    f"verts={vert_count}, tris={tri_count}, tex='{tex_filename or 'NONE'}'",
                    level="INFO"
                )

            return True

        except Exception as e:
            _log(f"[DAE][ERROR] Error en write_dae {filepath} (geom='{geom_name}'): {e}",
                 level="ERROR")
            return False































# =========================================================
#  BLOQUE 6 / 8 - ESCRITOR URDF (CON DEBUG)
# =========================================================

class URDFWriter:
    """Genera archivos URDF (.urdf) a partir de la lista de links y joints."""

    # -----------------------------
    #  Helpers de formato
    # -----------------------------
    @staticmethod
    def _fmt_xyz(xyz: Tuple[float, float, float]) -> str:
        """Formatea (x,y,z) a string."""
        try:
            return f"{xyz[0]:.6g} {xyz[1]:.6g} {xyz[2]:.6g}"
        except:
            return "0 0 0"

    @staticmethod
    def _fmt_rpy(rpy: Tuple[float, float, float]) -> str:
        """Formatea (r,p,y) a string."""
        try:
            return f"{rpy[0]:.6g} {rpy[1]:.6g} {rpy[2]:.6g}"
        except:
            return "0 0 0"

    @staticmethod
    def _relative_mesh_path(urdf_path: str, mesh_path: str) -> str:
        """
        Convierte ruta absoluta de mesh a relativa al URDF.

        Si no se puede resolver bien (otra unidad/disco), cae a "meshes/<basename>".

        DEBUG:
        - Si _DEBUG_TRANSFORMS está activo, imprime cómo se resolvió la ruta.
        """
        try:
            urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
            mesh_abs = os.path.abspath(mesh_path)

            common = os.path.commonpath([urdf_dir, mesh_abs])
            if common == urdf_dir:
                rel = os.path.relpath(mesh_abs, urdf_dir)
                rel_norm = rel.replace("\\", "/")
                if _DEBUG_TRANSFORMS:
                    _log(
                        f"[URDF] _relative_mesh_path: urdf_dir='{urdf_dir}', "
                        f"mesh_abs='{mesh_abs}' → rel='{rel_norm}'",
                        level="DEBUG"
                    )
                return rel_norm
            else:
                fallback = "meshes/" + os.path.basename(mesh_path)
                if _DEBUG_TRANSFORMS:
                    _log(
                        f"[URDF][WARN] _relative_mesh_path: urdf_dir='{urdf_dir}' y mesh_abs='{mesh_abs}' "
                        f"no comparten raíz → fallback='{fallback}'",
                        level="WARNING"
                    )
                return fallback

        except Exception as e:
            fallback = "meshes/" + os.path.basename(mesh_path)
            _log(
                f"[URDF][ERROR] _relative_mesh_path lanzó excepción: {e} → fallback='{fallback}'",
                level="ERROR"
            )
            return fallback

    # -----------------------------
    #  Helpers de debug
    # -----------------------------
    @staticmethod
    def _debug_link(link: Dict):
        """Imprime info de un link antes de escribir URDF."""
        if not _DEBUG_TRANSFORMS:
            return
        try:
            name = link.get("name", "unnamed_link")
            mass = link.get("mass", 0.0)
            com = link.get("com", (0.0, 0.0, 0.0))
            inertia = link.get("inertia", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
            origin_xyz = link.get("origin_xyz", (0.0, 0.0, 0.0))
            origin_rpy = link.get("origin_rpy", (0.0, 0.0, 0.0))
            visual = link.get("visual")
            collision = link.get("collision")

            visual_ok = bool(visual and os.path.exists(visual))
            collision_ok = bool(collision and os.path.exists(collision))

            _log(
                "[URDF][LINK] name='{name}', mass={mass:.6g}, "
                "COM={com}, origin_xyz={ox}, origin_rpy={orpy}, "
                "visual='{vis}' (exists={vok}), collision='{col}' (exists={cok})".format(
                    name=_sanitize(name),
                    mass=mass,
                    com=com,
                    ox=origin_xyz,
                    orpy=origin_rpy,
                    vis=visual or "None",
                    vok=visual_ok,
                    col=collision or "None",
                    cok=collision_ok,
                ),
                level="DEBUG"
            )
        except Exception as e:
            _log(f"[URDF][LINK] Error en _debug_link: {e}", level="DEBUG")

    @staticmethod
    def _debug_joint(j: Dict):
        """Imprime info de un joint antes de escribir URDF."""
        if not _DEBUG_TRANSFORMS:
            return
        try:
            name = j.get("name", "unnamed_joint")
            parent = j.get("parent", "unknown")
            child = j.get("child", "unknown")
            jtype = j.get("type", "fixed")
            origin_xyz = j.get("origin_xyz", (0.0, 0.0, 0.0))
            origin_rpy = j.get("origin_rpy", (0.0, 0.0, 0.0))
            axis = j.get("axis", (0.0, 0.0, 1.0))
            limit = j.get("limit", None)

            _log(
                "[URDF][JOINT] name='{name}', type='{jtype}', parent='{parent}', child='{child}', "
                "origin_xyz={ox}, origin_rpy={orpy}, axis={ax}, limit={lim}".format(
                    name=_sanitize(name),
                    jtype=jtype,
                    parent=_sanitize(parent),
                    child=_sanitize(child),
                    ox=origin_xyz,
                    orpy=origin_rpy,
                    ax=axis,
                    lim=limit,
                ),
                level="DEBUG"
            )
        except Exception as e:
            _log(f"[URDF][JOINT] Error en _debug_joint: {e}", level="DEBUG")

    # -----------------------------
    #  Escritura principal de URDF
    # -----------------------------
    @staticmethod
    def write_urdf(
        filepath: str,
        robot_name: str,
        links: List[Dict],
        joints: List[Dict],
        base_link_name: Optional[str] = None
    ) -> bool:
        """
        Genera un archivo URDF completo.

        Espera que cada elemento de `links` tenga al menos:
            {
                "name": str,
                "mass": float,
                "com": (cx, cy, cz)   # COM en coordenadas del link
                "inertia": (ixx, iyy, izz, ixy, ixz, iyz),
                "visual": ruta_abs_dae_o_None,
                "collision": ruta_abs_dae_o_None,
                "origin_xyz": (x,y,z),    # POSE GLOBAL del frame del link
                "origin_rpy": (r,p,y),    # (para debug y joints fixed)
            }

        Y cada elemento de `joints`:
            {
                "name": str,
                "parent": str,
                "child": str,
                "type": "fixed" | "revolute" | "continuous" | "prismatic",
                "origin_xyz": (x,y,z),   # en coords del parent
                "origin_rpy": (r,p,y),   # en coords del parent
                "axis": (ax,ay,az),
                "limit": (lo,hi) | None
            }

        DEBUG:
        - Imprime resumen de nº de links y joints.
        - Para cada link y joint llama a _debug_link / _debug_joint.
        - Al final indica dónde se escribió el URDF.
        """
        if not links:
            _log("No hay links para generar URDF", level="ERROR")
            return False

        try:
            # ---------------------------------------------
            #  Crear directorio de salida si es necesario
            # ---------------------------------------------
            output_dir = os.path.dirname(filepath)
            if output_dir and not ensure_dir(output_dir):
                _log(f"No se pudo crear directorio: {output_dir}", level="ERROR")
                return False

            # ---------------------------------------------
            #  Determinar base_link (solo para nombre del robot)
            #  (La lógica actual usa base_link_name solo de forma informativa)
            # ---------------------------------------------
            if base_link_name is None:
                if any(lnk.get("name") == "base_link" for lnk in links):
                    base_link_name = "base_link"
                elif links:
                    base_link_name = links[0].get("name", "base_link")
                else:
                    base_link_name = "base_link"

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[URDF] write_urdf: robot_name='{_sanitize(robot_name)}', "
                    f"base_link_name='{_sanitize(base_link_name)}', "
                    f"links={len(links)}, joints={len(joints)}, "
                    f"filepath='{filepath}'",
                    level="DEBUG"
                )

            # ---------------------------------------------
            #  Construcción de líneas URDF
            # ---------------------------------------------
            lines: List[str] = []
            lines.append(f'<robot name="{_sanitize(robot_name)}">')

            # ---------------------
            #  LINKS
            # ---------------------
            for link in links:
                URDFWriter._debug_link(link)

                name = link.get("name", "unnamed_link")
                mass = link.get("mass", 1.0)
                com = link.get("com", (0.0, 0.0, 0.0))
                inertia = link.get("inertia", (1.0, 1.0, 1.0, 0.0, 0.0, 0.0))
                visual_path = link.get("visual")
                collision_path = link.get("collision")

                lines.append(f'  <link name="{_sanitize(name)}">')

                # --- Inertial ---
                lines.append('    <inertial>')
                # Aquí usamos COM en coords del link; la rotación del inercial se asume 0
                lines.append(
                    f'      <origin xyz="{URDFWriter._fmt_xyz(com)}" rpy="0 0 0"/>'
                )
                lines.append(f'      <mass value="{mass:.6g}"/>')

                ixx, iyy, izz, ixy, ixz, iyz = inertia
                lines.append(
                    f'      <inertia ixx="{ixx:.6g}" ixy="{ixy:.6g}" ixz="{ixz:.6g}" '
                    f'iyy="{iyy:.6g}" iyz="{iyz:.6g}" izz="{izz:.6g}"/>'  # noqa: E501
                )
                lines.append('    </inertial>')

                # --- Visual ---
                if visual_path and os.path.exists(visual_path):
                    rel_path = URDFWriter._relative_mesh_path(filepath, visual_path)
                    lines.append('    <visual>')
                    # La geometría está ya en coords del link, así que origin=0
                    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                    lines.append(
                        f'      <geometry><mesh filename="{rel_path}" scale="1 1 1"/></geometry>'  # noqa: E501
                    )
                    lines.append('    </visual>')
                else:
                    if visual_path and _DEBUG_TRANSFORMS:
                        _log(
                            f"[URDF][WARN] Link '{name}': visual_path='{visual_path}' no existe en disco.",
                            level="WARNING"
                        )

                # --- Collision ---
                if collision_path and os.path.exists(collision_path):
                    rel_path = URDFWriter._relative_mesh_path(filepath, collision_path)
                    lines.append('    <collision>')
                    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                    lines.append(
                        f'      <geometry><mesh filename="{rel_path}" scale="1 1 1"/></geometry>'  # noqa: E501
                    )
                    lines.append('    </collision>')
                else:
                    if collision_path and _DEBUG_TRANSFORMS:
                        _log(
                            f"[URDF][WARN] Link '{name}': collision_path='{collision_path}' no existe en disco.",
                            level="WARNING"
                        )

                lines.append('  </link>')

            # ---------------------
            #  JOINTS
            # ---------------------
            for j in joints:
                URDFWriter._debug_joint(j)

                name = j.get("name", "unnamed_joint")
                parent = j.get("parent", "unknown")
                child = j.get("child", "unknown")
                jtype = j.get("type", "fixed")
                origin_xyz = j.get("origin_xyz", (0.0, 0.0, 0.0))
                origin_rpy = j.get("origin_rpy", (0.0, 0.0, 0.0))
                axis = j.get("axis", (0.0, 0.0, 1.0))
                limit = j.get("limit", None)

                # Normalizar eje
                ax = vec_normalize(axis)

                lines.append(
                    f'  <joint name="{_sanitize(name)}" type="{jtype}">'
                )
                lines.append(
                    f'    <origin xyz="{URDFWriter._fmt_xyz(origin_xyz)}" '
                    f'rpy="{URDFWriter._fmt_rpy(origin_rpy)}"/>'  # noqa: E501
                )
                lines.append(f'    <parent link="{_sanitize(parent)}"/>')
                lines.append(f'    <child link="{_sanitize(child)}"/>')

                # Ejes y límites para articulaciones móviles
                if jtype in ("revolute", "continuous", "prismatic"):
                    lines.append(
                        f'    <axis xyz="{URDFWriter._fmt_xyz(ax)}"/>'  # reuse _fmt_xyz para axis
                    )

                if limit is not None and jtype in ("revolute", "prismatic"):
                    lo, hi = limit
                    # Si vienen en grados (valores grandes), convierto a radianes
                    if abs(lo) > 2 * math.pi or abs(hi) > 2 * math.pi:
                        lo = math.radians(lo)
                        hi = math.radians(hi)

                    lines.append(
                        f'    <limit lower="{lo:.6g}" upper="{hi:.6g}" effort="10" velocity="1"/>'  # noqa: E501
                    )

                lines.append('  </joint>')

            lines.append('</robot>')

            # ---------------------------------------------
            #  Escribir archivo a disco
            # ---------------------------------------------
            with open(filepath, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))

            _log(f"[URDF] URDF generado exitosamente: {filepath}", level="INFO")
            return True

        except Exception as e:
            _log(f"Error al escribir URDF {filepath}: {e}", level="ERROR")
            return False




































# =========================================================
#  BLOQUE 7 / 8 - CONSTRUCTOR DE ROBOT (RobotBuilder)
#                  (CON DEBUG DE OCCURRENCES/LINKS/JOINTS)
# =========================================================

class RobotBuilder:
    """
    Construye la estructura del robot:
      - Genera links (uno principal por occurrence + extras).
      - Exporta todos los cuerpos convertibles (BRep + MeshBodies) a .dae.
      - Calcula poses GLOBALES y LOCALES con SISTEMA CÓDIGO 1.
      - Reconstruye joints para que el URDF se vea como en Fusion 360.

    NOTA SOBRE TRANSFORMACIONES (soporta "todas las combinaciones"):
      - En Fusion 360 SOLO las occurrences tienen transformaciones propias.
      - BRepBodies, MeshBodies, caras, etc. viven en el sistema local del componente.
      - El sistema usa SIEMPRE:
          * Geometría en coords del COMPONENTE (native_body)  → local infinita
          * transform2 de la occurrence (ya acumula toda la cadena de padres)
        Con eso cubres casos como:
          - meshes dentro de breps dentro de breps dentro de occurrences
          - breps dentro de occurrences anidadas
          - meshBodies en root, en componentes, en sub-componentes, etc.
        Es decir, cualquier profundidad de anidación queda absorbida por transform2.

    DEBUG:
      - Imprime para cada occurrence:
          * nombre, fullPathName, clave estable, transform2 (xyz cm → m).
          * cuántos BRep/Mesh hay en comp vs occ.
      - Para cada link:
          * mass, COM_global, COM_local, origin_xyz / origin_rpy.
          * qué body fue el "principal" y cuántos extras tiene.
      - Para cada joint:
          * parent, child, type, origin_xyz/rpy, axis, limit.
    """

    # -----------------------------------------------------
    #  INIT
    # -----------------------------------------------------
    def __init__(self,
                 design: adsk.fusion.Design,
                 robot_name: str,
                 output_dir: str,
                 mesh_mode: str = "very_low"):
        if not design:
            raise ValueError("Diseño no válido")

        self.design = design
        self.robot_name = _sanitize(robot_name)
        self.output_dir = output_dir
        self.mesh_dir = os.path.join(output_dir, "meshes")

        # Mapear modos externos a internos
        if mesh_mode == "very_low":
            self.mesh_mode_internal = "very_low_optimized"
        elif mesh_mode == "display":
            self.mesh_mode_internal = "display_mesh"
        else:
            self.mesh_mode_internal = "very_low_optimized"
            _log(
                f"Advertencia: Modo de malla '{mesh_mode}' no reconocido, usando 'very_low'",
                level="WARNING"
            )

        # Estructuras internas
        self.links: List[Dict] = []
        self.joints: List[Dict] = []

        # Mapas occurrence <-> link
        self.occ_to_link: Dict[str, str] = {}
        self.link_to_occ: Dict[str, Optional[adsk.fusion.Occurrence]] = {}

        self.single_link_no_joints: bool = False
        self.base_link_name: Optional[str] = None

        _log(
            f"Inicializado RobotBuilder para '{self.robot_name}' "
            f"con mesh_mode_internal='{self.mesh_mode_internal}'",
            level="INFO"
        )

    # -----------------------------------------------------
    #  Helpers de debug internos
    # -----------------------------------------------------
    @staticmethod
    def _occ_debug_id(occ: Optional[adsk.fusion.Occurrence]) -> str:
        if not occ:
            return "<None>"
        try:
            fp = getattr(occ, "fullPathName", "")
        except Exception:
            fp = ""
        try:
            name = occ.name
        except Exception:
            name = "<no-name>"
        if fp:
            return f"{name} | path='{fp}'"
        return name

    def _debug_occ_collect(
        self,
        occ: adsk.fusion.Occurrence,
        comp_brep,
        occ_brep,
        comp_mesh,
        occ_mesh
    ):
        """Debug de qué bodies/meshes se detectaron para una occurrence."""
        if not (_DEBUG_MESH_TREE or _DEBUG_TRANSFORMS):
            return

        try:
            key = get_occurrence_key(occ)
        except Exception:
            key = "<err-key>"

        try:
            m = occ.transform2
            t = m.translation
            xyz_cm = (t.x, t.y, t.z)
            xyz_m, rpy = TransformUtils.occ_abs_pose(occ, self.design)
        except Exception:
            xyz_cm = (0.0, 0.0, 0.0)
            xyz_m = (0.0, 0.0, 0.0)
            rpy = (0.0, 0.0, 0.0)

        _log(
            "[RB][OCC] {id} | key='{key}' | "
            "comp_brep={cb}, occ_brep={ob}, comp_mesh={cm}, occ_mesh={om} | "
            "T_world(cm)={tcm} → xyz_m={xyz} rpy={rpy}".format(
                id=self._occ_debug_id(occ),
                key=key,
                cb=len(comp_brep),
                ob=len(occ_brep),
                cm=len(comp_mesh),
                om=len(occ_mesh),
                tcm=xyz_cm,
                xyz=xyz_m,
                rpy=rpy,
            ),
            level="DEBUG"
        )

    def _debug_link_created(
        self,
        link_name: str,
        occ: Optional[adsk.fusion.Occurrence],
        main_item,
        extras: List[Any],
        mass: float,
        com_global,
        com_local,
        xyz_world,
        rpy_world,
        visual_path: Optional[str]
    ):
        if not _DEBUG_TRANSFORMS:
            return

        _log(
            "[RB][LINK] main link='{lnk}' from occ={occ_id} | main_body={main} | extras={n_extras} | "
            "mass={mass:.6g}, COM_global={cg}, COM_local={cl}, origin_xyz={xyz}, origin_rpy={rpy}, "
            "visual='{vis}'".format(
                lnk=link_name,
                occ_id=self._occ_debug_id(occ),
                main=_describe_entity(main_item),
                n_extras=len(extras),
                mass=mass,
                cg=com_global,
                cl=com_local,
                xyz=xyz_world,
                rpy=rpy_world,
                vis=visual_path or "None",
            ),
            level="DEBUG"
        )

    def _debug_extra_link(
        self,
        parent_link: str,
        extra_name: str,
        extra,
        occ: Optional[adsk.fusion.Occurrence],
        mass_e: float,
        com_global_e,
        com_local_e,
        visual_extra: Optional[str]
    ):
        if not _DEBUG_TRANSFORMS:
            return

        _log(
            "[RB][EXTRA] extra_link='{ename}' (parent='{parent}') from occ={occ_id} | "
            "body={body} | mass={mass:.6g}, COM_global={cg}, COM_local={cl}, visual='{vis}'".format(
                ename=extra_name,
                parent=parent_link,
                occ_id=self._occ_debug_id(occ),
                body=_describe_entity(extra),
                mass=mass_e,
                cg=com_global_e,
                cl=com_local_e,
                vis=visual_extra or "None",
            ),
            level="DEBUG"
        )

    def _debug_root_link(
        self,
        link_name: str,
        body_or_mesh,
        mass: float,
        com_global,
        visual_path: Optional[str],
        kind: str
    ):
        if not _DEBUG_TRANSFORMS:
            return

        _log(
            "[RB][ROOT-{kind}] link='{lnk}', body={body}, mass={mass:.6g}, COM_global={cg}, visual='{vis}'".format(
                kind=kind.upper(),
                lnk=link_name,
                body=_describe_entity(body_or_mesh),
                mass=mass,
                cg=com_global,
                vis=visual_path or "None",
            ),
            level="DEBUG"
        )

    def _debug_relative_pose(
        self,
        parent_occ: Optional[adsk.fusion.Occurrence],
        child_occ: adsk.fusion.Occurrence,
        xyz_m,
        rpy
    ):
        if not _DEBUG_TRANSFORMS:
            return

        _log(
            "[RB][RELPOSE] parent={p_id} → child={c_id} | "
            "origin_xyz={xyz}, origin_rpy={rpy}".format(
                p_id=self._occ_debug_id(parent_occ),
                c_id=self._occ_debug_id(child_occ),
                xyz=xyz_m,
                rpy=rpy,
            ),
            level="DEBUG"
        )

    def _debug_fixed_to_base(
        self,
        link_name: str,
        xyz,
        rpy,
        occ: Optional[adsk.fusion.Occurrence]
    ):
        if not _DEBUG_LINK_JOINT:
            return

        _log(
            "[RB][FIXED→BASE] base_link → '{lnk}' | origin_xyz={xyz}, origin_rpy={rpy}, occ={occ_id}".format(
                lnk=link_name,
                xyz=xyz,
                rpy=rpy,
                occ_id=self._occ_debug_id(occ),
            ),
            level="DEBUG"
        )

    def _debug_fusion_joint_choice(
        self,
        j,
        occ1,
        occ2,
        link1_name: str,
        link2_name: str,
        parent_name: str,
        child_name: str,
        jtype: str,
        axis,
        limit
    ):
        if not _DEBUG_LINK_JOINT:
            return

        _log(
            "[RB][FJOINT] '{jname}' | type='{jt}' | occ1={o1}→link='{l1}', "
            "occ2={o2}→link='{l2}' | chosen parent='{p}' child='{c}' | "
            "axis={ax}, limit={lim}".format(
                jname=_sanitize(getattr(j, "name", "") or "<no-name>"),
                jt=jtype,
                o1=self._occ_debug_id(occ1),
                o2=self._occ_debug_id(occ2),
                l1=link1_name,
                l2=link2_name,
                p=parent_name,
                c=child_name,
                ax=axis,
                lim=limit,
            ),
            level="DEBUG"
        )

    # -----------------------------------------------------
    #  Helper: ¿hay joints de Fusion?
    # -----------------------------------------------------
    def _has_fusion_joints(self, root) -> bool:
        """Devuelve True si el root tiene joints/asBuiltJoints."""
        try:
            if len(root.joints) > 0:
                return True
        except:
            pass
        try:
            if len(root.asBuiltJoints) > 0:
                return True
        except:
            pass
        return False

    # -----------------------------------------------------
    #  Helper: pose relativa parent → child usando occurrences
    # -----------------------------------------------------
    def _relative_pose_from_occs(
        self,
        parent_occ: Optional[adsk.fusion.Occurrence],
        child_occ: adsk.fusion.Occurrence
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Calcula la pose RELATIVA child respecto al parent usando matrices completas:

            T_world^P = transform2(parent_occ)  (o identidad si parent es None)
            T_world^C = transform2(child_occ)

            T_P^C = (T_world^P)^-1 * T_world^C

        Devuelve (xyz_m, rpy) EN COORDENADAS DEL PARENT.

        Esto es lo que se usa como <origin> del joint en URDF para que
        las occurrences queden en la misma posición/orientación que en Fusion.
        """
        design = self.design

        # Escala cm → m
        try:
            scale = design.unitsManager.convert(1.0, "cm", "m")
        except:
            scale = 0.01

        # -----------------------------
        #  Matriz del parent
        # -----------------------------
        if parent_occ is None:
            # Parent = base_link (identidad)
            R_P = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
            t_P = (0.0, 0.0, 0.0)
        else:
            mP = parent_occ.transform2
            tP = mP.translation
            t_P = (tP.x, tP.y, tP.z)
            R_P = [
                [mP.getCell(0, 0), mP.getCell(0, 1), mP.getCell(0, 2)],
                [mP.getCell(1, 0), mP.getCell(1, 1), mP.getCell(1, 2)],
                [mP.getCell(2, 0), mP.getCell(2, 1), mP.getCell(2, 2)],
            ]

        # -----------------------------
        #  Matriz del child
        # -----------------------------
        mC = child_occ.transform2
        tC = mC.translation
        t_C = (tC.x, tC.y, tC.z)
        R_C = [
            [mC.getCell(0, 0), mC.getCell(0, 1), mC.getCell(0, 2)],
            [mC.getCell(1, 0), mC.getCell(1, 1), mC.getCell(1, 2)],
            [mC.getCell(2, 0), mC.getCell(2, 1), mC.getCell(2, 2)],
        ]

        # -----------------------------
        #  R_P^T y (t_C - t_P)
        # -----------------------------
        dp_x = t_C[0] - t_P[0]
        dp_y = t_C[1] - t_P[1]
        dp_z = t_C[2] - t_P[2]

        # R_P^T
        R_PT = [
            [R_P[0][0], R_P[1][0], R_P[2][0]],
            [R_P[0][1], R_P[1][1], R_P[2][1]],
            [R_P[0][2], R_P[1][2], R_P[2][2]],
        ]

        # p_rel = R_P^T * (t_C - t_P)   (en cm)
        p_rel_x = R_PT[0][0] * dp_x + R_PT[0][1] * dp_y + R_PT[0][2] * dp_z
        p_rel_y = R_PT[1][0] * dp_x + R_PT[1][1] * dp_y + R_PT[1][2] * dp_z
        p_rel_z = R_PT[2][0] * dp_x + R_PT[2][1] * dp_y + R_PT[2][2] * dp_z

        # -----------------------------
        #  R_rel = R_P^T * R_C
        # -----------------------------
        R_rel = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                R_rel[i][j] = (
                    R_PT[i][0] * R_C[0][j] +
                    R_PT[i][1] * R_C[1][j] +
                    R_PT[i][2] * R_C[2][j]
                )

        r11, r12, r13 = R_rel[0]
        r21, r22, r23 = R_rel[1]
        r31, r32, r33 = R_rel[2]

        # -----------------------------
        #  Convertir R_rel a RPY (igual que _matrix_to_xyz_rpy)
        # -----------------------------
        if abs(r31) < 1.0:
            pitch = math.asin(-r31)
            roll = math.atan2(r32, r33)
            yaw = math.atan2(r21, r11)
        else:
            pitch = math.pi / 2 if r31 <= -1.0 else -math.pi / 2
            roll = 0.0
            yaw = math.atan2(-r12, r22)

        # -----------------------------
        #  Traducir p_rel a metros
        # -----------------------------
        xyz_m = (p_rel_x * scale, p_rel_y * scale, p_rel_z * scale)
        rpy = (roll, pitch, yaw)

        self._debug_relative_pose(parent_occ, child_occ, xyz_m, rpy)
        return xyz_m, rpy

    # -----------------------------------------------------
    #  Construcción principal del robot
    # -----------------------------------------------------
    def build(self) -> Tuple[List[Dict], List[Dict]]:
        """
        Construye la estructura completa del robot:

          - Genera links por occurrence (y extras por body extra).
          - Genera links también para bodies sueltos en el root.
          - Crea joints de Fusion (no rígidos).
          - Conecta el resto como fixed joints a base_link.

        Devuelve:
            (links, joints)
        """
        try:
            root = self.design.rootComponent
            all_occs = list(root.allOccurrences)

            _log(f"Total occurrences encontradas = {len(all_occs)}", level="INFO")

            idx = 0

            # -------------------------------------------------
            #  1) Links por occurrence (BRep + MeshBodies)
            # -------------------------------------------------
            for occ in all_occs:
                comp_brep, occ_brep, comp_mesh, occ_mesh = BodyCollector.collect_bodies_for_occurrence(occ)

                # Debug de qué se recogió en esta occurrence
                self._debug_occ_collect(occ, comp_brep, occ_brep, comp_mesh, occ_mesh)

                # Todos los cuerpos convertibles: BRep + Mesh
                bodies = list(occ_brep) + list(comp_brep)
                meshes = list(occ_mesh) + list(comp_mesh)
                exported_items = bodies + meshes

                if not exported_items:
                    continue

                occ_key = get_occurrence_key(occ)

                main_item = exported_items[0]
                extras = exported_items[1:]

                link_name = _sanitize(f"link_{idx}_{occ.name}")
                idx += 1

                # Pose absoluta del occurrence (CÓDIGO 1)
                xyz_world, rpy_world = TransformUtils.occ_abs_pose(occ, self.design)

                # Física combinada de TODOS los bodies/meshes de la occurrence
                phys_list = []
                for it in exported_items:
                    native = TransformUtils.native_body(it)
                    phys = PhysicsUtils.get_body_physical_properties(native)
                    phys_list.append(phys)

                mass, com_global, inertia_global = PhysicsUtils.combine_bodies_physics(phys_list)

                # COM local (respecto al origen del link = frame del occurrence)
                com_local = (
                    com_global[0] - xyz_world[0],
                    com_global[1] - xyz_world[1],
                    com_global[2] - xyz_world[2],
                )

                # Malla visual del body principal (en coords del componente nativo)
                visual_path = None
                verts, tris, uv, tex = MeshUtils.brep_mesh_with_per_face_texture(
                    main_item,
                    occ,
                    self.design,
                    self.mesh_dir,
                    link_name,
                    self.mesh_mode_internal
                )

                if verts and tris:
                    dae_path = os.path.join(self.mesh_dir, f"{link_name}.dae")
                    if DAEWriter.write_dae(dae_path, verts, tris, uv, tex, geom_name=link_name):
                        visual_path = dae_path

                # Link principal de la occurrence
                main_link = {
                    "name": link_name,
                    "mass": mass,
                    "com": com_local,
                    "inertia": inertia_global,
                    "visual": visual_path,
                    "collision": visual_path,
                    "origin_xyz": xyz_world,   # pose global del frame del link
                    "origin_rpy": rpy_world,
                    "occ": occ,
                }
                self.links.append(main_link)

                # Debug link principal
                self._debug_link_created(
                    link_name,
                    occ,
                    main_item,
                    extras,
                    mass,
                    com_global,
                    com_local,
                    xyz_world,
                    rpy_world,
                    visual_path
                )

                self.occ_to_link[occ_key] = link_name
                self.link_to_occ[link_name] = occ

                # -----------------------------
                #  Extras bodies como sub-links
                # -----------------------------
                for i, extra in enumerate(extras, start=1):
                    extra_name = _sanitize(f"{link_name}_b{i}_{extra.name}")
                    native_extra = TransformUtils.native_body(extra)
                    mass_e, com_e_global, inertia_e = PhysicsUtils.get_body_physical_properties(native_extra)

                    # COM local de extra respecto al mismo frame de occurrence
                    com_e_local = (
                        com_e_global[0] - xyz_world[0],
                        com_e_global[1] - xyz_world[1],
                        com_e_global[2] - xyz_world[2],
                    )

                    visual_extra = None
                    verts_e, tris_e, uv_e, tex_e = MeshUtils.brep_mesh_with_per_face_texture(
                        extra,
                        occ,
                        self.design,
                        self.mesh_dir,
                        extra_name,
                        self.mesh_mode_internal
                    )

                    if verts_e and tris_e:
                        dae_extra = os.path.join(self.mesh_dir, f"{extra_name}.dae")
                        if DAEWriter.write_dae(dae_extra, verts_e, tris_e, uv_e, tex_e, geom_name=extra_name):
                            visual_extra = dae_extra

                    self.links.append({
                        "name": extra_name,
                        "mass": mass_e,
                        "com": com_e_local,
                        "inertia": inertia_e,
                        "visual": visual_extra,
                        "collision": visual_extra,
                        "origin_xyz": xyz_world,
                        "origin_rpy": rpy_world,
                        "occ": occ,
                    })

                    # Debug extra link
                    self._debug_extra_link(
                        parent_link=link_name,
                        extra_name=extra_name,
                        extra=extra,
                        occ=occ,
                        mass_e=mass_e,
                        com_global_e=com_e_global,
                        com_local_e=com_e_local,
                        visual_extra=visual_extra
                    )

                    # Joint fijo entre el link principal de la occurrence y el extra
                    self.joints.append({
                        "name": f"fixed_extra_{extra_name}",
                        "parent": link_name,
                        "child": extra_name,
                        "type": "fixed",
                        "origin_xyz": (0.0, 0.0, 0.0),
                        "origin_rpy": (0.0, 0.0, 0.0),
                        "axis": (0.0, 0.0, 1.0),
                        "limit": None,
                    })

            # -------------------------------------------------
            #  2) Cuerpos sueltos en root (BRep + Mesh)
            # -------------------------------------------------
            try:
                root_bodies = list(root.bRepBodies)
            except:
                root_bodies = []

            try:
                root_meshes = list(getattr(root, "meshBodies", []))
            except:
                root_meshes = []

            # BRep en root (no occurrences)
            for i, body in enumerate(root_bodies):
                link_name = _sanitize(f"root_body_{i}_{body.name}")

                native = TransformUtils.native_body(body)
                mass, com_global, inertia_global = PhysicsUtils.get_body_physical_properties(native)

                visual_path = None
                verts, tris, uv, tex = MeshUtils.brep_mesh_with_per_face_texture(
                    native,
                    None,
                    self.design,
                    self.mesh_dir,
                    link_name,
                    self.mesh_mode_internal
                )

                if verts and tris:
                    dae_path = os.path.join(self.mesh_dir, f"{link_name}.dae")
                    if DAEWriter.write_dae(dae_path, verts, tris, uv, tex, geom_name=link_name):
                        visual_path = dae_path

                self.links.append({
                    "name": link_name,
                    "mass": mass,
                    "com": com_global,  # root_body ya está en el frame global
                    "inertia": inertia_global,
                    "visual": visual_path,
                    "collision": visual_path,
                    "origin_xyz": (0.0, 0.0, 0.0),
                    "origin_rpy": (0.0, 0.0, 0.0),
                    "occ": None,
                })
                self.link_to_occ[link_name] = None

                self._debug_root_link(
                    link_name=link_name,
                    body_or_mesh=body,
                    mass=mass,
                    com_global=com_global,
                    visual_path=visual_path,
                    kind="brep"
                )

            # MeshBodies sueltos en root
            for j, mbody in enumerate(root_meshes):
                link_name = _sanitize(f"root_mesh_{j}_{mbody.name}")

                native = TransformUtils.native_body(mbody)
                mass, com_global, inertia_global = PhysicsUtils.get_body_physical_properties(native)

                visual_path = None
                verts, tris, uv, tex = MeshUtils.brep_mesh_with_per_face_texture(
                    native,
                    None,
                    self.design,
                    self.mesh_dir,
                    link_name,
                    self.mesh_mode_internal
                )

                if verts and tris:
                    dae_path = os.path.join(self.mesh_dir, f"{link_name}.dae")
                    if DAEWriter.write_dae(dae_path, verts, tris, uv, tex, geom_name=link_name):
                        visual_path = dae_path

                self.links.append({
                    "name": link_name,
                    "mass": mass,
                    "com": com_global,
                    "inertia": inertia_global,
                    "visual": visual_path,
                    "collision": visual_path,
                    "origin_xyz": (0.0, 0.0, 0.0),
                    "origin_rpy": (0.0, 0.0, 0.0),
                    "occ": None,
                })
                self.link_to_occ[link_name] = None

                self._debug_root_link(
                    link_name=link_name,
                    body_or_mesh=mbody,
                    mass=mass,
                    com_global=com_global,
                    visual_path=visual_path,
                    kind="mesh"
                )

            # -------------------------------------------------
            #  Nada que exportar
            # -------------------------------------------------
            if not self.links:
                raise RuntimeError("No se encontraron cuerpos sólidos ni mallas convertibles.")

            has_fusion_joints = self._has_fusion_joints(root)

            # Caso trivial: un solo link y sin joints
            if not has_fusion_joints and len(self.links) == 1:
                self.single_link_no_joints = True
                self.base_link_name = None
                _log("Solo un link y sin joints: se exporta un URDF de un solo link.", level="INFO")
                return self.links, self.joints

            # -------------------------------------------------
            #  3) base_link
            # -------------------------------------------------
            self.base_link_name = "base_link"
            self.links.insert(0, {
                "name": self.base_link_name,
                "mass": 1e-6,
                "com": (0.0, 0.0, 0.0),
                "inertia": (1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0),
                "visual": None,
                "collision": None,
                "origin_xyz": (0.0, 0.0, 0.0),
                "origin_rpy": (0.0, 0.0, 0.0),
                "occ": None,
            })
            self.link_to_occ[self.base_link_name] = None

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[RB] base_link insertado. Total links ahora = {len(self.links)}",
                    level="DEBUG"
                )

            # -------------------------------------------------
            #  4) Joints de Fusion (no rígidos)
            # -------------------------------------------------
            if has_fusion_joints:
                all_fusion_joints = []

                try:
                    all_fusion_joints.extend(root.joints)
                except:
                    pass
                try:
                    all_fusion_joints.extend(root.asBuiltJoints)
                except:
                    pass

                used_children: set = set()

                for j in all_fusion_joints:
                    try:
                        occ1 = getattr(j, "occurrenceOne", None)
                        occ2 = getattr(j, "occurrenceTwo", None)

                        if occ1 is None or occ2 is None:
                            continue

                        k1 = get_occurrence_key(occ1)
                        k2 = get_occurrence_key(occ2)
                        l1 = self.occ_to_link.get(k1)
                        l2 = self.occ_to_link.get(k2)
                        if not l1 or not l2:
                            continue

                        jtype, axis, limit = TransformUtils.map_fusion_joint(j)

                        # Ignorar joints rígidos (ya representados por frames fijos)
                        if jtype == "fixed":
                            if _DEBUG_LINK_JOINT:
                                _log(
                                    f"[RB][FJOINT] joint '{getattr(j, 'name', '<no-name>')}' "
                                    f"entre {self._occ_debug_id(occ1)} y {self._occ_debug_id(occ2)} "
                                    "es 'fixed' → se ignora (ya representado por links).",
                                    level="DEBUG"
                                )
                            continue

                        # Elegir parent / child con heurística (evitar ciclos)
                        if l1 in used_children and l2 not in used_children:
                            parent_name, child_name = l2, l1
                        elif l2 in used_children and l1 not in used_children:
                            parent_name, child_name = l1, l2
                        else:
                            parent_name, child_name = (l1, l2) if l1 < l2 else (l2, l1)

                        used_children.add(child_name)

                        parent_occ = self.link_to_occ.get(parent_name)
                        child_occ = self.link_to_occ.get(child_name)

                        # Pose relativa parent → child (usa transform2 de Fusion)
                        xyz_rel, rpy_rel = self._relative_pose_from_occs(parent_occ, child_occ)

                        j_name = _sanitize(getattr(j, "name", "") or f"joint_{parent_name}_to_{child_name}")

                        self._debug_fusion_joint_choice(
                            j,
                            occ1,
                            occ2,
                            l1,
                            l2,
                            parent_name,
                            child_name,
                            jtype,
                            axis,
                            limit
                        )

                        self.joints.append({
                            "name": j_name,
                            "parent": parent_name,
                            "child": child_name,
                            "type": jtype,
                            "origin_xyz": xyz_rel,
                            "origin_rpy": rpy_rel,
                            "axis": axis,
                            "limit": limit,
                        })

                    except Exception as e:
                        _log(f"Error al procesar joint de Fusion: {e}", level="ERROR")
                        continue


            # -------------------------------------------------
            #  5) Links sin joints *reales* → fixed a base_link
            #      (ignoramos joints internos fixed_extra_*)
            # -------------------------------------------------

            # Sólo contamos joints "reales" (no los fixed_extra_ que unen
            # sub-bodies dentro de la misma occurrence).
            joint_children = {
                j["child"]
                for j in self.joints
                if not j.get("name", "").startswith("fixed_extra_")
            }
            joint_parents = {
                j["parent"]
                for j in self.joints
                if not j.get("name", "").startswith("fixed_extra_")
            }

            for link in self.links:
                name = link["name"]
                if name == self.base_link_name:
                    continue

                # Si el link no aparece ni como parent ni como child
                # en ningún joint REAL, lo colgamos de base_link.
                if name not in joint_children and name not in joint_parents:
                    occ = self.link_to_occ.get(name)

                    if occ is not None:
                        # Pose global del occurrence en Fusion
                        xyz, rpy = TransformUtils.occ_abs_pose(occ, self.design)
                    else:
                        # Fallback por si no tenemos occ asociado
                        xyz = link.get("origin_xyz", (0.0, 0.0, 0.0))
                        rpy = link.get("origin_rpy", (0.0, 0.0, 0.0))

                    self._debug_fixed_to_base(name, xyz, rpy, occ)

                    # Usamos el prefijo 'root_' para que coincida con tu URDF bueno:
                    #   root_link_1_BajoTubo1_v1_1, etc.
                    joint_name = f"root_{name}"

                    self.joints.append({
                        "name": joint_name,
                        "parent": self.base_link_name,
                        "child": name,
                        "type": "fixed",
                        "origin_xyz": xyz,
                        "origin_rpy": rpy,
                        "axis": (0.0, 0.0, 1.0),
                        "limit": None,
                    })



            # -------------------------------------------------
            #  5) Links sin joints *reales* → fixed a base_link
            #      (ignoramos joints internos fixed_extra_*)
            # -------------------------------------------------

            # Sólo contamos joints "reales" (no los fixed_extra_ que unen
            # sub-bodies dentro de la misma occurrence).
            joint_children = {
                j["child"]
                for j in self.joints
                if not j.get("name", "").startswith("fixed_extra_")
            }
            joint_parents = {
                j["parent"]
                for j in self.joints
                if not j.get("name", "").startswith("fixed_extra_")
            }

            for link in self.links:
                name = link["name"]
                if name == self.base_link_name:
                    continue

                # Si el link no aparece ni como parent ni como child
                # en ningún joint REAL, lo colgamos de base_link.
                if name not in joint_children and name not in joint_parents:
                    occ = self.link_to_occ.get(name)

                    if occ is not None:
                        # Pose global del occurrence en Fusion
                        xyz, rpy = TransformUtils.occ_abs_pose(occ, self.design)
                    else:
                        # Fallback por si no tenemos occ asociado
                        xyz = link.get("origin_xyz", (0.0, 0.0, 0.0))
                        rpy = link.get("origin_rpy", (0.0, 0.0, 0.0))

                    self._debug_fixed_to_base(name, xyz, rpy, occ)

                    # Usamos el prefijo 'root_' para que coincida con tu URDF bueno:
                    #   root_link_1_BajoTubo1_v1_1, etc.
                    joint_name = f"root_{name}"

                    self.joints.append({
                        "name": joint_name,
                        "parent": self.base_link_name,
                        "child": name,
                        "type": "fixed",
                        "origin_xyz": xyz,
                        "origin_rpy": rpy,
                        "axis": (0.0, 0.0, 1.0),
                        "limit": None,
                    })



            _log(f"Robot construido: {len(self.links)} links, {len(self.joints)} joints", level="INFO")
            return self.links, self.joints

        except Exception as e:
            _log(
                f"Error en RobotBuilder.build(): {e}\n{traceback.format_exc()}",
                also_messagebox=True,
                level="ERROR"
            )
            raise





























# =========================================================
#  BLOQUE 8 / 8 - EXPORTADOR PRINCIPAL (RobotExporter)
#                  + HELPER DE DEBUG GLOBAL
# =========================================================

def _describe_entity(ent: Any) -> str:
    """
    Pequeño helper para describir BRepBodies, MeshBodies, occurrences, etc.
    Lo usamos en los prints de debug para entender QUÉ entidad
    está generando un link o mesh.

    Ejemplos de salida:
      - "BRepBody(name='Cubo', isProxy=False)"
      - "Occurrence(name='Soporte:1', fullPath='Root:1/Soporte:1')"
      - "MeshBody(name='malla_importada', isProxy=True)"
    """
    try:
        if ent is None:
            return "<None>"

        cls = ent.__class__.__name__

        # Intentar leer nombre
        try:
            name = getattr(ent, "name", None)
        except Exception:
            name = None

        # Es occurrence
        try:
            if isinstance(ent, adsk.fusion.Occurrence):
                fp = getattr(ent, "fullPathName", "")
                return f"Occurrence(name='{name}', fullPath='{fp}')"
        except Exception:
            pass

        # Es BRepBody
        try:
            from adsk.fusion import BRepBody
            if isinstance(ent, BRepBody):
                is_proxy = getattr(ent, "isProxy", False)
                return f"BRepBody(name='{name}', isProxy={is_proxy})"
        except Exception:
            pass

        # Es MeshBody
        try:
            from adsk.fusion import MeshBody
            if isinstance(ent, MeshBody):
                is_proxy = getattr(ent, "isProxy", False)
                return f"MeshBody(name='{name}', isProxy={is_proxy})"
        except Exception:
            pass

        # Fallback genérico
        if name is not None:
            return f"{cls}(name='{name}')"
        return cls
    except Exception as e:
        return f"<error _describe_entity: {e}>"


class RobotExporter:
    """Punto de entrada principal para exportar robots a URDF."""

    @staticmethod
    def export_robot(
        design: adsk.fusion.Design,
        output_dir: str = None,
        robot_name: str = "robot",
        mesh_mode: str = "very_low"
    ) -> Optional[str]:
        """
        Exporta un robot completo a URDF + DAE.

        Args:
            design      : Diseño de Fusion 360.
            output_dir  : Directorio de salida (si es None, usa Desktop/robot_name_urdf).
            robot_name  : Nombre del robot (se sanitiza).
            mesh_mode   : "very_low" (Very Low Quality Optimized) o "display" (DisplayMesh).

        Retorna:
            Ruta del archivo URDF creado, o None en caso de error.

        DEBUG:
          - Muestra la configuración inicial (output_dir, mesh_mode, flags de debug).
          - Muestra un resumen de links/joints devueltos por RobotBuilder.build().
          - Detecta links "colgados" (sin joints) y estructura padre→hijos.
          - Antes de escribir el URDF, se puede ver exactamente qué topología va a tener.
        """
        try:
            if design is None:
                _log("Design es None en export_robot", also_messagebox=True, level="ERROR")
                return None

            robot_name_sane = _sanitize(robot_name)

            # ---------------------------------------------
            #  Directorio de salida por defecto (Desktop)
            # ---------------------------------------------
            if output_dir is None:
                home = os.path.expanduser("~")
                output_dir = os.path.join(home, "Desktop", f"{robot_name_sane}_urdf")

            # DEBUG: configuración de exportación
            _log(
                "[EXPORT] Iniciando exportación URDF\n"
                f"         robot_name    = '{robot_name_sane}'\n"
                f"         output_dir    = '{output_dir}'\n"
                f"         mesh_mode     = '{mesh_mode}'\n"
                f"         _DEBUG_TRANSFORMS = {_DEBUG_TRANSFORMS}\n"
                f"         _DEBUG_MESH_TREE  = {_DEBUG_MESH_TREE}\n"
                f"         _DEBUG_LINK_JOINT = {_DEBUG_LINK_JOINT}",
                level="INFO"
            )

            # ---------------------------------------------
            #  Crear carpeta de salida y de mallas
            # ---------------------------------------------
            if not ensure_dir(output_dir):
                _log(
                    f"No se pudo crear directorio de salida: {output_dir}",
                    also_messagebox=True,
                    level="ERROR"
                )
                return None

            mesh_dir = os.path.join(output_dir, "meshes")
            if not ensure_dir(mesh_dir):
                _log(
                    f"No se pudo crear directorio de mallas: {mesh_dir}",
                    also_messagebox=True,
                    level="ERROR"
                )
                return None

            # ---------------------------------------------
            #  Construir el robot (links + joints)
            # ---------------------------------------------
            _log("Construyendo estructura del robot (RobotBuilder.build())...", level="INFO")
            builder = RobotBuilder(
                design=design,
                robot_name=robot_name_sane,
                output_dir=output_dir,
                mesh_mode=mesh_mode
            )

            links, joints = builder.build()

            if not links:
                _log(
                    "No se generaron links durante la construcción del robot.",
                    also_messagebox=True,
                    level="ERROR"
                )
                return None

            # ---------------------------------------------
            #  DEBUG: resumen de estructura links/joints
            # ---------------------------------------------
            if _DEBUG_TRANSFORMS or _DEBUG_LINK_JOINT or _DEBUG_MESH_TREE:
                try:
                    link_names = [lnk.get("name", "<unnamed>") for lnk in links]
                    joint_children = {j.get("child") for j in joints}
                    joint_parents = {j.get("parent") for j in joints}
                    all_joint_nodes = joint_children.union(joint_parents)

                    dangling_links = [
                        name for name in link_names
                        if name not in all_joint_nodes
                    ]

                    _log(
                        "[EXPORT][DEBUG] Resultado de RobotBuilder.build():\n"
                        f"   - links totales  = {len(links)}\n"
                        f"   - joints totales = {len(joints)}\n"
                        f"   - builder.single_link_no_joints = {builder.single_link_no_joints}\n"
                        f"   - builder.base_link_name        = {builder.base_link_name}",
                        level="DEBUG"
                    )

                    if dangling_links:
                        _log(
                            "[EXPORT][DEBUG] Links sin joints (potencialmente 'flotando' o conectados "
                            f"solo vía fixed a base_link): {dangling_links}",
                            level="DEBUG"
                        )

                    if _DEBUG_LINK_JOINT:
                        # Pequeño árbol padre → hijos para ver la topología
                        tree: Dict[str, List[str]] = {}
                        for j in joints:
                            p = j.get("parent", "<?>")
                            c = j.get("child", "<?>")
                            tree.setdefault(p, []).append(c)
                        _log("[EXPORT][DEBUG] Árbol padre → hijos de joints:", level="DEBUG")
                        for parent, children in tree.items():
                            _log(f"    {parent} -> {children}", level="DEBUG")

                except Exception as e:
                    _log(f"[EXPORT][DEBUG] Error al generar resumen de links/joints: {e}", level="DEBUG")

            # ---------------------------------------------
            #  Generar URDF
            # ---------------------------------------------
            _log("Generando archivo URDF...", level="INFO")
            urdf_path = os.path.join(output_dir, f"{robot_name_sane}.urdf")

            # Si builder.single_link_no_joints es True, dejamos base_link_name=None
            base_link = builder.base_link_name if not builder.single_link_no_joints else None

            if _DEBUG_TRANSFORMS:
                _log(
                    f"[EXPORT] Llamando a URDFWriter.write_urdf() con:\n"
                    f"         urdf_path  = '{urdf_path}'\n"
                    f"         base_link  = '{base_link}'",
                    level="DEBUG"
                )

            success = URDFWriter.write_urdf(
                filepath=urdf_path,
                robot_name=robot_name_sane,
                links=links,
                joints=joints,
                base_link_name=base_link
            )

            if success:
                _log(
                    f"Exportación completada exitosamente: {urdf_path}",
                    also_messagebox=True,
                    level="INFO"
                )
                return urdf_path
            else:
                _log(
                    "Error al generar archivo URDF (URDFWriter.write_urdf devolvió False)",
                    also_messagebox=True,
                    level="ERROR"
                )
                return None

        except Exception as e:
            error_msg = f"Error en RobotExporter.export_robot:\n{traceback.format_exc()}"
            _log(error_msg, also_messagebox=True, level="ERROR")
            return None






























