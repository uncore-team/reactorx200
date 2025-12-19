#!/usr/bin/env python

import argparse
import xml.etree.ElementTree as ET

def extract_materials_from_urdf(urdf_file):
    """Extrae materiales y texturas del archivo URDF."""
    urdf_tree = ET.parse(urdf_file)
    urdf_root = urdf_tree.getroot()

    materials = {}
    for material in urdf_root.findall("material"):
        name = material.attrib.get("name")
        texture = material.find("texture")
        color = material.find("color")

        texture_file = texture.attrib["filename"] if texture is not None else None
        rgba = color.attrib["rgba"] if color is not None else None

        materials[name] = {
            "texture": texture_file,
            "color": rgba
        }
    return materials

def extract_mesh_material_mapping(urdf_file):
    """Asocia mallas con materiales desde el archivo URDF."""
    urdf_tree = ET.parse(urdf_file)
    urdf_root = urdf_tree.getroot()

    mesh_material_map = {}
    for visual in urdf_root.findall("visual"):
        geometry = visual.find("geometry/mesh")
        material = visual.find("material")
        if geometry is not None and material is not None:
            mesh_filename = geometry.attrib.get("filename")
            material_name = material.attrib.get("name")
            if mesh_filename and material_name:
                mesh_material_map[mesh_filename] = material_name
    return mesh_material_map

def modify_mjcf_with_materials(mjcf_file, materials, urdf_file, output_file):
    """Modifica el archivo MJCF agregando materiales y texturas."""
    mjcf_tree = ET.parse(mjcf_file)
    mjcf_root = mjcf_tree.getroot()

    # Crear sección <asset> si no existe
    asset_section = mjcf_root.find("asset")
    if asset_section is None:
        asset_section = ET.SubElement(mjcf_root, "asset")

    # Agregar texturas y materiales al archivo MJCF
    for material_name, properties in materials.items():
        if properties.get("texture") or properties.get("color"):
            if properties.get("texture"):
                ET.SubElement(
                    asset_section,
                    "texture",
                    name=f"{material_name}_texture",
                    file=properties["texture"]
                )
            material_attributes = {"name": material_name}
            if properties.get("texture"):
                material_attributes["texture"] = f"{material_name}_texture"
            if properties.get("color"):
                material_attributes["rgba"] = properties["color"]
            ET.SubElement(asset_section, "material", **material_attributes)

    # Crear mapa de mallas a materiales desde el URDF
    mesh_material_map = extract_mesh_material_mapping(urdf_file)

    # Asociar materiales a los geoms según su mesh o nombre de material
    for geom in mjcf_root.findall("geom"):
        material_name = geom.attrib.get("material")
        if not material_name:
            mesh_name = geom.attrib.get("mesh")
            if mesh_name in mesh_material_map:
                geom.set("material", mesh_material_map[mesh_name])

    # Escribir el archivo MJCF modificado
    mjcf_tree.write(output_file, encoding="utf-8", xml_declaration=True)

def main():
    parser = argparse.ArgumentParser(description="Convierte URDF a MJCF con materiales.")
    parser.add_argument("-u", "--urdf", required=True, help="Ruta al archivo URDF de entrada.")
    parser.add_argument("-m", "--mjcf", required=True, help="Ruta al archivo MJCF original.")
    parser.add_argument("-o", "--output", required=True, help="Ruta al archivo MJCF de salida.")

    args = parser.parse_args()

    # Extraer materiales del URDF
    materials = extract_materials_from_urdf(args.urdf)

    # Modificar el MJCF con los materiales extraídos
    modify_mjcf_with_materials(args.mjcf, materials, args.urdf, args.output)
    print(f"Archivo MJCF actualizado guardado en {args.output}")

if __name__ == "__main__":
    main()
