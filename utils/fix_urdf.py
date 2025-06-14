#!/usr/bin/env python

import xml.etree.ElementTree as ET
import argparse

def add_unique_names(input_file, output_file):
    # Cargar el URDF
    tree = ET.parse(input_file)
    root = tree.getroot()

    # Contadores para nombres únicos
    collision_counter = 1
    visual_counter = 1

    # Iterar sobre todos los links
    for link in root.findall(".//link"):
        # Procesar collisions
        for collision in link.findall("collision"):
            if "name" not in collision.attrib:
                collision.attrib["name"] = f"collision_{collision_counter}"
                collision_counter += 1

        # Procesar visuals
        for visual in link.findall("visual"):
            if "name" not in visual.attrib:
                visual.attrib["name"] = f"visual_{visual_counter}"
                visual_counter += 1

    # Guardar el URDF actualizado
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"Archivo URDF corregido guardado en: {output_file}")

if __name__ == "__main__":
    # Configurar el analizador de argumentos
    parser = argparse.ArgumentParser(description="Añadir nombres únicos a geometrías en un archivo URDF.")
    parser.add_argument("input_file", help="Nombre del archivo URDF de entrada")
    parser.add_argument("output_file", help="Nombre del archivo URDF de salida")

    # Parsear los argumentos
    args = parser.parse_args()

    # Ejecutar la función principal
    add_unique_names(args.input_file, args.output_file)

