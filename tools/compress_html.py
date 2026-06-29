import gzip
import os

def compress_to_header(input_file, output_header, variable_name="html_gz"):
    """
    Comprime un archivo HTML a GZIP y genera un header de C/C++
    """
    
    # Leer y comprimir
    with open(input_file, 'rb') as f:
        html_content = f.read()
    
    original_size = len(html_content)
    
    # Comprimir con máxima compresión
    compressed = gzip.compress(html_content, compresslevel=9)
    compressed_size = len(compressed)
    
    # Crear el archivo header
    with open(output_header, 'w', encoding='utf-8') as f:
        f.write(f"// Archivo comprimido desde: {input_file}\n")
        f.write(f"// Tamaño original: {original_size} bytes\n")
        f.write(f"// Tamaño comprimido: {compressed_size} bytes\n")
        f.write(f"// Compresión: {100 - (compressed_size * 100 // original_size)}%\n")
        f.write(f"// Generado automáticamente\n\n")
        f.write(f"#include <stdint.h>\n\n")
        f.write(f"const uint8_t {variable_name}[] = {{\n    ")
        
        # Escribir bytes en formato C
        for i, byte in enumerate(compressed):
            if i > 0 and i % 12 == 0:
                f.write("\n    ")
            f.write(f"0x{byte:02x}, ")
        
        f.write(f"\n}};\n\n")
        f.write(f"const unsigned int {variable_name}_len = {compressed_size};\n")
    
    print(f"✅ Header generado: {output_header}")
    print(f"📊 {original_size:,} -> {compressed_size:,} bytes ({100 - (compressed_size * 100 // original_size)}% compresión)")
    print(f"💾 Ahorro: {original_size - compressed_size:,} bytes")

if __name__ == "__main__":
    # Configuración
    INPUT_FILE = "index.html"
    OUTPUT_HEADER = "index_html_gz.h"
    VARIABLE_NAME = "index_html_gz"
    
    if not os.path.exists(INPUT_FILE):
        print(f"❌ Error: No se encuentra {INPUT_FILE}")
        print(f"   Asegúrate de que el archivo esté en: {os.path.abspath(INPUT_FILE)}")
        exit(1)
    
    compress_to_header(INPUT_FILE, OUTPUT_HEADER, VARIABLE_NAME)
    print("\n📁 Archivos creados:")
    print(f"   - {OUTPUT_HEADER}")