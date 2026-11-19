"""Convierte el .ino en C++ para la PC: prototipos al principio (como hace Arduino)
y los valores por defecto de los parámetros solo en el prototipo."""
import re, sys
source = open(sys.argv[1]).read()
pattern = re.compile(r"^((?:const\s+)?[A-Za-z_][\w<>\*&\s]*?[\s\*&])(\w+)\s*\(([^;{}()]*(?:\([^)]*\)[^;{}()]*)*)\)\s*\{", re.M)
protos = []
def strip_defaults(params):
    return re.sub(r"\s*=\s*[^,]+", "", params)
def replace(m):
    ret, name, params = m.group(1), m.group(2), m.group(3)
    if name in ("if", "for", "while", "switch") or ret.strip() in ("else", "return"):
        return m.group(0)
    protos.append(f"{ret.strip()} {name}({' '.join(params.split())});")
    return f"{ret}{name}({strip_defaults(params)}) {{"
first = pattern.search(source)
while first and first.group(2) in ("if", "for", "while", "switch"):
    first = pattern.search(source, first.end())
body = pattern.sub(replace, source)
# Como Arduino: los prototipos van justo antes de la primera funcion.
marker = body.index(first.group(0).split("(")[0])
print(body[:marker] + "\n".join(protos) + "\n" + body[marker:])
