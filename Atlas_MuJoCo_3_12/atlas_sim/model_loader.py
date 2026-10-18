"""Read paths with Python and compile using an in-memory MuJoCo VFS."""
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET

import mujoco

DEFAULT_MODELS = Path(__file__).resolve().parents[1] / 'models'


def load_atlas_model(model_dir=None):
    """Keep native MuJoCo file I/O away from physical Unicode paths.

    The packaged scene has a single XML file and mesh assets. Only Python
    opens physical paths; the compiler receives XML text and relative VFS keys.
    """
    directory = Path(model_dir) if model_dir is not None else DEFAULT_MODELS
    xml_path = directory / 'atlas.xml'
    if not xml_path.is_file():
        raise FileNotFoundError(
            f'未找到模型文件：{xml_path}\n'
            '请完整解压工程；models/atlas.xml 应与 models/meshes/ 网格目录同时存在。'
        )
    xml = xml_path.read_text(encoding='utf-8-sig')
    scene = ET.fromstring(xml)
    compiler = scene.find('compiler')
    meshdir = compiler.get('meshdir', '') if compiler is not None else ''
    files = {}
    missing = []
    for mesh in scene.findall('./asset/mesh'):
        filename = mesh.get('file')
        if not filename:
            continue
        key = str(PurePosixPath(meshdir.replace('\\', '/')) / filename.replace('\\', '/'))
        path = directory.joinpath(*PurePosixPath(key).parts)
        if not path.is_file():
            missing.append(str(path))
        else:
            files[key] = path.read_bytes()
    if missing:
        raise FileNotFoundError(
            '缺少 Atlas 网格文件：\n' + '\n'.join(missing) +
            '\n请从完整工程包恢复 models/meshes/，不要只复制 XML 或 Python 文件。'
        )
    with mujoco.MjVfs() as vfs:
        for key, content in files.items():
            vfs[key] = content
        return mujoco.MjModel.from_xml_string(xml, vfs=vfs)
