from pathlib import Path

from setuptools import setup


PACKAGE_NAME = "vision"


def model_data_files() -> list[tuple[str, list[str]]]:
    data_files: list[tuple[str, list[str]]] = [
        (f"share/{PACKAGE_NAME}/data", ["data/README.md"]),
    ]

    for model_dir in sorted(Path("data").glob("yolo26n_ncnn_imgsz_*")):
        files = [
            str(model_dir / "metadata.yaml"),
            str(model_dir / "model.ncnn.bin"),
            str(model_dir / "model.ncnn.param"),
        ]
        if all(Path(file_path).is_file() for file_path in files):
            data_files.append((f"share/{PACKAGE_NAME}/data/{model_dir.name}", files))

    return data_files


def launch_files() -> list[tuple[str, list[str]]]:
    launch_paths = sorted(str(path) for path in Path("launch").glob("*.launch.py"))
    if not launch_paths:
        return []
    return [(f"share/{PACKAGE_NAME}/launch", launch_paths)]


setup(
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        *launch_files(),
        *model_data_files(),
    ],
)
