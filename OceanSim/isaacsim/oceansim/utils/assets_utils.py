from pathlib import Path


def get_oceansim_assets_path() -> str:
    return str(Path(__file__).resolve().parent.parent.parent.parent / "assets")