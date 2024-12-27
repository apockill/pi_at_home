from pathlib import Path

TIMESTEPS_FILENAME = "timesteps"  # usd
SCENE_FILENAME = "scene"  # usd
METADATA_FILENAME = "metadata.json"
DEFAULT_MESH_TEXTURES_DIR = Path("/robot/isaac_src/assets/textures/random")
DEFAULT_SKYBOX_TEXTURES_DIR = Path("/robot/isaac_src/assets/textures/skyboxes")
DEFAULT_RECORDINGS_DIR = Path("/robot/synthetic-output/recordings/")
RECORDER_ACTION_GRAPH_PATH = "/World/RecorderActionGraphs"
BOUND = tuple[float, float, float]
