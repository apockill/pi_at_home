DEFAULT_MYARM_FEATURES = {
    "action": {
        "dtype": "float32",
        "shape": [
            7
        ],
        "names": [
            "main_joint1",
            "main_joint2",
            "main_joint3",
            "main_joint4",
            "main_joint5",
            "main_joint6",
            "main_gripper"
        ]
    },
    "observation.state": {
        "dtype": "float32",
        "shape": [
            7
        ],
        "names": [
            "main_joint1",
            "main_joint2",
            "main_joint3",
            "main_joint4",
            "main_joint5",
            "main_joint6",
            "main_gripper"
        ]
    },
    "observation.images.wrist": {
        "dtype": "video",
        "shape": [
            240,
            320,
            3
        ],
        "names": [
            "height",
            "width",
            "channels"
        ],
        "info": {
            "video.fps": 30.0,
            "video.height": 240,
            "video.width": 320,
            "video.channels": 3,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": True,
            "has_audio": True
        }
    },
    "observation.images.top": {
        "dtype": "video",
        "shape": [
            240,
            320,
            3
        ],
        "names": [
            "height",
            "width",
            "channels"
        ],
        "info": {
            "video.fps": 30.0,
            "video.height": 240,
            "video.width": 320,
            "video.channels": 3,
            "video.codec": "av1",
            "video.pix_fmt": "yuv420p",
            "video.is_depth_map": True,
            "has_audio": True
        }
    },
}