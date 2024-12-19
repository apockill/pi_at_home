import omni.kit.app
import omni.usd

omni.usd.get_context().open_stage(
    "/robot/launch-profile/isaac-sim-assets/main-scene.usd"
)

# Start the simulation timeline on startup
timeline = omni.timeline.get_timeline_interface()
timeline.play()
