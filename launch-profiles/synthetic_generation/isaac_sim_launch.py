import omni.kit.app
import omni.usd

omni.usd.get_context().open_stage("/robot/isaac_src/assets/scenes/cube-to-cup-task.usd")

# Start the simulation timeline on startup
timeline = omni.timeline.get_timeline_interface()
timeline.play()
