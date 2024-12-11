import math

from myarm_ai.robot_protocols import DummyBouncingRobot


def test_moves_between_reads() -> None:
    dummy = DummyBouncingRobot(
        parameters=DummyBouncingRobot.Parameters(
            joint_mins=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            joint_maxs=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            joint_speeds=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        )
    )

    first_reading = dummy.read_joints()
    second_reading = dummy.read_joints()
    third_reading = dummy.read_joints()

    assert first_reading != second_reading != third_reading

    # Sanity test first joint
    assert math.isclose(first_reading[0], 0.1)
    assert math.isclose(second_reading[0], 0.2)
    assert math.isclose(third_reading[0], 0.3)

    # Sanity test the last joint (which should have doubled backwards)
    assert math.isclose(first_reading[-1], 0.7)
    assert math.isclose(second_reading[-1], 1.4)
    assert math.isclose(third_reading[-1], 0.7)
