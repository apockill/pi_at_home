from collections.abc import Generator
from unittest import mock

import pytest
from myarm_ai.robot_protocols import dummy_copycat_robot
from myarm_ai.robot_protocols.dummy_copycat_robot import DummyCopycatRobot


@pytest.fixture()
def mock_time() -> Generator[mock.MagicMock, None, None]:
    """Mock the 'time' module in the dummy_copycat_robot module context"""
    with mock.patch.object(dummy_copycat_robot, "time") as patched_time:
        # We'll let test code set patched_time.time.return_value as needed
        yield patched_time


def test_robot_initialization() -> None:
    """Sanity check"""
    # If your Parameters default is 7 joints, we'll just test that
    robot = DummyCopycatRobot()

    assert len(robot._motors) == robot.params.n_joints
    for m in robot._motors:
        assert m.position == 0.0
        assert m.speed == robot.params.speed


def test_write_joints_mismatch_error() -> None:
    """Ensure write_joints raises ValueError if the number of joints is incorrect"""

    robot = DummyCopycatRobot()

    # By default, n_joints=7, so giving only 2 positions should fail
    with pytest.raises(ValueError):
        robot.write_joints([1.0, 2.0], speed=1.0)


@pytest.mark.parametrize(
    ("targets", "speed", "dt", "expected_positions"),
    [
        # Move from 0 -> [1.0, 2.0] with speed=1.0 for 0s => no movement
        ([1.0, 2.0], 1.0, 0.0, [0.0, 0.0]),
        # Move from 0 -> [1.0, 2.0] with speed=1.0 for 1s => partial move:
        #   joint0 should reach 1.0 exactly, joint1 moves to 1.0 out of 2.0
        ([1.0, 2.0], 1.0, 1.0, [1.0, 1.0]),
        # Move from 0 -> [1.0, 2.0] with speed=1.0 for 2s => both fully reached
        ([1.0, 2.0], 1.0, 2.0, [1.0, 2.0]),
        # Negative direction: 0 -> [-2.0, -4.0] with speed=2.0 for 1s => partial
        #   -2.0 is reached in 1s, -4.0 requires 2s. After 1s, joint1 at -2.0
        ([-2.0, -4.0], 2.0, 1.0, [-2.0, -2.0]),
        # Negative direction: 0 -> [-2.0, -4.0] with speed=2.0 for 2s => fully reached
        ([-2.0, -4.0], 2.0, 2.0, [-2.0, -4.0]),
    ],
)
def test_robot_moves_joints(
    mock_time: mock.MagicMock,
    targets: list[float],
    speed: float,
    dt: float,
    expected_positions: list[float],
) -> None:
    """Verifies that the robot (with 2 joints) moves as expected after dt seconds."""
    # Set an arbitrary baseline time
    mock_time.return_value = 1000.0

    # Create a robot with 2 joints to match our test data
    robot = DummyCopycatRobot(
        parameters=DummyCopycatRobot.Parameters(n_joints=2, speed=speed)
    )

    # Write new targets
    robot.write_joints(targets, speed)

    # 1st read sets last_update but no time has passed => positions remain at 0.0
    _ = robot.read_joints()

    # Advance mock time by dt
    mock_time.return_value += dt

    # 2nd read => positions update
    positions_after_dt = robot.read_joints()

    # Compare to expected
    for pos, expected in zip(positions_after_dt, expected_positions, strict=False):
        assert pos == pytest.approx(expected, abs=1e-4)
