import time
from collections.abc import Generator
from unittest.mock import MagicMock

import pytest
from myarm_ai.robot_protocols import (
    BaseRobotProtocol,
    DummyCopycatRobot,
    ThreadedReaderWriterRobot,
)
from node_helpers.timing import TestingTimeout


@pytest.fixture()
def mock_robot():
    """Create a mock for the underlying BaseRobotProtocol."""
    mock = MagicMock(spec=BaseRobotProtocol)
    mock.read_joints.return_value = [0.0, 0.0, 0.0]
    return mock


@pytest.fixture()
def threaded_robot(
    mock_robot: MagicMock,
) -> Generator[ThreadedReaderWriterRobot, None, None]:
    robot = ThreadedReaderWriterRobot(
        parameters=ThreadedReaderWriterRobot.Parameters(
            robot_protocol=DummyCopycatRobot
        )
    )
    robot._robot = mock_robot
    yield robot

    robot.close()
    assert not robot._thread.is_alive()


def test_read_joints_returns_cached_values(threaded_robot: ThreadedReaderWriterRobot):
    """Ensure read_joints quickly returns the cached last-known joints."""
    with threaded_robot._handle_lock:
        threaded_robot._robot.read_joints.reset_mock()
        initial_read = threaded_robot.read_joints()
        assert initial_read == [0.0, 0.0, 0.0]
        threaded_robot._robot.read_joints.assert_not_called()  # Because the separate thread does the actual read

    # Wait a bit to let the background thread do at least one read call.
    timeout = TestingTimeout(0.1)
    while timeout and threaded_robot._robot.read_joints.call_count == 0:
        time.sleep(0.01)

    # The background thread calls read_joints, so mock should have been called.
    assert threaded_robot._robot.read_joints.call_count > 0


def test_write_joints_sets_last_write_request(
    threaded_robot: ThreadedReaderWriterRobot,
):
    """Check that calling write_joints caches the request."""
    with threaded_robot._handle_lock:
        threaded_robot._robot.write_joints.reset_mock()
        threaded_robot.write_joints([1.0, 2.0, 3.0], speed=0.5)
        # Check that we haven't immediately written to the robot
        threaded_robot._robot.write_joints.assert_not_called()
        assert threaded_robot._last_write_request == ([1.0, 2.0, 3.0], 0.5)

    timeout = TestingTimeout(0.1)
    while timeout and threaded_robot._robot.write_joints.call_count == 0:
        time.sleep(0.01)

    threaded_robot._robot.write_joints.assert_called_once()
    threaded_robot._robot.write_joints.assert_called_once_with([1.0, 2.0, 3.0], 0.5)


def test_thread_avoids_write_if_joints_same(threaded_robot: ThreadedReaderWriterRobot):
    """
    If the new write_joints request is the same as the last read,
    the background thread won't call write_joints.
    """
    # By default, mock_robot.read_joints() -> [0.0, 0.0, 0.0]
    # So let's "wait" for at least one read
    timeout = TestingTimeout(0.1)
    while timeout and threaded_robot._robot.read_joints.call_count == 0:
        time.sleep(0.01)

    # Now the last known read is [0.0, 0.0, 0.0].
    # If we request a write to the same [0.0, 0.0, 0.0], it should skip.
    threaded_robot.write_joints([0.0, 0.0, 0.0], 1.0)
    threaded_robot._robot.write_joints.assert_not_called()


def test_close_joins_thread(threaded_robot):
    threaded_robot.close()

    assert not threaded_robot._thread.is_alive()
