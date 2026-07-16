import xml.etree.ElementTree as ET

import pytest

from turtlebot4_rl_gz.config import GazeboWorldConfig
from turtlebot4_rl_gz.model import goal_marker_sdf


def test_goal_marker_is_valid_static_sdf() -> None:
    root = ET.fromstring(goal_marker_sdf())
    assert root.tag == 'sdf'
    assert root.findtext('./model/static') == 'true'
    assert root.find('./model/link/visual/geometry/cylinder') is not None


def test_entity_names_and_goal_bounds_are_validated() -> None:
    with pytest.raises(ValueError, match='unsupported characters'):
        GazeboWorldConfig(world_name='../unsafe')
    with pytest.raises(ValueError, match='goal bounds'):
        GazeboWorldConfig(goal_min_x=2.0, goal_max_x=1.0)
