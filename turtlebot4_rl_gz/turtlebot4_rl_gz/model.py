"""Pure SDF assets used by the Gazebo adapter."""


def goal_marker_sdf() -> str:
    """Return a static green cylinder used only to visualize the current goal."""
    return (
        "<sdf version='1.9'><model name='rl_goal'><static>true</static>"
        "<link name='link'><visual name='visual'><geometry><cylinder>"
        '<radius>0.12</radius><length>0.02</length></cylinder></geometry>'
        '<material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse>'
        '</material></visual></link></model></sdf>'
    )
