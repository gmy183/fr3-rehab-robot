from rehab_robot_control.rehab_moveit_topology import (
    discover_fr3_moveit_topology,
)


def test_moveit_package_discovered():
    topology = (
        discover_fr3_moveit_topology()
    )

    assert (
        topology.package_share.exists()
    )


def test_fr3_group_discovered():
    topology = (
        discover_fr3_moveit_topology()
    )

    assert topology.planning_group


def test_fr3_chain_discovered():
    topology = (
        discover_fr3_moveit_topology()
    )

    assert (
        topology.base_link
        == "fr3_link0"
    )

    assert (
        topology.tip_link
        == "fr3_link8"
    )


def test_follow_joint_trajectory_controller():
    topology = (
        discover_fr3_moveit_topology()
    )

    assert (
        topology.trajectory_controller
        == "fr3_arm_controller"
    )

    assert (
        topology.trajectory_action_ns
        == "follow_joint_trajectory"
    )
