try:
    from rmf_fleet_msgs.msg import FleetState, RobotMode
    print("Import successful")
    print(f"RobotMode.MODE_IDLE: {RobotMode.MODE_IDLE}")
except ImportError as e:
    print(f"Import failed: {e}")
