
import rmf_adapter as adpt
import inspect

print("dir(adpt):", dir(adpt))
print("dir(adpt.Adapter):", dir(adpt.Adapter))

# Check for any make parameters
print("adpt.NodeOptions:", dir(adpt.NodeOptions))
print("adpt.FleetUpdateHandle:", dir(adpt.FleetUpdateHandle))
try:
    opts = adpt.NodeOptions()
    print("NodeOptions instance:", dir(opts))
except:
    print("Cannot instantiate NodeOptions")

print("adpt.init_rclcpp signature:", inspect.signature(adpt.init_rclcpp) if hasattr(adpt.init_rclcpp, "__signature__") else "No signature found")
