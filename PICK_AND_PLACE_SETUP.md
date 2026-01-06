# Pick-and-Place Implementation Guide

## ✅ Implementation Complete!

The simplified pick-and-place system has been implemented with the following components:

### Unity Scripts (C#)
1. **SimpleGrasp.cs** (Enhanced) - Located at: `/mnt/c/Users/user/Documents/HRI_digital_twin_factory_setting/Assets/Scripts/SimpleGrasp.cs`
   - Added events: `OnGraspSuccess`, `OnGraspFailure`, `OnReleaseComplete`
   - Added properties: `IsGrasping`, `GraspedObject`, `GraspDuration`
   - Enhanced timeout detection and logging

2. **GraspEventPublisher.cs** (New) - Located at: `/mnt/c/Users/user/Documents/HRI_digital_twin_factory_setting/Assets/Scripts/GraspEventPublisher.cs`
   - Bridges Unity grasp events to ROS2 topics
   - Publishes to `/unity/grasp_success` (Bool) and `/unity/grasp_feedback` (String)
   - Attach to the same GameObject as SimpleGrasp.cs (gripper_base_link)

3. **PickAndPlaceManager.cs** (New) - Located at: `/mnt/c/Users/user/Documents/HRI_digital_twin_factory_setting/Assets/Scripts/PickAndPlaceManager.cs`
   - Scene-level state machine for pick-and-place operations
   - Tracks metrics: success rate, cycle time, attempts
   - Provides scene reset functionality for repeated trials

### ROS2 Script (Python)
1. **pick_and_place.py** (Enhanced) - Located at: `/home/user/ar4_ros_driver/annin_ar4_driver/scripts/pick_and_place.py`
   - Subscribes to Unity feedback topics
   - Waits for grasp confirmation before continuing
   - Enhanced logging with step-by-step progress
   - Configurable timeouts and physics delays

---

## Setup Instructions

### Step 1: Unity Setup

1. **Open your Unity project** (HRI_digital_twin_factory_setting)

2. **Locate gripper_base_link GameObject** in HRI_scene1.unity:
   - This should already have SimpleGrasp.cs attached
   - If not, find the gripper end-effector GameObject

3. **Add GraspEventPublisher.cs**:
   - Select the gripper_base_link GameObject
   - In Inspector: "Add Component" → "Grasp Event Publisher"
   - It will automatically find the SimpleGrasp component

4. **Create a scene-level manager**:
   - Create an empty GameObject named "PickAndPlaceManager"
   - Add component: "Pick And Place Manager"
   - Assign references:
     - `Grasp Controller`: Drag the gripper_base_link (with SimpleGrasp) here
     - `Pick Zone`: Create an empty GameObject as a marker for pick location
     - `Place Zone`: Create an empty GameObject as a marker for place location

5. **Create a test object**:
   - Create a Cube: GameObject → 3D Object → Cube
   - Scale it to 0.05 × 0.05 × 0.05 (5cm cube)
   - Add tag "Grabbable" (Create tag if it doesn't exist)
   - Add Rigidbody component:
     - Mass: 0.5
     - Drag: 0.1
     - Collision Detection: Continuous
   - Position it near the pick zone

6. **Save the scene**

### Step 2: ROS2 Setup

1. **Make the script executable**:
```bash
chmod +x /home/user/ar4_ros_driver/annin_ar4_driver/scripts/pick_and_place.py
```

2. **Rebuild the workspace** (no changes to CMakeLists needed, just to be safe):
```bash
cd ~/ar4_ros_driver
colcon build
source install/setup.bash
```

---

## Testing the System

### Terminal Setup

**Terminal 1: Unity Bridge**
```bash
cd ~/ar4_ros_driver
source install/setup.bash
ros2 launch annin_ar4_driver unity_bridge.launch.py
```

**Terminal 2: MoveIt (if using IK)**
```bash
source install/setup.bash
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

**Terminal 3: Pick-and-Place Controller**
```bash
source install/setup.bash
ros2 run annin_ar4_driver pick_and_place.py
```

**Unity**: Open HRI_scene1.unity and press Play

---

## Expected Behavior

### ROS2 Console:
```
[pick_and_place]: === Starting Pick-and-Place Scenario ===
[pick_and_place]: [1/9] Moving to home position...
[pick_and_place]: [2/9] Opening gripper...
[pick_and_place]: [3/9] Approaching pick location...
[pick_and_place]: [4/9] Attempting grasp...
[pick_and_place]: Waiting for Unity grasp confirmation (timeout: 5.0s)...
[pick_and_place]: ✓ Unity confirmed grasp success
[pick_and_place]: [5/9] Lifting object...
...
[pick_and_place]: === Pick-and-Place Scenario Complete! ===
```

### Unity Console:
```
[SimpleGrasp] Grasped Cube at 12.34s
[GraspEventPublisher] Published grasp success: Cube
[PickAndPlaceManager] State: Picking → PickVerification
[PickAndPlaceManager] Grasp success! Success rate: 100.0% (1/1)
```

---

## Troubleshooting

### Issue: "No Unity feedback received"
- **Check**: Is Unity running and connected to ROS2?
- **Check**: Is GraspEventPublisher attached to gripper_base_link?
- **Fix**: Monitor topics:
  ```bash
  ros2 topic list | grep unity
  ros2 topic echo /unity/grasp_feedback
  ```

### Issue: "Grasp timeout"
- **Check**: Is the test object tagged as "Grabbable"?
- **Check**: Is the object within 5cm of gripper?
- **Check**: SimpleGrasp `closeThreshold` setting (default 0.02m)
- **Fix**: Increase timeout in pick_and_place.py:
  ```python
  self.grasp_confirmation_timeout = 10.0  # Increase to 10 seconds
  ```

### Issue: "Gripper not closing properly"
- **Check**: Gripper ArticulationBody reference in SimpleGrasp
- **Check**: Joint limits in Unity (should be 0.0 to 0.04)
- **Fix**: Verify in Unity Inspector that `gripperJoint` is assigned

### Issue: "Object falls through gripper"
- **Check**: Object has Rigidbody component
- **Check**: Collision detection is set to "Continuous"
- **Fix**: Increase physics settle delay:
  ```python
  self.physics_settle_delay = 1.0  # Increase to 1 second
  ```

---

## Customization

### Change Pick/Place Positions

Edit in `pick_and_place.py`:
```python
# Line 142 - Pick location
pick_pose = [1.57, -0.5, -1.0, 0.0, 0.0, 0.0]

# Line 166 - Place location
place_pose = [-1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### Disable Unity Feedback (Fallback Mode)

In `pick_and_place.py`, line 42:
```python
self.use_unity_feedback = False  # Set to False
```

### Adjust Grasp Detection Range

In Unity, SimpleGrasp component:
- `Close Threshold`: How closed gripper must be (default: 0.02m)
- `Grasp Timeout`: How long to wait for object (default: 2.0s)

In `AttemptGrasp()` method (line 57):
```csharp
Collider[] colliders = Physics.OverlapSphere(transform.position, 0.05f); // Change 0.05f to adjust range
```

---

## Next Steps

1. **Test basic pick-and-place**: Run the scenario and verify grasp detection
2. **Tune positions**: Adjust joint angles to match your workspace
3. **Add multiple objects**: Test with different object sizes
4. **Data collection**: Use PickAndPlaceManager metrics for research
5. **EEG integration**: Replace manual execution with EEG-triggered commands

---

## ROS2 Topics

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/unity/grasp_success` | std_msgs/Bool | Unity → ROS2 | Grasp success/failure flag |
| `/unity/grasp_feedback` | std_msgs/String | Unity → ROS2 | Detailed event messages |
| `/joint_states` | sensor_msgs/JointState | ROS2 ↔ Unity | Current robot state |
| `/gripper_controller/gripper_cmd` | control_msgs/GripperCommand | ROS2 → Robot | Gripper control |
| `/joint_trajectory_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | ROS2 → Robot | Arm control |

---

## Success Criteria

- ✅ Object detected when gripper closes
- ✅ Unity publishes grasp events to ROS2
- ✅ ROS2 receives confirmation before lifting
- ✅ Object stays attached during transport
- ✅ Object released at place location
- ✅ Cycle completes in < 30 seconds
- ✅ Success rate >= 90% over 10 trials

---

## Files Modified/Created

**Modified:**
- `/mnt/c/Users/user/Documents/HRI_digital_twin_factory_setting/Assets/Scripts/SimpleGrasp.cs`
- `/home/user/ar4_ros_driver/annin_ar4_driver/scripts/pick_and_place.py`

**Created:**
- `/mnt/c/Users/user/Documents/HRI_digital_twin_factory_setting/Assets/Scripts/GraspEventPublisher.cs`
- `/mnt/c/Users/user/Documents/HRI_digital_twin_factory_setting/Assets/Scripts/PickAndPlaceManager.cs`
- `/home/user/ar4_ros_driver/PICK_AND_PLACE_SETUP.md` (this file)

---

## Contact & Support

For issues or questions:
1. Check the plan file: `/home/user/.claude/plans/swift-moseying-origami.md`
2. Review ROS2 logs: `ros2 topic echo /unity/grasp_feedback`
3. Check Unity console for SimpleGrasp debug messages
