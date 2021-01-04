### score_paths.py
- actual "logic"

### main.py
- Mostly ros filler
- dummy global path generator
- truncates global path 

### generate_markers.py
- Helper to create the visualization_msgs::Marker for rviz

### helpers.py
- Some data types defined

### main.rviz
- rviz settings

Everything else is either dummy functions or ros filler

```bash
colcon build --symlink-install --packages-up-to path_score
ros2 launch path_score main.launch.py
```

requires numpy