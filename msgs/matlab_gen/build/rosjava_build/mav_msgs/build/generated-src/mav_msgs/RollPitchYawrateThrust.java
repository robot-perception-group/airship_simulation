package mav_msgs;

public interface RollPitchYawrateThrust extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mav_msgs/RollPitchYawrateThrust";
  static final java.lang.String _DEFINITION = "Header header\n\n# We use the coordinate frames with the following convention:\n#   x: forward\n#   y: left\n#   z: up\n\n# rotation convention (z-y\'-x\'\'):\n# yaw rotates around fixed frame\'s z axis\n# pitch rotates around new y-axis (y\')\n# roll rotates around new x-axis (x\'\')\n\n# This is a convenience-message to support that low-level (microcontroller-based) state\n# estimators may not have knowledge about the absolute yaw.\n# Roll- and pitch-angle should be specified in the header/frame_id frame\nfloat64 roll                   # Roll angle [rad]\nfloat64 pitch                  # Pitch angle  [rad]\nfloat64 yaw_rate               # Yaw rate around z-axis [rad/s]\n\ngeometry_msgs/Vector3 thrust   # Thrust [N] expressed in the body frame.\n                               # For a fixed-wing, usually the x-component is used.\n                               # For a multi-rotor, usually the z-component is used.\n                               # Set all un-used components to 0.\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getRoll();
  void setRoll(double value);
  double getPitch();
  void setPitch(double value);
  double getYawRate();
  void setYawRate(double value);
  geometry_msgs.Vector3 getThrust();
  void setThrust(geometry_msgs.Vector3 value);
}
