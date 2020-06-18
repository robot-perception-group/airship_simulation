package mav_msgs;

public interface AttitudeThrust extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mav_msgs/AttitudeThrust";
  static final java.lang.String _DEFINITION = "Header header\n\ngeometry_msgs/Quaternion attitude    # Attitude expressed in the header/frame_id frame.\ngeometry_msgs/Vector3 thrust         # Thrust [N] expressed in the body frame.\n                                     # For a fixed-wing, usually the x-component is used.\n                                     # For a multi-rotor, usually the z-component is used.\n                                     # Set all un-used components to 0.\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Quaternion getAttitude();
  void setAttitude(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getThrust();
  void setThrust(geometry_msgs.Vector3 value);
}
