package mav_msgs;

public interface RateThrust extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mav_msgs/RateThrust";
  static final java.lang.String _DEFINITION = "Header header\n\n# We use the coordinate frames with the following convention:\n#   x: forward\n#   y: left\n#   z: up\n\ngeometry_msgs/Vector3 angular_rates  # Roll-, pitch-, yaw-rate around body axes [rad/s]\ngeometry_msgs/Vector3 thrust         # Thrust [N] expressed in the body frame.\n                                     # For a fixed-wing, usually the x-component is used.\n                                     # For a multi-rotor, usually the z-component is used.\n                                     # Set all un-used components to 0.\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getAngularRates();
  void setAngularRates(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getThrust();
  void setThrust(geometry_msgs.Vector3 value);
}
