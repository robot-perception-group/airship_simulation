package mav_msgs;

public interface Actuators extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mav_msgs/Actuators";
  static final java.lang.String _DEFINITION = "Header header\n\n# This message defines lowest level commands to be sent to the actuator(s). \n\nfloat64[] angles             # Angle of the actuator in [rad]. \n                             # E.g. servo angle of a control surface(not angle of the surface!), orientation-angle of a thruster.      \nfloat64[] angular_velocities # Angular velocities of the actuator in [rad/s].\n                             # E.g. \"rpm\" of rotors, propellers, thrusters \nfloat64[] normalized         # Everything that does not fit the above, normalized between [-1 ... 1].";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double[] getAngles();
  void setAngles(double[] value);
  double[] getAngularVelocities();
  void setAngularVelocities(double[] value);
  double[] getNormalized();
  void setNormalized(double[] value);
}
