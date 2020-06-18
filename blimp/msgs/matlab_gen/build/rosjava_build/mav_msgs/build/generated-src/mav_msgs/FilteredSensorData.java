package mav_msgs;

public interface FilteredSensorData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mav_msgs/FilteredSensorData";
  static final java.lang.String _DEFINITION = "Header header\n\ngeometry_msgs/Vector3 accelerometer\t# acceleration in vehicle frame [m/s^2]\ngeometry_msgs/Vector3 gyroscope     # rotational velocity in vehicle frame [rad/s]\ngeometry_msgs/Vector3 magnetometer  # Magnetometer measurements in vehicle frame [uT]\nfloat64 barometer                   # Height from barometer relative to start-up point [m]\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Vector3 getAccelerometer();
  void setAccelerometer(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getGyroscope();
  void setGyroscope(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getMagnetometer();
  void setMagnetometer(geometry_msgs.Vector3 value);
  double getBarometer();
  void setBarometer(double value);
}
