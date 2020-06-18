package mav_msgs;

public interface GpsWaypoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mav_msgs/GpsWaypoint";
  static final java.lang.String _DEFINITION = "Header header\n\nfloat64 latitude     # latitude in degree\nfloat64 longitude    # longitude in degree\nfloat64 altitude     # above start-up point\nfloat64 heading      # GPS heading\nfloat64 maxSpeed     # maximum approach speed\nfloat64 maxAcc       # maximum approach accelerations\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  double getLatitude();
  void setLatitude(double value);
  double getLongitude();
  void setLongitude(double value);
  double getAltitude();
  void setAltitude(double value);
  double getHeading();
  void setHeading(double value);
  double getMaxSpeed();
  void setMaxSpeed(double value);
  double getMaxAcc();
  void setMaxAcc(double value);
}
