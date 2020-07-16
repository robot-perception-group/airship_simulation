# A representation of 3D position, 3D orientation and 3D velocity and in free space, composed of position and orientation.
# Point position (north,east,down)
# Point velocity (north,east,down)
# Quaternion orientation (x,y,z,w -- 0,0,0,1 = X(front)-> north, Y(right)-> east, Z(bottom))
# float64[100] covariance diagonal 10x10 matrix, column order: posN,posE,posD,velN,velE,velD,Qx,Qy,Qz,Qw
# Point rotation (roll,pitch,yaw)
# float64 thrust (power setting -1 <= thrust <= +1 , negative thrust=engine off)
# int32 flightmode (TODO: to be defined later)
# NOTE: To include complete covariance information, a float32[100] Covariance (10x10 matrix) would be needed as well. Transferring that with every update might limit bandwidth significantly.


Header header
geometry_msgs/Point position
geometry_msgs/Point velocity
geometry_msgs/Point acceleration
geometry_msgs/Quaternion orientation
float64[100] covariance
geometry_msgs/Point angVelocity
float64 thrust
int32 flightmode
geometry_msgs/Point POI
