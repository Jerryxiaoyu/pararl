package kinova_msgs;

public interface KinovaPose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/KinovaPose";
  static final java.lang.String _DEFINITION = "float32 X\nfloat32 Y\nfloat32 Z\nfloat32 ThetaX\nfloat32 ThetaY\nfloat32 ThetaZ";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getX();
  void setX(float value);
  float getY();
  void setY(float value);
  float getZ();
  void setZ(float value);
  float getThetaX();
  void setThetaX(float value);
  float getThetaY();
  void setThetaY(float value);
  float getThetaZ();
  void setThetaZ(float value);
}
