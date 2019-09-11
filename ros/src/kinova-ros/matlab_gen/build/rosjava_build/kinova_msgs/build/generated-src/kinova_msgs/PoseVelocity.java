package kinova_msgs;

public interface PoseVelocity extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/PoseVelocity";
  static final java.lang.String _DEFINITION = "float32 twist_linear_x\nfloat32 twist_linear_y\nfloat32 twist_linear_z\nfloat32 twist_angular_x\nfloat32 twist_angular_y\nfloat32 twist_angular_z\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getTwistLinearX();
  void setTwistLinearX(float value);
  float getTwistLinearY();
  void setTwistLinearY(float value);
  float getTwistLinearZ();
  void setTwistLinearZ(float value);
  float getTwistAngularX();
  void setTwistAngularX(float value);
  float getTwistAngularY();
  void setTwistAngularY(float value);
  float getTwistAngularZ();
  void setTwistAngularZ(float value);
}
