package kinova_msgs;

public interface CartesianForce extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/CartesianForce";
  static final java.lang.String _DEFINITION = "float32 force_x\nfloat32 force_y\nfloat32 force_z\nfloat32 torque_x\nfloat32 torque_y\nfloat32 torque_z\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getForceX();
  void setForceX(float value);
  float getForceY();
  void setForceY(float value);
  float getForceZ();
  void setForceZ(float value);
  float getTorqueX();
  void setTorqueX(float value);
  float getTorqueY();
  void setTorqueY(float value);
  float getTorqueZ();
  void setTorqueZ(float value);
}
