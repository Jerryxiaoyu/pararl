package kinova_msgs;

public interface JointTorque extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/JointTorque";
  static final java.lang.String _DEFINITION = "float32 joint1\nfloat32 joint2\nfloat32 joint3\nfloat32 joint4\nfloat32 joint5\nfloat32 joint6\nfloat32 joint7\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getJoint1();
  void setJoint1(float value);
  float getJoint2();
  void setJoint2(float value);
  float getJoint3();
  void setJoint3(float value);
  float getJoint4();
  void setJoint4(float value);
  float getJoint5();
  void setJoint5(float value);
  float getJoint6();
  void setJoint6(float value);
  float getJoint7();
  void setJoint7(float value);
}
