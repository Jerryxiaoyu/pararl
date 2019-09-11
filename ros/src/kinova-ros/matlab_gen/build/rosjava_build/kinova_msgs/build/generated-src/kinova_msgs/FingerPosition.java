package kinova_msgs;

public interface FingerPosition extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/FingerPosition";
  static final java.lang.String _DEFINITION = "float32 finger1\nfloat32 finger2\nfloat32 finger3\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  float getFinger1();
  void setFinger1(float value);
  float getFinger2();
  void setFinger2(float value);
  float getFinger3();
  void setFinger3(float value);
}
