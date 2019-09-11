package kinova_msgs;

public interface SetFingersPositionResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/SetFingersPositionResult";
  static final java.lang.String _DEFINITION = "# Result\nFingerPosition fingers\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.FingerPosition getFingers();
  void setFingers(kinova_msgs.FingerPosition value);
}
