package kinova_msgs;

public interface SetFingersPositionGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/SetFingersPositionGoal";
  static final java.lang.String _DEFINITION = "# Goal\nFingerPosition fingers\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.FingerPosition getFingers();
  void setFingers(kinova_msgs.FingerPosition value);
}
