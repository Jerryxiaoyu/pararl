package kinova_msgs;

public interface ArmJointAnglesFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/ArmJointAnglesFeedback";
  static final java.lang.String _DEFINITION = "# Feedback\nJointAngles angles";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.JointAngles getAngles();
  void setAngles(kinova_msgs.JointAngles value);
}
