package kinova_msgs;

public interface ArmJointAnglesGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/ArmJointAnglesGoal";
  static final java.lang.String _DEFINITION = "# Goal\nJointAngles angles\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.JointAngles getAngles();
  void setAngles(kinova_msgs.JointAngles value);
}
