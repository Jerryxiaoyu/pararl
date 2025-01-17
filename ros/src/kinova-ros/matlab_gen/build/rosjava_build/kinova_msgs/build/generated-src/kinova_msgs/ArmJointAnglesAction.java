package kinova_msgs;

public interface ArmJointAnglesAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/ArmJointAnglesAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nkinova_msgs/ArmJointAnglesActionGoal action_goal\nkinova_msgs/ArmJointAnglesActionResult action_result\nkinova_msgs/ArmJointAnglesActionFeedback action_feedback\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.ArmJointAnglesActionGoal getActionGoal();
  void setActionGoal(kinova_msgs.ArmJointAnglesActionGoal value);
  kinova_msgs.ArmJointAnglesActionResult getActionResult();
  void setActionResult(kinova_msgs.ArmJointAnglesActionResult value);
  kinova_msgs.ArmJointAnglesActionFeedback getActionFeedback();
  void setActionFeedback(kinova_msgs.ArmJointAnglesActionFeedback value);
}
