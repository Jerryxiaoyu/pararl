package kinova_msgs;

public interface SetFingersPositionAction extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/SetFingersPositionAction";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nkinova_msgs/SetFingersPositionActionGoal action_goal\nkinova_msgs/SetFingersPositionActionResult action_result\nkinova_msgs/SetFingersPositionActionFeedback action_feedback\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.SetFingersPositionActionGoal getActionGoal();
  void setActionGoal(kinova_msgs.SetFingersPositionActionGoal value);
  kinova_msgs.SetFingersPositionActionResult getActionResult();
  void setActionResult(kinova_msgs.SetFingersPositionActionResult value);
  kinova_msgs.SetFingersPositionActionFeedback getActionFeedback();
  void setActionFeedback(kinova_msgs.SetFingersPositionActionFeedback value);
}
