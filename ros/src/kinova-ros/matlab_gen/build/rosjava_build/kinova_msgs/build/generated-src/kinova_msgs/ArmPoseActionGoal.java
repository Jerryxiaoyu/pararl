package kinova_msgs;

public interface ArmPoseActionGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/ArmPoseActionGoal";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalID goal_id\nkinova_msgs/ArmPoseGoal goal\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalID getGoalId();
  void setGoalId(actionlib_msgs.GoalID value);
  kinova_msgs.ArmPoseGoal getGoal();
  void setGoal(kinova_msgs.ArmPoseGoal value);
}
