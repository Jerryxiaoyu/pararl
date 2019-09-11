package kinova_msgs;

public interface Arm_KinovaPoseActionResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/Arm_KinovaPoseActionResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nkinova_msgs/Arm_KinovaPoseResult result\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  kinova_msgs.Arm_KinovaPoseResult getResult();
  void setResult(kinova_msgs.Arm_KinovaPoseResult value);
}
