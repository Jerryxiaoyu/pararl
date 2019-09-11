package kinova_msgs;

public interface Arm_KinovaPoseGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/Arm_KinovaPoseGoal";
  static final java.lang.String _DEFINITION = "# Goal\nKinovaPose kinova_pose\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.KinovaPose getKinovaPose();
  void setKinovaPose(kinova_msgs.KinovaPose value);
}
