package kinova_msgs;

public interface ArmPoseGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/ArmPoseGoal";
  static final java.lang.String _DEFINITION = "# Goal\ngeometry_msgs/PoseStamped pose\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  geometry_msgs.PoseStamped getPose();
  void setPose(geometry_msgs.PoseStamped value);
}
