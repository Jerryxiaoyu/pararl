package kinova_msgs;

public interface ArmPoseFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/ArmPoseFeedback";
  static final java.lang.String _DEFINITION = "# Feedback message\ngeometry_msgs/PoseStamped pose";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  geometry_msgs.PoseStamped getPose();
  void setPose(geometry_msgs.PoseStamped value);
}
