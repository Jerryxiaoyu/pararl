package kinova_msgs;

public interface Arm_KinovaPoseFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/Arm_KinovaPoseFeedback";
  static final java.lang.String _DEFINITION = "# Feedback message\nKinovaPose kinova_pose";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  kinova_msgs.KinovaPose getKinovaPose();
  void setKinovaPose(kinova_msgs.KinovaPose value);
}
