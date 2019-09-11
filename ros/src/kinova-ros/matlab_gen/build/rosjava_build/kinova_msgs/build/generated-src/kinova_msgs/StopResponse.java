package kinova_msgs;

public interface StopResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/StopResponse";
  static final java.lang.String _DEFINITION = "string stop_result";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getStopResult();
  void setStopResult(java.lang.String value);
}
