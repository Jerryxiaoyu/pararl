package kinova_msgs;

public interface SetTorqueControlModeRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/SetTorqueControlModeRequest";
  static final java.lang.String _DEFINITION = "uint16 state\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  short getState();
  void setState(short value);
}
