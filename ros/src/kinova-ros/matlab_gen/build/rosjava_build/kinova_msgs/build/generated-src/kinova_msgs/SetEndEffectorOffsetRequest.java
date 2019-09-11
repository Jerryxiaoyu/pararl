package kinova_msgs;

public interface SetEndEffectorOffsetRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/SetEndEffectorOffsetRequest";
  static final java.lang.String _DEFINITION = "uint16 status\ngeometry_msgs/Vector3 offset\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  short getStatus();
  void setStatus(short value);
  geometry_msgs.Vector3 getOffset();
  void setOffset(geometry_msgs.Vector3 value);
}
