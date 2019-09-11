package kinova_msgs;

public interface SetForceControlParamsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinova_msgs/SetForceControlParamsRequest";
  static final java.lang.String _DEFINITION = "geometry_msgs/Vector3 inertia_linear\ngeometry_msgs/Vector3 inertia_angular\ngeometry_msgs/Vector3 damping_linear\ngeometry_msgs/Vector3 damping_angular\ngeometry_msgs/Vector3 force_min_linear\ngeometry_msgs/Vector3 force_min_angular\ngeometry_msgs/Vector3 force_max_linear\ngeometry_msgs/Vector3 force_max_angular\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  geometry_msgs.Vector3 getInertiaLinear();
  void setInertiaLinear(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getInertiaAngular();
  void setInertiaAngular(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getDampingLinear();
  void setDampingLinear(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getDampingAngular();
  void setDampingAngular(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getForceMinLinear();
  void setForceMinLinear(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getForceMinAngular();
  void setForceMinAngular(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getForceMaxLinear();
  void setForceMaxLinear(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getForceMaxAngular();
  void setForceMaxAngular(geometry_msgs.Vector3 value);
}
