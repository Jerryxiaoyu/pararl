classdef ArmJointAnglesGoal < robotics.ros.Message
    %ArmJointAnglesGoal MATLAB implementation of kinova_msgs/ArmJointAnglesGoal
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'kinova_msgs/ArmJointAnglesGoal' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'cadc8c9df1338a45563e10d707e71551' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        KinovaMsgsJointAnglesClass = robotics.ros.msg.internal.MessageFactory.getClassForType('kinova_msgs/JointAngles') % Dispatch to MATLAB class for message type kinova_msgs/JointAngles
    end
    
    properties (Dependent)
        Angles
    end
    
    properties (Access = protected)
        Cache = struct('Angles', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Angles'} % List of non-constant message properties
        ROSPropertyList = {'angles'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = ArmJointAnglesGoal(msg)
            %ArmJointAnglesGoal Construct the message object ArmJointAnglesGoal
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function angles = get.Angles(obj)
            %get.Angles Get the value for property Angles
            if isempty(obj.Cache.Angles)
                obj.Cache.Angles = feval(obj.KinovaMsgsJointAnglesClass, obj.JavaMessage.getAngles);
            end
            angles = obj.Cache.Angles;
        end
        
        function set.Angles(obj, angles)
            %set.Angles Set the value for property Angles
            validateattributes(angles, {obj.KinovaMsgsJointAnglesClass}, {'nonempty', 'scalar'}, 'ArmJointAnglesGoal', 'Angles');
            
            obj.JavaMessage.setAngles(angles.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Angles)
                obj.Cache.Angles.setJavaObject(angles.getJavaObject);
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Angles = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Recursively copy compound properties
            cpObj.Angles = copy(obj.Angles);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Angles = feval([obj.KinovaMsgsJointAnglesClass '.loadobj'], strObj.Angles);
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Angles = saveobj(obj.Angles);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.kinova_msgs.ArmJointAnglesGoal.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.kinova_msgs.ArmJointAnglesGoal;
            obj.reload(strObj);
        end
    end
end
