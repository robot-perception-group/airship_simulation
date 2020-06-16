classdef AttitudeThrust < robotics.ros.Message
    %AttitudeThrust MATLAB implementation of mav_msgs/AttitudeThrust
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'mav_msgs/AttitudeThrust' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '7cee443b02735e42bda0ad5910302718' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsQuaternionClass = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Quaternion') % Dispatch to MATLAB class for message type geometry_msgs/Quaternion
        GeometryMsgsVector3Class = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Vector3') % Dispatch to MATLAB class for message type geometry_msgs/Vector3
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        Attitude
        Thrust
    end
    
    properties (Access = protected)
        Cache = struct('Header', [], 'Attitude', [], 'Thrust', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Attitude', 'Header', 'Thrust'} % List of non-constant message properties
        ROSPropertyList = {'attitude', 'header', 'thrust'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = AttitudeThrust(msg)
            %AttitudeThrust Construct the message object AttitudeThrust
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
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'AttitudeThrust', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function attitude = get.Attitude(obj)
            %get.Attitude Get the value for property Attitude
            if isempty(obj.Cache.Attitude)
                obj.Cache.Attitude = feval(obj.GeometryMsgsQuaternionClass, obj.JavaMessage.getAttitude);
            end
            attitude = obj.Cache.Attitude;
        end
        
        function set.Attitude(obj, attitude)
            %set.Attitude Set the value for property Attitude
            validateattributes(attitude, {obj.GeometryMsgsQuaternionClass}, {'nonempty', 'scalar'}, 'AttitudeThrust', 'Attitude');
            
            obj.JavaMessage.setAttitude(attitude.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Attitude)
                obj.Cache.Attitude.setJavaObject(attitude.getJavaObject);
            end
        end
        
        function thrust = get.Thrust(obj)
            %get.Thrust Get the value for property Thrust
            if isempty(obj.Cache.Thrust)
                obj.Cache.Thrust = feval(obj.GeometryMsgsVector3Class, obj.JavaMessage.getThrust);
            end
            thrust = obj.Cache.Thrust;
        end
        
        function set.Thrust(obj, thrust)
            %set.Thrust Set the value for property Thrust
            validateattributes(thrust, {obj.GeometryMsgsVector3Class}, {'nonempty', 'scalar'}, 'AttitudeThrust', 'Thrust');
            
            obj.JavaMessage.setThrust(thrust.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Thrust)
                obj.Cache.Thrust.setJavaObject(thrust.getJavaObject);
            end
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
            obj.Cache.Attitude = [];
            obj.Cache.Thrust = [];
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
            cpObj.Header = copy(obj.Header);
            cpObj.Attitude = copy(obj.Attitude);
            cpObj.Thrust = copy(obj.Thrust);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
            obj.Attitude = feval([obj.GeometryMsgsQuaternionClass '.loadobj'], strObj.Attitude);
            obj.Thrust = feval([obj.GeometryMsgsVector3Class '.loadobj'], strObj.Thrust);
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
            
            strObj.Header = saveobj(obj.Header);
            strObj.Attitude = saveobj(obj.Attitude);
            strObj.Thrust = saveobj(obj.Thrust);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.mav_msgs.AttitudeThrust.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.mav_msgs.AttitudeThrust;
            obj.reload(strObj);
        end
    end
end
