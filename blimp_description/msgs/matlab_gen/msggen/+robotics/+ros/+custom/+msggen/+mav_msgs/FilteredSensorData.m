classdef FilteredSensorData < robotics.ros.Message
    %FilteredSensorData MATLAB implementation of mav_msgs/FilteredSensorData
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'mav_msgs/FilteredSensorData' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'a9b51fae1f4ed3a8a0522b4ffe61659f' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        GeometryMsgsVector3Class = robotics.ros.msg.internal.MessageFactory.getClassForType('geometry_msgs/Vector3') % Dispatch to MATLAB class for message type geometry_msgs/Vector3
        StdMsgsHeaderClass = robotics.ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        Accelerometer
        Gyroscope
        Magnetometer
        Barometer
    end
    
    properties (Access = protected)
        Cache = struct('Header', [], 'Accelerometer', [], 'Gyroscope', [], 'Magnetometer', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Accelerometer', 'Barometer', 'Gyroscope', 'Header', 'Magnetometer'} % List of non-constant message properties
        ROSPropertyList = {'accelerometer', 'barometer', 'gyroscope', 'header', 'magnetometer'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = FilteredSensorData(msg)
            %FilteredSensorData Construct the message object FilteredSensorData
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
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'FilteredSensorData', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function accelerometer = get.Accelerometer(obj)
            %get.Accelerometer Get the value for property Accelerometer
            if isempty(obj.Cache.Accelerometer)
                obj.Cache.Accelerometer = feval(obj.GeometryMsgsVector3Class, obj.JavaMessage.getAccelerometer);
            end
            accelerometer = obj.Cache.Accelerometer;
        end
        
        function set.Accelerometer(obj, accelerometer)
            %set.Accelerometer Set the value for property Accelerometer
            validateattributes(accelerometer, {obj.GeometryMsgsVector3Class}, {'nonempty', 'scalar'}, 'FilteredSensorData', 'Accelerometer');
            
            obj.JavaMessage.setAccelerometer(accelerometer.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Accelerometer)
                obj.Cache.Accelerometer.setJavaObject(accelerometer.getJavaObject);
            end
        end
        
        function gyroscope = get.Gyroscope(obj)
            %get.Gyroscope Get the value for property Gyroscope
            if isempty(obj.Cache.Gyroscope)
                obj.Cache.Gyroscope = feval(obj.GeometryMsgsVector3Class, obj.JavaMessage.getGyroscope);
            end
            gyroscope = obj.Cache.Gyroscope;
        end
        
        function set.Gyroscope(obj, gyroscope)
            %set.Gyroscope Set the value for property Gyroscope
            validateattributes(gyroscope, {obj.GeometryMsgsVector3Class}, {'nonempty', 'scalar'}, 'FilteredSensorData', 'Gyroscope');
            
            obj.JavaMessage.setGyroscope(gyroscope.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Gyroscope)
                obj.Cache.Gyroscope.setJavaObject(gyroscope.getJavaObject);
            end
        end
        
        function magnetometer = get.Magnetometer(obj)
            %get.Magnetometer Get the value for property Magnetometer
            if isempty(obj.Cache.Magnetometer)
                obj.Cache.Magnetometer = feval(obj.GeometryMsgsVector3Class, obj.JavaMessage.getMagnetometer);
            end
            magnetometer = obj.Cache.Magnetometer;
        end
        
        function set.Magnetometer(obj, magnetometer)
            %set.Magnetometer Set the value for property Magnetometer
            validateattributes(magnetometer, {obj.GeometryMsgsVector3Class}, {'nonempty', 'scalar'}, 'FilteredSensorData', 'Magnetometer');
            
            obj.JavaMessage.setMagnetometer(magnetometer.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Magnetometer)
                obj.Cache.Magnetometer.setJavaObject(magnetometer.getJavaObject);
            end
        end
        
        function barometer = get.Barometer(obj)
            %get.Barometer Get the value for property Barometer
            barometer = double(obj.JavaMessage.getBarometer);
        end
        
        function set.Barometer(obj, barometer)
            %set.Barometer Set the value for property Barometer
            validateattributes(barometer, {'numeric'}, {'nonempty', 'scalar'}, 'FilteredSensorData', 'Barometer');
            
            obj.JavaMessage.setBarometer(barometer);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
            obj.Cache.Accelerometer = [];
            obj.Cache.Gyroscope = [];
            obj.Cache.Magnetometer = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Barometer = obj.Barometer;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
            cpObj.Accelerometer = copy(obj.Accelerometer);
            cpObj.Gyroscope = copy(obj.Gyroscope);
            cpObj.Magnetometer = copy(obj.Magnetometer);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Barometer = strObj.Barometer;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
            obj.Accelerometer = feval([obj.GeometryMsgsVector3Class '.loadobj'], strObj.Accelerometer);
            obj.Gyroscope = feval([obj.GeometryMsgsVector3Class '.loadobj'], strObj.Gyroscope);
            obj.Magnetometer = feval([obj.GeometryMsgsVector3Class '.loadobj'], strObj.Magnetometer);
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
            
            strObj.Barometer = obj.Barometer;
            strObj.Header = saveobj(obj.Header);
            strObj.Accelerometer = saveobj(obj.Accelerometer);
            strObj.Gyroscope = saveobj(obj.Gyroscope);
            strObj.Magnetometer = saveobj(obj.Magnetometer);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.mav_msgs.FilteredSensorData.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.mav_msgs.FilteredSensorData;
            obj.reload(strObj);
        end
    end
end