classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    properties (Constant)
        mav_msgs_Actuators = 'mav_msgs/Actuators'
        mav_msgs_AttitudeThrust = 'mav_msgs/AttitudeThrust'
        mav_msgs_FilteredSensorData = 'mav_msgs/FilteredSensorData'
        mav_msgs_GpsWaypoint = 'mav_msgs/GpsWaypoint'
        mav_msgs_RateThrust = 'mav_msgs/RateThrust'
        mav_msgs_RollPitchYawrateThrust = 'mav_msgs/RollPitchYawrateThrust'
        mav_msgs_Status = 'mav_msgs/Status'
        mav_msgs_TorqueThrust = 'mav_msgs/TorqueThrust'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(8, 1);
                msgList{1} = 'mav_msgs/Actuators';
                msgList{2} = 'mav_msgs/AttitudeThrust';
                msgList{3} = 'mav_msgs/FilteredSensorData';
                msgList{4} = 'mav_msgs/GpsWaypoint';
                msgList{5} = 'mav_msgs/RateThrust';
                msgList{6} = 'mav_msgs/RollPitchYawrateThrust';
                msgList{7} = 'mav_msgs/Status';
                msgList{8} = 'mav_msgs/TorqueThrust';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
