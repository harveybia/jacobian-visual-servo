-- if you wish to execute code contained in an external file instead,
-- use the require-directive, e.g.:
--
-- require 'myExternalFile'
--
-- Above will look for <V-REP executable path>/myExternalFile.lua or
-- <V-REP executable path>/lua/myExternalFile.lua
-- (the file can be opened in this editor with the popup menu over
-- the file name)

function process_msg(msg)
    sim.addStatusbarMessage(string.format("Joints:[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
        msg.position[1], msg.position[2], msg.position[3],
        msg.position[4], msg.position[5], msg.position[6],
        msg.position[7], msg.position[8], msg.position[9]))
    
    sim.rmlMoveToJointPositions(jointHandles,-1,
        currentVel,currentAccel,maxVel,maxAccel,maxJerk,msg.position,targetVel)
end

function subscriber_callback(msg)
    -- Enable queuing behavior
    --receivedMessages[#receivedMessages+1]=msg
    
    -- Queue size = 1
    receivedMessages[1]=msg
end

function sysCall_threadmain()
    -- Put some initialization code here

    -- Disable backlash by setting this to 0
    backlashEnable=0

    sub=simROS.subscribe('/joint_states','sensor_msgs/JointState',
        'subscriber_callback',1)
    receivedMessages={}

    vel=4000
    accel=4000
    jerk=80
    currentVel={0,0,0,0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,
            vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,
            vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,
              accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,
              accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,
             jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,
             jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0,0,0,0}

    joint_cam={-1}
    joints_h={-1,-1,-1,-1}
    joints_v={-1,-1,-1,-1}
    
    jointHandles={-1,-1,-1,-1,-1,-1,-1,-1,-1}
    backlashHandles={-1,-1,-1,-1,-1,-1,-1,-1,-1}
    jointNames={'snake_joint_cam','snake_joint_v1','snake_joint_h1',
                'snake_joint_v2','snake_joint_h2','snake_joint_v3',
                'snake_joint_h3','snake_joint_v4','snake_joint_h4'}
    jointHandles[1]=sim.getObjectHandle(jointNames[1])
    jointHandles[2]=sim.getObjectHandle(jointNames[2])
    jointHandles[3]=sim.getObjectHandle(jointNames[3])
    jointHandles[4]=sim.getObjectHandle(jointNames[4])
    jointHandles[5]=sim.getObjectHandle(jointNames[5])
    jointHandles[6]=sim.getObjectHandle(jointNames[6])
    jointHandles[7]=sim.getObjectHandle(jointNames[7])
    jointHandles[8]=sim.getObjectHandle(jointNames[8])
    jointHandles[9]=sim.getObjectHandle(jointNames[9])

    backlashHandles[1]=sim.getObjectHandle(jointNames[1].."_backlash")
    backlashHandles[2]=sim.getObjectHandle(jointNames[2].."_backlash")
    backlashHandles[3]=sim.getObjectHandle(jointNames[3].."_backlash")
    backlashHandles[4]=sim.getObjectHandle(jointNames[4].."_backlash")
    backlashHandles[5]=sim.getObjectHandle(jointNames[5].."_backlash")
    backlashHandles[6]=sim.getObjectHandle(jointNames[6].."_backlash")
    backlashHandles[7]=sim.getObjectHandle(jointNames[7].."_backlash")
    backlashHandles[8]=sim.getObjectHandle(jointNames[8].."_backlash")
    backlashHandles[9]=sim.getObjectHandle(jointNames[9].."_backlash")

    
    for i=1,9,1 do
        sim.setObjectInt32Parameter(
            backlashHandles[i],sim_jointintparam_motor_enabled,1-backlashEnable)
    end

    rosInterfacePresent = sim.isPluginLoaded('ROSInterface')
    if rosInterfacePresent then
        jStatePub = simROS.advertise(
            '/snake_arm/joint_states', 'sensor_msgs/JointState')
    end
    
    for i=1,4,1 do
        joints_h[i]=sim.getObjectHandle('snake_joint_h'..(i))
    end
    joint_cam[1]=sim.getObjectHandle('snake_joint_cam')
    for i=1,4,1 do
        joints_v[i]=sim.getObjectHandle('snake_joint_v'..(i))
    end

    t=0
    seq=0

    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        while #receivedMessages>0 do
            local msg=receivedMessages[#receivedMessages]
            table.remove(receivedMessages)
            -- do something with the msg
            process_msg(msg)
        end

        if rosInterfacePresent then
            publishJointStates()
        end

        sim.switchThread()
    end

end

function sysCall_cleanup()
    -- Put some clean-up code here
end

function publishJointStates()
    -- This joint states feedback simulates actual joint angle
    -- read off from joint encoders
    actual_joint_states={0,0,0,0,0,0,0,0,0}
    for i=1,9,1 do
        jHandle=jointHandles[i]
        bHandle=backlashHandles[i]
        actual_joint_states[i]=
            sim.getJointPosition(jHandle)+sim.getJointPosition(bHandle)
    end

    d={}
    d['header']={seq=0,stamp=simROS.getTime(), frame_id="a"}
    d['name']=jointNames
    d['position']=actual_joint_states
    d['velocity']={0,0,0,0,0,0,0,0,0}
    d['effort']={0,0,0,0,0,0,0,0,0}
    simROS.publish(jStatePub,d)

    seq=seq+1
end