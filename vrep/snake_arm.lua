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
    jointHandles[1]=sim.getObjectHandle('snake_joint_cam')
    jointHandles[2]=sim.getObjectHandle('snake_joint_v1')
    jointHandles[3]=sim.getObjectHandle('snake_joint_h1')
    jointHandles[4]=sim.getObjectHandle('snake_joint_v2')
    jointHandles[5]=sim.getObjectHandle('snake_joint_h2')
    jointHandles[6]=sim.getObjectHandle('snake_joint_v3')
    jointHandles[7]=sim.getObjectHandle('snake_joint_h3')
    jointHandles[8]=sim.getObjectHandle('snake_joint_v4')
    jointHandles[9]=sim.getObjectHandle('snake_joint_h4')
    
    for i=1,4,1 do
        joints_h[i]=sim.getObjectHandle('snake_joint_h'..(i))
    end
    joint_cam[1]=sim.getObjectHandle('snake_joint_cam')
    for i=1,4,1 do
        joints_v[i]=sim.getObjectHandle('snake_joint_v'..(i))
    end

    t=0

    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        while #receivedMessages>0 do
            local msg=receivedMessages[#receivedMessages]
            table.remove(receivedMessages)
            -- do something with the msg
            process_msg(msg)
        end
        sim.switchThread()
    end

end

function sysCall_cleanup()
    -- Put some clean-up code here
end