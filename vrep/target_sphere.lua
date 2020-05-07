function sysCall_init()
    rosInterfacePresent = sim.isPluginLoaded('ROSInterface')
    if rosInterfacePresent then
        targetPosePub = simROS.advertise(
            '/gst_desired', 'geometry_msgs/Pose')
    end

    local modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
    local objName = sim.getObjectName(modelHandle)
    sphereHandle = sim.getObjectHandle(objName)

    baseLinkHandle = sim.getObjectHandle('snake_arm')
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
    if rosInterfacePresent then
        pose = getPose(sphereHandle, baseLinkHandle)
        simROS.publish(targetPosePub, pose)
    end
end

function sysCall_cleanup()
    -- do some clean-up here
end

function getPose(objectHandle, relTo)
    -- This function get the object pose at ROS format geometry_msgs/Pose
    p = sim.getObjectPosition(objectHandle,relTo)
    o = sim.getObjectQuaternion(objectHandle,relTo)
    return {
        position={x=p[1],y=p[2],z=p[3]},
        orientation={x=o[1],y=o[2],z=o[3],w=o[4]}
    }
end
-- See the user manual or the available code snippets for additional callback functions and details
