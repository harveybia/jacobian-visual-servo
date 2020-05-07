function sysCall_init()
    -- Check if ROSInterface is loaded
    rosInterfacePresent = sim.isPluginLoaded('ROSInterface')

    if rosInterfacePresent then
        local modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
        local objName = sim.getObjectName(modelHandle)
        blaserCamHandle = sim.getObjectHandle(objName)

        local parentHandle = sim.getObjectParent(modelHandle)
        rosImageTopic = sim.getUserParameter(parentHandle, 'rosImageTopic')
        print(rosImageTopic)
        if (rosImageTopic == nil) then
            rosImageTopic = '/snake_cam/image_color'
            sim.setUserParameter(parentHandle, 'rosImageTopic', rosImageTopic)
        end
        print(rosImageTopic)

        sim.addStatusbarMessage('Initializing ROS stuff')
        blaserVideoPub=simROS.advertise('/snake_cam/image_color', 'sensor_msgs/Image')
        -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
        simROS.publisherTreatUInt8ArrayAsString(blaserVideoPub)

        posePub = simROS.advertise('/snake_arm/pose', 'geometry_msgs/Pose')
    end
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- Publish the image of blaser cam
    if rosInterfacePresent then
        local data,w,h=sim.getVisionSensorCharImage(blaserCamHandle)
        sim.transformImage(data,{w,h},4)

        d={}
        d['header']={seq=0,stamp=simROS.getTime(), frame_id="a"}
        d['height']=h
        d['width']=w
        d['encoding']='rgb8'
        d['is_bigendian']=1
        d['step']=w*3
        d['data']=data
        simROS.publish(blaserVideoPub,d)
    end

    -- Publish the end effector transform and pose
    if rosInterfacePresent then
        base_handle=sim.getObjectHandle('snake_arm')
        tf_base_link_ee=getTransformStamped(blaserCamHandle,'ee_link',base_handle,'base_link')
        -- print(tf_base_link_ee)
        simROS.sendTransform(tf_base_link_ee)

        pose_base_link_ee=getPose(blaserCamHandle,base_handle)
        simROS.publish(posePub,pose_base_link_ee)
    end
end

function sysCall_cleanup()
    -- do some clean-up here
    if rosInterfacePresent then
        simROS.shutdownPublisher(blaserVideoPub)
        sim.addStatusbarMessage('Cleaned up: Snake Camera')
    end
end

function getTransformStamped(objHandle,name,relTo,relToName)
    -- This function retrieves the stamped transform for a specific object
    t = sim.getSystemTime()
    p = sim.getObjectPosition(objHandle,relTo)
    o = sim.getObjectQuaternion(objHandle,relTo)
	return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
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
