function sysCall_init()
    -- Check if ROSInterface is loaded
    rosInterfacePresent = sim.isPluginLoaded('ROSInterface')

    if rosInterfacePresent then
        local modelHandle = sim.getObjectAssociatedWithScript(sim.handle_self)
        local objName = sim.getObjectName(modelHandle)
        visionSensorHandle=sim.getObjectHandle(objName)
        baseHandle=sim.getObjectHandle('snake_arm')
        eeHandle=sim.getObjectHandle('snake_body10')
        aprilHandle=sim.getObjectHandle('apriltag_frame')
        cameraHandle=sim.getObjectHandle('ee_cam_frame')

        local parentHandle = sim.getObjectParent(modelHandle)
        rosImageTopic = sim.getUserParameter(parentHandle, 'rosImageTopic')
        if (rosImageTopic == nil) then
            rosImageTopic = '/snake_cam/image_color'
            sim.setUserParameter(parentHandle, 'rosImageTopic', rosImageTopic)
        end
        print(string.format('rosImageTopic: %s',rosImageTopic))

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
        local data,w,h=sim.getVisionSensorCharImage(visionSensorHandle)
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

    -- Publish transforms and poses
    if rosInterfacePresent then
        -- world -> base_link
        tf_world_base_link=getTransformStamped(
            baseHandle,'base_link',-1,'world')
        simROS.sendTransform(tf_world_base_link)

        -- base_link -> ee_link
        tf_base_link_ee=getTransformStamped(
            eeHandle,'ee_link',baseHandle,'base_link')
        simROS.sendTransform(tf_base_link_ee)

        -- ee_link -> camera
        tf_ee_link_cam=getTransformStamped(
            cameraHandle,'camera',eeHandle,'ee_link')
        simROS.sendTransform(tf_ee_link_cam)

        -- base_link -> apriltag
        tf_base_april=getTransformStamped(
            aprilHandle,'tag_0',baseHandle,'base_link')
        simROS.sendTransform(tf_base_april)

        pose_base_link_ee=getPose(eeHandle,baseHandle)
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
    t = simROS.getTime()
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
