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
end

function sysCall_cleanup()
    -- do some clean-up here
    if rosInterfacePresent then
        simROS.shutdownPublisher(blaserVideoPub)
        sim.addStatusbarMessage('Cleaned up: Snake Camera')
    end
end

-- See the user manual or the available code snippets for additional callback functions and details
