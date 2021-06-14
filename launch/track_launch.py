from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()    
    sl.declare_arg('link', default_value='')
    tf_args = ['0']*6 + [sl.arg('link'), 'coral_cam_view']
        
    sl.node('tf2_ros', 'static_transform_publisher', arguments = tf_args)
    
    return sl.launch_description()
