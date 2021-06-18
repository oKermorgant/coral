from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()    
    sl.declare_arg('link', default_value='')
    sl.declare_arg('x', default_value=0.)
    sl.declare_arg('y', default_value=0.)
    sl.declare_arg('z', default_value=0.)
    sl.declare_arg('roll', default_value=0.)
    sl.declare_arg('pitch', default_value=0.)
    sl.declare_arg('yaw', default_value=0.)

    tf_args = [sl.arg('x'), sl.arg('y'), sl.arg('z'), sl.arg('yaw'), sl.arg('pitch'), sl.arg('roll'), sl.arg('link'), 'coral_cam_view']
        
    sl.node('tf2_ros', 'static_transform_publisher', arguments = tf_args)
    
    return sl.launch_description()
