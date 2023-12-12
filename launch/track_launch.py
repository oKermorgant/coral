from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)
    sl.declare_arg('link', default_value='')
    sl.declare_arg('x', default_value=0.)
    sl.declare_arg('y', default_value=0.)
    sl.declare_arg('z', default_value=0.)
    sl.declare_arg('roll', default_value=0.)
    sl.declare_arg('pitch', default_value=0.)
    sl.declare_arg('yaw', default_value=0.)

    tf_args = ['--frame-id', sl.arg('link'), '--child-frame-id', 'coral_cam_view']

    for axis in ('x','y','z','roll','pitch','yaw'):
        tf_args.extend([f'--{axis}', sl.arg(axis)])

    sl.node('tf2_ros', 'static_transform_publisher', arguments = tf_args)

    return sl.launch_description()
