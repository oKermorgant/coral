from simple_launch import SimpleLauncher

sl = SimpleLauncher(use_sim_time='auto')
sl.declare_arg('link', default_value='base_link')
sl.declare_arg('x', default_value=0.)
sl.declare_arg('y', default_value=0.)
sl.declare_arg('z', default_value=0.)
sl.declare_arg('roll', default_value=0.)
sl.declare_arg('pitch', default_value=0.)
sl.declare_arg('yaw', default_value=0.)


def launch_setup():

    slider_file = '/tmp/coral_cam_view.yaml'

    config = ['type: tf2_msgs/msg/TFMessage',
              f"transforms[0].header.frame_id: {sl.arg('link')}",
              'transforms[0].child_frame_id: coral_cam_view']

    for axis in ('x','y','z','roll','pitch','yaw'):

        field = 'translation' if len(axis) == 1 else 'rotation'

        config += [f'{axis}:',
                   f'  to: transforms[0].transform.{field}.{axis}',
                   '  max: pi',
                   '  min: -pi']

    config = ['/tf:'] + [f'  {line}' for line in config]

    with open(slider_file, 'w') as f:
        f.write('\n'.join(config))

    sl.node('slider_publisher', 'slider_publisher',
            name = 'coral_cam_view',
            arguments = [slider_file])

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
