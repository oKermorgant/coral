from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_arg('waves_topic', '/vrx/wavefield/parameters')

    sl.create_gz_bridge(GazeboBridge(sl.arg('waves_topic'), '/coral/waves',
                                'ros_gz_interfaces/ParamVec', GazeboBridge.gz2ros))

    return sl.launch_description()
