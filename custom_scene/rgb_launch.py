from simple_launch import SimpleLauncher
import yaml
import os

sl = SimpleLauncher()
sl.declare_arg('field', 'uwFog')
sl.declare_arg('base', 'night')

def launch_setup():

    rgb = sl.find('coral','rgb.yaml')

    with open(rgb) as f:
        config = yaml.safe_load(f)
    config['color']['field'] = sl.arg('field')
    config['color']['base'] = sl.arg('base')
    with open(rgb,'w') as f:
        yaml.safe_dump(config, f)

    sl.node('slider_publisher',arguments=[rgb])

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)


