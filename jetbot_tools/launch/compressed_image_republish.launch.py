from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define parameter sets for each republisher
    color_params = {
        'input_topic': '/camera/color/raw/compressed',
        'output_topic': '/camera/color/in/compressed'
    }

    depth_params = {
        'input_topic': '/camera/depth/raw/compressedDepth',
        'output_topic': '/camera/depth/in/compressedDepth'
    }

    image_transport_params = {
        'namespace': '/camera/color',
        'input_topic': '/in/compressed',
        'output_topic': 'image_raw_decompressed'
    }

    depth_image_transport_params = {
        'namespace': '/camera/depth',
        'input_topic': '/in/compressedDepth',
        'output_topic': 'image_raw_decompressed'
    }

    # Define nodes
    color_republisher = Node(
        package='jetbot_tools',
        executable='compressed_image_republisher',
        name='color_compressed_republisher',
        output='screen',
        parameters=[color_params]
    )

    depth_republisher = Node(
        package='jetbot_tools',
        executable='compressed_image_republisher',
        name='depth_compressed_republisher',
        output='screen',
        parameters=[depth_params]
    )


    image_transport_republisher = Node(
        package='image_transport',
        executable='republish',
        name='image_transport_republisher',
        output='screen',
        arguments=['compressed', 'raw'],
        namespace=image_transport_params['namespace'],
        remappings=[
            ('in', image_transport_params['input_topic']),
            ('out', image_transport_params['output_topic'])
        ]
    )

    depth_image_transport_republisher = Node(
        package='image_transport',
        executable='republish',
        name='depth_image_transport_republisher',
        output='screen',
        arguments=['compressedDepth', 'raw'],
        namespace=depth_image_transport_params['namespace'],
        remappings=[
            ('in', depth_image_transport_params['input_topic']),
            ('out', depth_image_transport_params['output_topic'])
        ]
    )

    # Return LaunchDescription
    return LaunchDescription([
        color_republisher,
        depth_republisher,
        image_transport_republisher,
        depth_image_transport_republisher
    ])
