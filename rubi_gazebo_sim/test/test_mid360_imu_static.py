from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent
DESCRIPTION_ROOT = WORKSPACE_SRC / 'rubi_description'
MODEL_SDF = PACKAGE_ROOT / 'models' / 'RUBI_mid360_imu' / 'model.sdf'


def parse(path):
    return ET.parse(path).getroot()


def test_xacro_contract():
    macro_path = (
        DESCRIPTION_ROOT / 'urdf' / 'sensors' / 'mid360_imu.gazebo.xacro')
    root = parse(macro_path)
    sensors = root.findall('.//sensor[@type="imu"]')
    assert len(sensors) == 1
    sensor = sensors[0]
    assert sensor.findtext('pose') == '${x} ${y} ${z} 0 0 0'
    assert sensor.findtext('update_rate') == '${update_rate}'
    plugin = sensor.find('plugin')
    assert plugin is not None
    assert plugin.attrib['filename'] == 'libgazebo_ros_imu_sensor.so'
    assert plugin.findtext('ros/remapping') == '~/out:=${topic}'
    assert plugin.findtext('frame_name') == '${frame_id}'
    assert plugin.findtext('initial_orientation_as_reference') == 'false'

    variant = (
        DESCRIPTION_ROOT / 'urdf' / 'RUBI_mid360_imu.urdf.xacro').read_text()
    assert 'rubi.urdf' in variant
    assert 'name="livox_frame_raw"' in variant
    assert 'parent="livox_frame"' in variant
    assert 'rpy="3.141592653589793 0 0"' in variant
    assert 'topic="$(arg lidar_raw_topic)"' in variant
    assert 'parent="livox_frame_raw"' in variant
    assert 'topic="$(arg imu_raw_topic)"' in variant
    assert 'frame_id="livox_frame_raw"' in variant
    assert 'x="0.011"' in variant
    assert 'y="0.02329"' in variant
    assert 'z="-0.04412"' in variant
    assert 'update_rate="$(arg imu_update_rate)"' in variant


def test_runtime_sdf_contract():
    root = parse(MODEL_SDF)
    mid360_imus = root.findall('.//sensor[@name="mid360_imu"]')
    assert len(mid360_imus) == 1
    sensor = mid360_imus[0]
    assert sensor.attrib['type'] == 'imu'
    assert sensor.findtext('pose') == '0.011 0.02329 -0.04412 0 0 0'
    assert sensor.findtext('update_rate') == '200'

    ros_imu_plugins = root.findall(
        './/plugin[@filename="libgazebo_ros_imu_sensor.so"]')
    assert len(ros_imu_plugins) == 1
    plugin = ros_imu_plugins[0]
    assert plugin.findtext('ros/remapping') == '~/out:=/livox/imu_raw'
    assert plugin.findtext('frame_name') == 'livox_frame_raw'

    lidar_plugins = root.findall('.//plugin[@filename="libros2_livox.so"]')
    assert len(lidar_plugins) == 1
    assert lidar_plugins[0].findtext('topic') == '/livox/lidar_raw'
    assert lidar_plugins[0].findtext('csv_file_name') == (
        '/tmp/rubi_mid360_imu_mid360.csv')
    assert '/home/' not in MODEL_SDF.read_text()


def test_raw_mount_and_canonical_frame_contract():
    root = parse(MODEL_SDF)
    canonical_frame = root.find('.//frame[@name="livox_frame"]')
    assert canonical_frame is not None
    assert canonical_frame.attrib['attached_to'] == 'BODY'
    assert canonical_frame.findtext('pose').split()[3:] == ['0', '0', '0']

    raw_link = root.find('.//link[@name="livox_frame_raw"]')
    assert raw_link is not None
    assert raw_link.find('pose').attrib['relative_to'] == 'livox_frame'
    assert raw_link.findtext('pose').split()[3:] == [
        '3.141592653589793', '0', '0']
    imu_pose = raw_link.find('sensor[@name="mid360_imu"]/pose')
    assert imu_pose is not None
    assert imu_pose.text.split()[3:] == ['0', '0', '0']
    ray_pose = raw_link.find('sensor[@name="livox_frame_raw"]/pose')
    assert ray_pose is not None
    assert ray_pose.text.split()[3:] == ['0', '0', '0']


def test_safe_launch_defaults():
    launch_text = (
        PACKAGE_ROOT / 'launch' /
        'rubi_gazebo_legacy_policy_mid360_imu.launch.py').read_text()
    assert "DeclareLaunchArgument('launch_joy_node', default_value='false')" in launch_text
    assert "DeclareLaunchArgument('paused', default_value='true')" in launch_text
    assert "DeclareLaunchArgument('auto_demo', default_value='false')" in launch_text
    assert "DeclareLaunchArgument('safe_start_guard', default_value='true')" in launch_text
    assert "executable='mid360_frame_adapter_node'" in launch_text
    assert "'input_lidar_topic': '/livox/lidar_raw'" in launch_text
    assert "'output_lidar_topic': '/livox/lidar'" in launch_text
    assert "'input_imu_topic': '/livox/imu_raw'" in launch_text
    assert "'output_imu_topic': '/livox/imu'" in launch_text
    assert "'output_frame_id': 'livox_frame'" in launch_text

    sensor_only_launch = (
        PACKAGE_ROOT / 'launch' /
        'rubi_mid360_imu_gazebo.launch.py').read_text()
    assert "executable='mid360_frame_adapter_node'" in sensor_only_launch
    assert "'input_lidar_topic': '/livox/lidar_raw'" in sensor_only_launch
    assert "'output_lidar_topic': '/livox/lidar'" in sensor_only_launch
    assert "'input_imu_topic': '/livox/imu_raw'" in sensor_only_launch
    assert "'output_imu_topic': '/livox/imu'" in sensor_only_launch


def test_corrected_imu_analyzer_contract():
    analyzer = (
        PACKAGE_ROOT / 'scripts' / 'mid360_imu_analyzer.py').read_text()
    assert "frames == ['livox_frame']" in analyzer
    assert 'abs(accel_mean[2] - 9.81) < 0.25' in analyzer


def test_axis_fixture_preserves_the_raw_sensor_link():
    fixture = parse(
        DESCRIPTION_ROOT / 'urdf' / 'test' /
        'mid360_imu_axis_test.urdf.xacro')
    canonical_mass = fixture.find(
        './link[@name="livox_frame"]/inertial/mass')
    assert canonical_mass is not None
    assert canonical_mass.attrib['value'] == '0.01'

    raw_joint = fixture.find('./joint[@name="livox_frame_raw_joint"]')
    assert raw_joint is not None
    assert raw_joint.attrib['type'] == 'revolute'
    assert raw_joint.find('origin').attrib['rpy'] == (
        '3.141592653589793 0 0')
    assert raw_joint.find('limit').attrib['lower'] == '-0.000001'
    assert raw_joint.find('limit').attrib['upper'] == '0.000001'


def test_canonical_lidar_wrapper():
    wrapper_text = (
        PACKAGE_ROOT / 'launch' / 'rubi_gazebo_lidar.launch.py').read_text()
    assert 'rubi_gazebo_legacy_policy_mid360_imu.launch.py' in wrapper_text
    assert "DeclareLaunchArgument('gui', default_value='true')" in wrapper_text
    assert "DeclareLaunchArgument('rviz', default_value='false')" in wrapper_text
    assert "DeclareLaunchArgument('paused', default_value='true')" in wrapper_text
    assert "DeclareLaunchArgument('auto_demo', default_value='false')" in wrapper_text
    assert "DeclareLaunchArgument('launch_joy_node', default_value='false')" in wrapper_text
    assert "'namespace', default_value='/rubi_gazebo_legacy'" in wrapper_text

    implementation_markers = (
        'libros2_livox.so',
        'libgazebo_ros_imu_sensor.so',
        'rubi_gazebo_legacy_policy.onnx',
        'robot_description',
    )
    assert all(marker not in wrapper_text for marker in implementation_markers)


def test_legacy_joy_button_contract():
    plugin_text = (
        WORKSPACE_SRC / 'rubi_gazebo_plugins' / 'src' /
        'rubi_gazebo_legacy_policy_plugin.cpp').read_text()
    assert 'create_subscription<sensor_msgs::msg::Joy>(' in plugin_text
    assert '"/joy", 10' in plugin_text
    assert 'constexpr std::size_t kPolicyOnButton = 8;' in plugin_text
    assert 'constexpr std::size_t kTorqueOffButton = 9;' in plugin_text
    assert 'constexpr std::size_t kWalkReadyButton = 10;' in plugin_text
    assert 'rising_edge(kPolicyOnButton)' in plugin_text
    assert 'rising_edge(kTorqueOffButton)' in plugin_text
    assert 'rising_edge(kWalkReadyButton)' in plugin_text
