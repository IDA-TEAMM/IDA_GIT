"""
Girdap İDA — Gazebo ↔ ROS 2 Köprü Launch.

Layer 2 simülasyon entegrasyonu. Gazebo (gz sim / Harmonic) `gz-transport`
topic'lerini `ros_gz_bridge` ile ROS 2'ye taşır; karar yığını sahadaki
mavros topic'lerinin aynısını gördüğünden node'lar simülasyon/gerçek ayrımı
yapmadan çalışır.

Mimari:
    gz sim (usv modeli) ──ros_gz_bridge──→ /mavros/* (sensör), /girdap/gazebo/*
                          ←──────────────  thrust komutları (Double)

`ros_gz_bridge` argüman sözdizimi (parameter_bridge):
    "<topic>@<ros_msg>[<gz_msg>"   gz → ros   (sensörler)
    "<topic>@<ros_msg>]<gz_msg>"   ros → gz   (aktüatörler)
    "<topic>@<ros_msg>@<gz_msg>"   çift yönlü

Topic isimleri gz sim konvansiyonu `/model/<model>/...` ile kurulur. Farklı
bir dünya/SDF kullanılırsa yalnız `MODEL_NAME` (ve gerekirse tekil girişler)
güncellenir — `ros` alanı (karar yığınının gördüğü isim) sabit kalır.

Çalıştır (Gazebo dünya ayrıca `gz sim <world>.sdf` ile başlatılmalı):
    ros2 launch girdap_decision gazebo_bridge.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# SDF model adı — teammate'in dünyasındaki modele göre değiştir; tüm gz
# topic'leri buradan türetilir (/model/<MODEL_NAME>/...).
MODEL_NAME = "usv"
_GZ = f"/model/{MODEL_NAME}"


# --------------------------------------------------------------------------- #
# Topic eşleştirmeleri
#   gz  : Gazebo (gz-transport) topic adı
#   ros : Karar yığınının gördüğü ROS 2 topic adı (mavros sözleşmesi)
#   dir : "gz_to_ros" (sensör) | "ros_to_gz" (aktüatör) | "bidir"
# --------------------------------------------------------------------------- #
_BRIDGE_TOPICS = [
    # ----- Sim zamanı (her zaman birinci; use_sim_time bunu tüketir) -----
    {
        "gz": "/clock",
        "ros": "/clock",
        "ros_msg": "rosgraph_msgs/msg/Clock",
        "gz_msg": "gz.msgs.Clock",
        "dir": "gz_to_ros",
    },
    # ----- Sensörler (Gazebo → ROS 2; mavros topic'lerini taklit eder) -----
    {
        "gz": f"{_GZ}/imu",
        "ros": "/mavros/imu/data",
        "ros_msg": "sensor_msgs/msg/Imu",
        "gz_msg": "gz.msgs.IMU",
        "dir": "gz_to_ros",
    },
    {
        "gz": f"{_GZ}/gps",
        "ros": "/mavros/global_position/global",
        "ros_msg": "sensor_msgs/msg/NavSatFix",
        "gz_msg": "gz.msgs.NavSat",
        "dir": "gz_to_ros",
    },
    {
        # Aracın gerçek pozu (ground truth) — fusion doğrulaması / RViz.
        "gz": f"{_GZ}/odometry",
        "ros": "/girdap/gazebo/odom",
        "ros_msg": "nav_msgs/msg/Odometry",
        "gz_msg": "gz.msgs.Odometry",
        "dir": "gz_to_ros",
    },
    {
        # Gövde çerçevesi hız (mavros velocity_body muadili).
        "gz": f"{_GZ}/velocity",
        "ros": "/mavros/local_position/velocity_body",
        "ros_msg": "geometry_msgs/msg/TwistStamped",
        "gz_msg": "gz.msgs.Twist",
        "dir": "gz_to_ros",
    },
    {
        # LiDAR (Livox Mid-360 muadili) — perception cost map girişi.
        "gz": f"{_GZ}/lidar/points",
        "ros": "/livox/lidar",
        "ros_msg": "sensor_msgs/msg/PointCloud2",
        "gz_msg": "gz.msgs.PointCloudPacked",
        "dir": "gz_to_ros",
    },
    {
        # Kamera (OAK-D Lite muadili) — perception + Dosya 1a.
        "gz": f"{_GZ}/camera/image",
        "ros": "/camera/image_raw",
        "ros_msg": "sensor_msgs/msg/Image",
        "gz_msg": "gz.msgs.Image",
        "dir": "gz_to_ros",
    },
    # ----- Aktüatör komutları (ROS 2 → Gazebo) -----
    # Diferansiyel tahrik: mikser /girdap/control/thrust [T_l, T_r] dizisini
    # iki gz Thruster plugin topic'ine (Double) böler. Gazebo tarafında her
    # thruster plugin'i <topic>=thrust_left/thrust_right dinler.
    {
        "gz": f"{_GZ}/thrust_left",
        "ros": "/girdap/gazebo/thrust_left",
        "ros_msg": "std_msgs/msg/Float64",
        "gz_msg": "gz.msgs.Double",
        "dir": "ros_to_gz",
    },
    {
        "gz": f"{_GZ}/thrust_right",
        "ros": "/girdap/gazebo/thrust_right",
        "ros_msg": "std_msgs/msg/Float64",
        "gz_msg": "gz.msgs.Double",
        "dir": "ros_to_gz",
    },
]


def _bridge_arg(entry: dict) -> str:
    """ros_gz_bridge parameter_bridge CLI argüman string'i üret."""
    sep = {"gz_to_ros": "[", "ros_to_gz": "]", "bidir": "@"}[entry["dir"]]
    return f"{entry['gz']}@{entry['ros_msg']}{sep}{entry['gz_msg']}"


def _remap_pair(entry: dict) -> tuple[str, str]:
    """Köprü node'unun ROS tarafını gz topic adından hedef ada yönlendir."""
    return (entry["gz"], entry["ros"])


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Sim zamanı kullan (Gazebo /clock topic'inden)",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    bridge_args = [_bridge_arg(e) for e in _BRIDGE_TOPICS]
    remappings = [_remap_pair(e) for e in _BRIDGE_TOPICS]

    return LaunchDescription(
        [
            use_sim_time_arg,
            LogInfo(
                msg=(
                    f"[gazebo_bridge] model='{MODEL_NAME}' — "
                    f"{len(_BRIDGE_TOPICS)} topic köprüleniyor "
                    f"(sensör→ROS, thrust→gz)."
                ),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=bridge_args,
                remappings=remappings,
            ),
        ]
    )
