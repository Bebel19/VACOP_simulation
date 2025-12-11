import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Odométrie LiDAR (ICP)
    vo_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform': 0.3,

            # --- CORRECTION 1 : FORCER L'INITIALISATION ---
            'Odom/ScanKeyFrameThr': '0.0',       # Accepte tous les scans, même simples
            'Reg/Force3DoF': 'true',             # Force le calcul en 2D (x, y, theta) -> Indispensable pour un rover

            # --- CORRECTION 2 : FILTRAGE EXTÉRIEUR ---
            'Icp/VoxelSize': '0.10',             # Garde du détail (15cm)
            'Icp/RangeMin': '4',               # Ignore les cercles au sol (Rayon < 2.5m)
            'Icp/RangeMax': '0',                 # 0 = Infini (voir le plus loin possible)
            'Icp/MaxCorrespondenceDistance': '0.8', # Cherche des correspondances jusqu'à 1m (utile si ça va vite)
            'Icp/OutlierRatio': '0.65',
            
            # --- CORRECTION 3 : GÉOMÉTRIE (Point-to-Plane) ---
            'Icp/PointToPlane': 'true',          # Utilise les normales (obligatoire pour LiDAR 3D)
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneGroundNormalsUp': '0.8', # Ignore le sol plat (vecteurs vers le haut)
            'Icp/PointToPlaneMinComplexity': '0.005', # Abaisse le seuil de complexité (évite le warning "corridor")

            # --- CORRECTION 4 : ANTI-CRASH & VITESSE (IMPORTANT) ---
            'Odom/Strategy': '1',                # 1 = Frame-to-Frame (Rapide)
            'Odom/GuessMotion': 'true',          # Devine la position future selon la vitesse (CRUCIAL)
            'Icp/MaxTranslation': '0',           # 0 = Désactive la limite de vitesse (Empêche l'erreur "limit out of bounds")
            'Icp/MaxRotation': '0',              # 0 = Désactive la limite de rotation
            'Odom/ResetCountdown': '1',          # Si l'odom est perdue, redémarre auto après 1 scan (au lieu de mourir)
        }],
        remappings=[
            ('scan_cloud', '/scan/point_cloud'),
            ('odom', '/odom'),
        ]
    )
    
    # 2. SLAM (Mapping)
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        # arguments=['-d'], 
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            
            # --- CORRECTION CRITIQUE : FORCER LA CARTE VIA LE NUAGE 3D ---
            'Grid/Sensor': '1',                  # 0=Laser 2D, 1=Nuage 3D. C'est ça qui bloquait !
            
            # --- FILTRAGE DU SOL (Pour avoir une carte propre) ---
            'Grid/MaxGroundHeight': '0.2',       # Les points sous 20cm sont le sol (cases blanches)
            'Grid/MinGroundHeight': '-0.5',      # Ignore les bugs sous le sol
            'Grid/NormalsSegmentation': 'false', # Segmentation simple par hauteur
            
            # --- PARAMÈTRES CARTE ---
            'Grid/RangeMax': '20.0',             # Portée de la carte
            'Grid/RayTracing': 'true',           # Nettoie les cases vides
            'Grid/3D': 'false',                  # Force une carte 2D standard
            
            # Optimisation
            'Rtabmap/DetectionRate': '1.0',
            'Optimizer/Strategy': '1',
            'Reg/Force3DoF': 'true',
            
            'Reg/Strategy': '1',      # 1 = Icp (Utilise le Lidar pour se recaler)
            'RGBD/NeighborLinkRefining':'true', # Affine les liens avec ICP
            'Icp/VoxelSize': '0.15',  # Même voxel que l'odométrie
            'Icp/PointToPlane': 'true',
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw/image_color'),
            ('scan_cloud', '/scan/point_cloud'),
            ('odom', '/odom'),
        ]
    )
    
    viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'subscribe_rgb': False,       # On a coupé la caméra, souviens-toi
            'subscribe_odom': True,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True, # On utilise le Cloud
            'approx_sync': True,
            'queue_size': 10,
            'wait_for_transform': 0.2,
        }],
        remappings=[
            ('scan_cloud', '/scan/point_cloud'),
            ('odom', '/odom'),
        ]
    )

    return LaunchDescription([vo_node, slam_node, viz_node])
