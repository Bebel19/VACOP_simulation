import open3d as o3d
import numpy as np
import os

# Nom du fichier exporté
filename = "ma_carte_3d.pcd"

# Vérifier si le fichier existe
if not os.path.exists(filename):
    print(f"Erreur: Le fichier {filename} n'existe pas.")
    print("Avez-vous lancé la commande 'rtabmap-export' ?")
    exit()

print(f"Chargement de {filename}...")
# Charger le nuage de points
pcd = o3d.io.read_point_cloud(filename)

# Afficher quelques infos
print(pcd)
print(f"Nombre de points : {len(pcd.points)}")

# --- OPTIONNEL : RECOLORER EN FONCTION DE LA HAUTEUR (Z) ---
# Cela aide beaucoup à distinguer le sol des murs visuellement
points = np.asarray(pcd.points)
if len(points) > 0:
    z_values = points[:, 2] # Prendre la colonne Z
    # Normaliser Z entre 0 et 1 pour la couleur
    z_norm = (z_values - min(z_values)) / (max(z_values) - min(z_values))
    # Appliquer une carte de couleur (bleu=bas, rouge=haut)
    import matplotlib.pyplot as plt
    cmap = plt.get_cmap("jet")
    colors = cmap(z_norm)[:, :3] # Garder RGB, jeter Alpha
    pcd.colors = o3d.utility.Vector3dVector(colors)
    print("Recoloration basée sur la hauteur appliquée.")
# ---------------------------------------------------------

print("Ouverture de la fenêtre 3D. Utilisez la souris pour tourner/zoomer.")
# Créer une fenêtre de visualisation
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Ma Carte 3D RTAB-Map", width=1280, height=720)

# Ajouter le nuage
vis.add_geometry(pcd)

# Options de rendu pour faire plus joli (points plus gros)
opt = vis.get_render_option()
opt.point_size = 3.0
opt.background_color = np.asarray([0.1, 0.1, 0.1]) # Fond gris foncé

# Lancer la boucle de visualisation
vis.run()
vis.destroy_window()
