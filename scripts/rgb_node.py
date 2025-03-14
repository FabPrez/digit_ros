import os
import sys
import rospy
import yaml
from omegaconf import OmegaConf

# Imposta il percorso corrente
current_dir = os.path.dirname(os.path.abspath(__file__))
other_dir = os.path.abspath(os.path.join(current_dir, 'digit-depth', 'scripts'))
other_dir_2 = os.path.abspath(os.path.join(current_dir, 'digit-depth', 'scripts', 'ros'))
sys.path.append(current_dir)
sys.path.append(other_dir)
sys.path.append(other_dir_2)

# Inizializza il nodo ROS con un nome univoco
sensor_id = rospy.get_param("~sensor_id", "D20928")
node_name = f"digit_node_{sensor_id}"
rospy.init_node(node_name, anonymous=False)

# Percorso del file di configurazione corrispondente
config_path = os.path.abspath(os.path.join(current_dir,"..", "config", f"digit_{sensor_id}.yaml"))

# Controllo se il file esiste
if not os.path.exists(config_path):
    rospy.logerr(f"❌ File di configurazione per il sensore {sensor_id} non trovato: {config_path}")
    sys.exit(1)

# Caricamento del file YAML
with open(config_path, "r") as file:
    config_data = yaml.safe_load(file)

# Converte il dizionario in un oggetto OmegaConf (compatibile con show_depth)
cfg = OmegaConf.create(config_data)

# Importa i moduli dopo aver settato il percorso
from depth import show_depth
from record import record_frame
from digit_image_pub import rgb_pub

if __name__ == "__main__":
    rospy.loginfo(f"Starting digit node for sensor {sensor_id}...")
    rgb_pub(cfg)
