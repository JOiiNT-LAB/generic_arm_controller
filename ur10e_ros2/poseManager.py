# pose_manager.py (Nessuna modifica rispetto alla versione precedente)
import json
import numpy as np
import os

class PoseManager:
    def __init__(self, file_path="robot_poses.json", logger=None):
        self.file_path = file_path
        self.poses = []  # Lista di configurazioni dei giunti (np.array)
        self.logger = logger if logger is not None else self._default_logger()
        self.load_poses()

    def _default_logger(self):
        import logging
        logger = logging.getLogger("PoseManager")
        logger.setLevel(logging.INFO)
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(levelname)s: %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger

    def load_poses(self):
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    data = json.load(f)
                    # Qui salvavamo np.array. Se /ur10e_fk_pose produce PoseStamped,
                    # devi decidere cosa salvare: la Pose (x,y,z,qx,qy,qz,qw) o il np.array dei giunti.
                    # Per salvare la Pose:
                    self.poses = [p for p in data] # Salveremo direttamente i dict per le Pose
                self.logger.info(f"Caricate {len(self.poses)} pose da '{self.file_path}'")
            except json.JSONDecodeError as e:
                self.logger.error(f"Errore nel parsing del file JSON '{self.file_path}': {e}")
                self.poses = []
            except Exception as e:
                self.logger.error(f"Errore durante il caricamento delle pose da '{self.file_path}': {e}")
                self.poses = []
        else:
            self.logger.info(f"File '{self.file_path}' non trovato. Nessuna posa caricata.")
        return self.poses

    def save_poses(self):
        try:
            # Le pose sono già in formato serializzabile (dict per PoseStamped)
            with open(self.file_path, 'w') as f:
                json.dump(self.poses, f, indent=2)
            self.logger.info(f"Salvate {len(self.poses)} pose in '{self.file_path}'")
            return True
        except Exception as e:
            self.logger.error(f"Errore durante il salvataggio delle pose in '{self.file_path}': {e}")
            return False

    def add_pose(self, pose_dict: dict): # Cambiato tipo di parametro in dict per Pose
        self.poses.append(pose_dict)
        self.logger.info(f"Aggiunta posa. Totale pose: {len(self.poses)}")
        self.save_poses()

    def get_poses(self) -> list[dict]: # Cambiato tipo di ritorno in list[dict]
        return self.poses

    def clear_poses(self):
        self.poses = []
        if os.path.exists(self.file_path):
            try:
                os.remove(self.file_path)
                self.logger.info(f"File '{self.file_path}' rimosso.")
            except Exception as e:
                self.logger.error(f"Impossibile rimuovere il file '{self.file_path}': {e}")
        self.logger.info("Tutte le pose sono state eliminate dalla memoria.")