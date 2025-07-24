import json
import os
import logging # Importa il modulo logging

class PoseManager:
    def __init__(self, file_path="robot_poses.json", logger=None):
        self.file_path = file_path
        # La lista 'poses' ora conterrà dizionari, non np.array
        self.poses = []
        self.logger = logger if logger is not None else self._default_logger()
        self.load_poses()

    def _default_logger(self):
        """Crea un logger di default se non viene fornito."""
        logger = logging.getLogger("PoseManager")
        logger.setLevel(logging.INFO)
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(levelname)s: %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger

    def load_poses(self):
        """Carica le pose dal file JSON."""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r') as f:
                    # Carica direttamente la lista di dizionari
                    data = json.load(f)
                    # Assicurati che i dati caricati siano una lista (o gestisci il caso in cui non lo sia)
                    if isinstance(data, list):
                        self.poses = data
                    else:
                        self.logger.warning(f"Il contenuto del file '{self.file_path}' non è una lista. Inizializzo le pose come vuote.")
                        self.poses = []
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
        """Salva le pose nel file JSON."""
        try:
            # Crea la directory se non esiste
            os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
            with open(self.file_path, 'w') as f:
                json.dump(self.poses, f, indent=4) # Usa indent=4 per una migliore leggibilità del JSON
            self.logger.info(f"Salvate {len(self.poses)} pose in '{self.file_path}'")
            return True
        except Exception as e:
            self.logger.error(f"Errore durante il salvataggio delle pose in '{self.file_path}': {e}")
            return False

    def add_pose(self, pose_data: dict):
        """Aggiunge una nuova posa (dizionario) alla lista e la salva nel file."""
        self.poses.append(pose_data)
        # La riga qui sotto genera log duplicati se save_poses logga già.
        # self.logger.info(f"Aggiunta posa. Totale pose: {len(self.poses)}")
        self.save_poses()
        # Aggiungo un log più specifico per l'aggiunta della posa, senza ripetere il totale
        task_name = pose_data.get('task_name', 'Unnamed Pose')
        self.logger.info(f"Posa '{task_name}' aggiunta al PoseManager.")


    def get_poses(self) -> list[dict]:
        """Restituisce la lista di tutte le pose salvate."""
        return self.poses

    def clear_poses(self):
        """Cancella tutte le pose dalla memoria e dal file."""
        self.poses = []
        if os.path.exists(self.file_path):
            try:
                os.remove(self.file_path)
                self.logger.info(f"File '{self.file_path}' rimosso.")
            except Exception as e:
                self.logger.error(f"Impossibile rimuovere il file '{self.file_path}': {e}")
        self.logger.info("Tutte le pose sono state eliminate dalla memoria del PoseManager.")
        # Non è necessario chiamare save_poses qui, dato che abbiamo rimosso il file.
        # Se il file non fosse rimosso, save_poses creerebbe un file JSON vuoto.
        # Decidiamo di rimuoverlo per coerenza con l'eliminazione totale.

    def get_pose_by_name(self, task_name: str) -> dict | None:
        """Recupera una posa specifica tramite il suo nome del task."""
        for pose in self.poses:
            if pose.get('task_name') == task_name:
                return pose
        self.logger.warning(f"Posa con nome '{task_name}' non trovata.")
        return None