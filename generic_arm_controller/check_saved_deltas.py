#!/usr/bin/env python3
"""Distanza tra pose 'move' consecutive salvate nello stesso source_frame.

Test di accettazione dell'aggancio one-shot del marker (vedi marker_lock.py),
indipendente dalla qualita' della calibrazione mano-occhio: se due pose sono
riferite alla STESSA stima di ^B T_M, un eventuale offset globale su quella
stima si cancella nella differenza, e la distanza tra le due pose salvate deve
coincidere con lo spostamento reale del braccio.

Se non coincide, le due pose sono riferite a stime DIVERSE del marker - cioe'
l'aggancio non sta funzionando (era il caso del vecchio meccanismo di freeze,
che si riaggiornava a 5 Hz: 15 cm di discesa reale venivano registrati come 3.4).

Uso:
    python3 check_saved_deltas.py [percorso_json]
"""

import json
import math
import os
import sys


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/ros2_ws/src/task_result/robot_poses_ws.json'
    )
    with open(path) as f:
        entries = json.load(f)

    print(f"{path}\n")
    prev = None
    prev_idx = None
    for i, e in enumerate(entries):
        if e.get('task_type') != 'move':
            # Un'azione gripper non spezza il confronto, ma va segnalata:
            # tra le due pose c'e' stata una presa/rilascio.
            print(f"  [{i}] -- {e.get('task_type')}: {e.get('command')} --")
            continue
        src = e.get('source_frame', '?')
        t = e['transform']['translation']
        cur = (t['x'], t['y'], t['z'])
        label = e.get('task_name', f'#{i}')
        line = f"  [{i}] {label:<24} rispetto a '{src}'"
        if prev is not None and prev[0] == src:
            d = math.dist(cur, prev[1])
            line += (f"   distanza da [{prev_idx}]: "
                     f"{d * 1000:7.1f} mm")
        print(line)
        prev = (src, cur)
        prev_idx = i

    print("\nConfronta ogni distanza con lo spostamento reale del braccio tra i "
          "due salvataggi:\nse coincidono (pochi mm di scarto) l'aggancio sta "
          "funzionando.")


if __name__ == '__main__':
    main()
