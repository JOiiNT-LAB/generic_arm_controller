"""
Marker-relative waypoints during a replay (task_executor_node_complete).

Gives the transform base_frame <- marker for a waypoint saved relative to the marker, in
one of two modes (see README "Marker lock"):

- 'once' (default): measured once per sequence by marker_lock.MarkerLocker and reused, so
  the taught pre-grasp -> grasp geometry is replayed exactly even with a hand-eye offset;
- 'live': re-read at every waypoint, falling back on the last good reading (old behaviour,
  kept for comparison).
"""

import rclpy
from generic_arm_controller.marker_lock import MarkerLocker
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformException

MODES = ('once', 'live')


class MarkerResolver:

    def __init__(self, node, tf_buffer, base_frame, mode, camera_optical_frame,
                 min_distance_m, max_age_s, samples):
        self._node = node
        self._tf_buffer = tf_buffer
        self._base_frame = base_frame
        self._mode = mode
        self._locker = MarkerLocker(
            node=node, tf_buffer=tf_buffer, base_frame=base_frame,
            camera_optical_frame=camera_optical_frame,
            min_distance_m=min_distance_m, max_age_s=max_age_s, samples=samples)
        self._static_tf = StaticTransformBroadcaster(node)
        self._live_cache = {}  # last good reading per frame, 'live' mode only

    def reset(self):
        """Forget the lock: the object may have moved since the previous sequence."""
        self._locker.release()
        self._live_cache = {}

    def resolve(self, source_frame, task_name):
        """Transform base_frame <- source_frame for this waypoint, or None (logged)."""
        if self._mode == 'once':
            return self._resolve_once(source_frame, task_name)
        return self._resolve_live(source_frame, task_name)

    def _log(self):
        return self._node.get_logger()

    def _locked_frame_name(self, frame: str) -> str:
        # Distinct from the saving node's "_locked_teach": the two nodes lock the marker at
        # different times, and the same child_frame_id with two values would conflict in TF.
        return f"{frame}_locked_run"

    def _publish_locked_frame(self, frame: str, ts: TransformStamped):
        """
        Publish the lock as a static TF, only for RViz.

        It shows where the running sequence thinks the marker is: the waypoints are
        transformed with `ts` directly.
        """
        out = TransformStamped()
        out.header.stamp = self._node.get_clock().now().to_msg()
        out.header.frame_id = self._base_frame
        out.child_frame_id = self._locked_frame_name(frame)
        out.transform = ts.transform
        self._static_tf.sendTransform(out)

    def _resolve_once(self, source_frame: str, task_name: str):
        """'once' mode: the lock, taken at the first call of the sequence, then reused."""
        transform, msg, is_new = self._locker.get_or_acquire(source_frame)
        if transform is None:
            self._log().error(
                f"{msg}. Impossibile eseguire '{task_name}': il marker deve "
                f"essere inquadrato almeno una volta, all'inizio della sequenza. "
                f"Metti come primo waypoint una posa assoluta da cui il marker "
                f"sia ben visibile. Salto."
            )
            return None
        if is_new:
            self._log().info(msg)
            self._publish_locked_frame(source_frame, transform)
            self._log().info(
                f"Da qui in poi tutti i waypoint relativi a '{source_frame}' "
                f"useranno questa stima: il marker puo' uscire "
                f"dall'inquadratura senza conseguenze."
            )
        else:
            self._log().info(
                f"'{task_name}': uso l'aggancio su '{source_frame}' gia' "
                f"acquisito in questa sequenza (marker non riletto)."
            )
        return transform

    def _resolve_live(self, source_frame: str, task_name: str):
        """'live' mode: re-read at every waypoint, falling back on the last good reading."""
        need_fallback = False
        fallback_reason = ""
        transform = None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            probe, reason = self._locker.sample_once(source_frame)
            if probe is None:
                need_fallback = True
                fallback_reason = reason
            else:
                self._live_cache[source_frame] = transform
        except TransformException as ex:
            need_fallback = True
            fallback_reason = f"TF error ({source_frame}->{self._base_frame}): {ex}"

        if need_fallback:
            transform = self._live_cache.get(source_frame)
            if transform is None:
                self._log().error(
                    f"{fallback_reason} per '{task_name}'. Nessuna lettura "
                    f"precedente disponibile per '{source_frame}'. Salto."
                )
                return None
            self._log().warn(
                f"{fallback_reason} per '{task_name}'. Uso l'ultima lettura "
                f"buona di '{source_frame}' di questa sequenza."
            )
        return transform
