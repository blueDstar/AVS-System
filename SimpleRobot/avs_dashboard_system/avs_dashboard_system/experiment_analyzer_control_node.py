"""
AVS Dashboard System — Experiment Analyzer Control Node
File: avs_dashboard_system/experiment_analyzer_control_node.py

Reads multiple experiment CSV files, computes comparison metrics, and
publishes controller comparison summaries for the dashboard.

Topics:
  Subscribed: /avs/analyzer_cmd   (JSON: load_experiment, compare, export)
  Published:  /avs/experiment_summary    (single experiment metrics JSON)
              /avs/controller_comparison  (multi-controller comparison JSON)
              /avs/experiment_list        (list of available experiments)
"""

import csv
import json
import os
import threading
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

from avs_dashboard_system.utils import (
    parse_json_string, compute_rmse, compute_mae, compute_smoothness
)

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE
)


class ExperimentAnalyzerControlNode(Node):
    """
    Reads experiment data from disk and publishes comparison metrics.
    All heavy computation runs in background threads to avoid blocking ROS.
    """

    def __init__(self):
        super().__init__('experiment_analyzer_control')

        self.declare_parameter('base_dir', '~/avs_experiments')
        self.declare_parameter('topic_analyzer_cmd', '/avs/analyzer_cmd')
        self.declare_parameter('topic_exp_summary', '/avs/experiment_summary')
        self.declare_parameter('topic_comparison', '/avs/controller_comparison')
        self.declare_parameter('topic_exp_list', '/avs/experiment_list')

        self._base_dir = Path(
            self.get_parameter('base_dir').value).expanduser()
        self._lock = threading.Lock()
        self._loaded_experiments: dict = {}   # dir_name → metrics dict

        # Publishers
        r10 = RELIABLE_QOS
        self._pub_summary    = self.create_publisher(
            String, self.get_parameter('topic_exp_summary').value, r10)
        self._pub_comparison = self.create_publisher(
            String, self.get_parameter('topic_comparison').value, r10)
        self._pub_exp_list   = self.create_publisher(
            String, self.get_parameter('topic_exp_list').value, r10)

        # Subscriber
        self.create_subscription(String,
            self.get_parameter('topic_analyzer_cmd').value,
            self._cb_analyzer_cmd, r10)

        # Timer: refresh experiment list every 10 seconds
        self.create_timer(10.0, self._refresh_experiment_list)
        self._refresh_experiment_list()

        self.get_logger().info(
            f'[experiment_analyzer_control] Ready | base_dir={self._base_dir}')

    # =========================================================================
    # Command handler
    # =========================================================================

    def _cb_analyzer_cmd(self, msg: String):
        data = parse_json_string(msg.data)
        if not data:
            return
        action = data.get('action', '')

        if action == 'list_experiments':
            self._refresh_experiment_list()
        elif action == 'load_experiment':
            exp_id = data.get('experiment_id', '')
            threading.Thread(
                target=self._load_experiment, args=(exp_id,),
                daemon=True).start()
        elif action == 'compare':
            exp_ids  = data.get('experiment_ids', [])
            scenario = data.get('scenario', '')
            threading.Thread(
                target=self._compare_experiments,
                args=(exp_ids, scenario),
                daemon=True).start()
        elif action == 'export_comparison':
            exp_ids = data.get('experiment_ids', [])
            threading.Thread(
                target=self._export_comparison_csv,
                args=(exp_ids,),
                daemon=True).start()
        else:
            self.get_logger().warn(
                f'[analyzer] Unknown action: {action}')

    # =========================================================================
    # Experiment list
    # =========================================================================

    def _refresh_experiment_list(self):
        if not self._base_dir.exists():
            return
        experiments = []
        for item in sorted(self._base_dir.iterdir(), reverse=True):
            if not item.is_dir():
                continue
            meta_file = item / 'metadata.json'
            summary_file = item / 'summary.json'
            entry = {
                'id': item.name,
                'path': str(item),
                'has_summary': summary_file.exists(),
                'has_csv': (item / 'data.csv').exists(),
            }
            if meta_file.exists():
                try:
                    with open(meta_file) as f:
                        meta = json.load(f)
                    entry.update({
                        'controller_name': meta.get('controller_name', ''),
                        'scenario_name':   meta.get('scenario_name', ''),
                        'mode':            meta.get('mode', ''),
                        'timestamp':       meta.get('timestamp', ''),
                        'notes':           meta.get('notes', ''),
                    })
                except Exception:
                    pass
            if summary_file.exists():
                try:
                    with open(summary_file) as f:
                        summary = json.load(f)
                    entry['summary'] = {
                        'duration_s': summary.get('duration_s', 0),
                        'rmse_lateral_error_mm': summary.get('rmse_lateral_error_mm', 0),
                        'mean_cmd_v': summary.get('mean_cmd_v', 0),
                        'tracking_valid_ratio': summary.get('tracking_valid_ratio', 0),
                    }
                except Exception:
                    pass
            experiments.append(entry)

        msg = String()
        msg.data = json.dumps({
            'experiments': experiments,
            'total': len(experiments),
            'base_dir': str(self._base_dir),
        })
        self._pub_exp_list.publish(msg)

    # =========================================================================
    # Load single experiment
    # =========================================================================

    def _load_experiment(self, exp_id: str):
        """Load CSV and compute metrics for one experiment."""
        exp_dir = self._base_dir / exp_id
        csv_path = exp_dir / 'data.csv'
        summary_path = exp_dir / 'summary.json'

        if summary_path.exists():
            # Already computed — just republish
            try:
                with open(summary_path) as f:
                    summary = json.load(f)
                with self._lock:
                    self._loaded_experiments[exp_id] = summary
                msg = String()
                msg.data = json.dumps(summary)
                self._pub_summary.publish(msg)
                return
            except Exception as e:
                self.get_logger().warn(
                    f'[analyzer] Failed to load summary for {exp_id}: {e}')

        if not csv_path.exists():
            self.get_logger().warn(
                f'[analyzer] No CSV found for experiment: {exp_id}')
            return

        try:
            metrics = self._compute_metrics_from_csv(csv_path)
            # Try to load metadata
            meta_path = exp_dir / 'metadata.json'
            if meta_path.exists():
                with open(meta_path) as f:
                    meta = json.load(f)
                metrics['metadata'] = meta

            with self._lock:
                self._loaded_experiments[exp_id] = metrics

            # Save for future use
            try:
                with open(summary_path, 'w') as f:
                    json.dump(metrics, f, indent=2)
            except Exception:
                pass

            msg = String()
            msg.data = json.dumps(metrics)
            self._pub_summary.publish(msg)
            self.get_logger().info(
                f'[analyzer] Loaded experiment: {exp_id}')
        except Exception as e:
            self.get_logger().error(
                f'[analyzer] Failed to analyze {exp_id}: {e}')

    def _compute_metrics_from_csv(self, csv_path: Path) -> dict:
        """Compute all comparison metrics from a CSV file."""
        times, lat_errs, head_errs = [], [], []
        cmd_vs, cmd_omegas = [], []
        odom_xs, odom_ys, front_dists = [], [], []
        lane_timeouts, odom_timeouts, lidar_stops = [], [], []
        lane_valids = []

        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                def g(k, default=0.0):
                    try:
                        return float(row.get(k, default) or default)
                    except (ValueError, TypeError):
                        return default
                times.append(g('time_s'))
                lat_errs.append(g('epsilon_x_mm'))
                head_errs.append(g('theta_rad'))
                cmd_vs.append(g('cmd_v'))
                cmd_omegas.append(g('cmd_omega'))
                odom_xs.append(g('odom_x'))
                odom_ys.append(g('odom_y'))
                front_dists.append(g('front_min_m'))
                lane_timeouts.append(int(g('ref_timeout')))
                odom_timeouts.append(int(g('odom_timeout')))
                lidar_stops.append(int(g('lidar_stop')))
                lane_valids.append(int(float(row.get('lane_valid', 0) or 0)))

        if not times:
            return {}

        # Path length
        path_len = 0.0
        for i in range(1, len(odom_xs)):
            dx = odom_xs[i] - odom_xs[i-1]
            dy = odom_ys[i] - odom_ys[i-1]
            path_len += (dx**2 + dy**2) ** 0.5

        return {
            'duration_s': round(max(times) if times else 0, 2),
            'mean_abs_lateral_error_mm': round(compute_mae(lat_errs), 3),
            'rmse_lateral_error_mm': round(compute_rmse(lat_errs), 3),
            'max_abs_lateral_error_mm': round(max(abs(e) for e in lat_errs), 3),
            'mean_abs_heading_error_rad': round(compute_mae(head_errs), 5),
            'rmse_heading_error_rad': round(compute_rmse(head_errs), 5),
            'mean_cmd_v': round(sum(cmd_vs)/len(cmd_vs) if cmd_vs else 0, 4),
            'mean_abs_cmd_omega': round(compute_mae(cmd_omegas), 5),
            'max_abs_cmd_omega': round(max(abs(o) for o in cmd_omegas), 5),
            'cmd_omega_smoothness': round(compute_smoothness(cmd_omegas), 6),
            'cmd_v_smoothness': round(compute_smoothness(cmd_vs), 6),
            'path_length_m': round(path_len, 3),
            'average_front_distance_m': round(
                sum(front_dists)/len(front_dists) if front_dists else 0, 3),
            'lane_timeout_count': sum(lane_timeouts),
            'odom_timeout_count': sum(odom_timeouts),
            'lidar_stop_count': sum(lidar_stops),
            'tracking_valid_ratio': round(
                sum(lane_valids)/len(lane_valids)*100 if lane_valids else 0, 1),
        }

    # =========================================================================
    # Compare experiments
    # =========================================================================

    def _compare_experiments(self, exp_ids: list, scenario: str):
        """Load multiple experiments and publish comparison."""
        if not exp_ids:
            # Compare all loaded
            with self._lock:
                exp_ids = list(self._loaded_experiments.keys())

        results = []
        for exp_id in exp_ids:
            # Load if not already in memory
            with self._lock:
                metrics = self._loaded_experiments.get(exp_id)

            if metrics is None:
                exp_dir = self._base_dir / exp_id
                csv_path = exp_dir / 'data.csv'
                summary_path = exp_dir / 'summary.json'
                if summary_path.exists():
                    try:
                        with open(summary_path) as f:
                            metrics = json.load(f)
                        with self._lock:
                            self._loaded_experiments[exp_id] = metrics
                    except Exception:
                        pass
                elif csv_path.exists():
                    try:
                        metrics = self._compute_metrics_from_csv(csv_path)
                        with self._lock:
                            self._loaded_experiments[exp_id] = metrics
                    except Exception:
                        pass

            if metrics:
                meta = metrics.get('metadata', {})
                entry = {
                    'experiment_id': exp_id,
                    'controller': meta.get('controller_name', exp_id),
                    'scenario': meta.get('scenario_name', scenario),
                    'mode': meta.get('mode', 'real'),
                }
                entry.update({k: v for k, v in metrics.items()
                               if k not in ('metadata',)})
                results.append(entry)

        comparison = {
            'scenario': scenario,
            'experiment_count': len(results),
            'experiments': results,
        }

        msg = String()
        msg.data = json.dumps(comparison)
        self._pub_comparison.publish(msg)
        self.get_logger().info(
            f'[analyzer] Comparison published: {len(results)} experiments')

    def _export_comparison_csv(self, exp_ids: list):
        """Export a CSV table comparing multiple experiments."""
        self._compare_experiments(exp_ids, 'all')
        # Write CSV to base_dir
        try:
            with self._lock:
                results = []
                for exp_id in exp_ids:
                    m = self._loaded_experiments.get(exp_id, {})
                    if m:
                        meta = m.get('metadata', {})
                        row = {'experiment_id': exp_id}
                        row['controller'] = meta.get('controller_name', '')
                        row['scenario'] = meta.get('scenario_name', '')
                        row.update({k: v for k, v in m.items()
                                    if k not in ('metadata',)})
                        results.append(row)

            if not results:
                return
            out_path = self._base_dir / 'comparison_table.csv'
            keys = list(results[0].keys())
            with open(out_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(results)
            self.get_logger().info(
                f'[analyzer] Comparison CSV exported: {out_path}')
        except Exception as e:
            self.get_logger().error(
                f'[analyzer] Export failed: {e}')


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentAnalyzerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
