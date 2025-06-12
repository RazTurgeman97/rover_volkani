#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import os, glob
import pandas as pd
import matplotlib.pyplot as plt
import yaml
from ament_index_python.packages import get_package_share_directory

class ExperimentMonitor(Node):
    def __init__(self):
        super().__init__('experiment_monitor')
        self.get_logger().info('Starting ExperimentMonitor…')

        # 1) which experiment?
        self.declare_parameter('experiment_type', 1)
        self.exp_type = int(self.get_parameter('experiment_type').value or 1)
        self.get_logger().info(f'Experiment type = {self.exp_type}')

        # 2) locate logs
        share = get_package_share_directory('experiment_monitor')
        self.exp_dir = os.path.join(share, 'logs', f'experiment_{self.exp_type}')
        self.get_logger().info(f'Looking for attempts under: {self.exp_dir}')

        # 3) load Nav2 tolerance
        #    assumes nav2_params.yaml under roverrobotics_driver/config
        drv_share = get_package_share_directory('roverrobotics_driver')
        navf = os.path.join(drv_share, 'config', 'nav2_params.yaml')
        try:
            nav = yaml.safe_load(open(navf))
            tol = nav['controller_server']['ros__parameters']['precise_goal_checker']['xy_goal_tolerance']
            self.goal_tol = float(tol)
            self.get_logger().info(f'Loaded goal tolerance = {self.goal_tol:.3f} m')
        except Exception as e:
            self.get_logger().warning(f'Could not load tolerance from {navf}: {e}')
            self.goal_tol = None

        # 4) process everything
        self.process_all_attempts()
        self.get_logger().info('All done, shutting down.')

    def process_all_attempts(self):
        # gather attempt dirs
        dirs = glob.glob(os.path.join(self.exp_dir,'attempt_*'))
        def idx(d): return int(os.path.basename(d).split('_')[1])
        attempts = sorted(dirs, key=idx)
        if not attempts:
            self.get_logger().warning('No attempts found.')
            return

        summaries = []
        for d in attempts:
            n = idx(d)
            fn = os.path.join(d,'results.csv')
            if not os.path.isfile(fn):
                self.get_logger().warning(f'missing results.csv in {d}')
                continue

            df = pd.read_csv(fn)
            self.get_logger().info(f'Loaded attempt {n}: {len(df)} rows')

            # detect outliers
            mu, sigma = df['duration_s'].mean(), df['duration_s'].std()
            thresh = mu + 2*sigma
            out = df[df['duration_s']>thresh]['plant'].tolist()
            if out:
                self.get_logger().warning(
                    f'Attempt {n} duration outliers (>{thresh:.1f}s): plants {out}'
                )

            # summary
            summary = {
                'attempt':          n,
                'total_duration_s': df['duration_s'].sum(),
                'mean_duration_s':  df['duration_s'].mean(),
                'mean_error_m':     df['error_m'].mean(),
                'success_rate':     df['success'].mean(),
            }
            # Add new fields to summary - assuming these are constant per attempt/results.csv file
            if not df.empty:
                # Pandas might read these as int (0/1 for bool), which is fine for CSV.
                # If specific bool type is needed later, it can be cast.
                if 'experiment_type' in df.columns:
                    summary['experiment_type'] = df['experiment_type'].iloc[0]
                if 'imu_enabled' in df.columns:
                    summary['imu_enabled'] = df['imu_enabled'].iloc[0] 
                if 'param_n' in df.columns:
                    summary['param_n'] = df['param_n'].iloc[0]
                if 'predetermined_list' in df.columns:
                    # Ensure we handle potential NaN if the column exists but value is missing for some reason
                    summary['predetermined_list'] = df['predetermined_list'].iloc[0] if pd.notna(df['predetermined_list'].iloc[0]) else ""

            summaries.append(summary)
            # Ensure all summary dicts have the same keys for DataFrame creation, even if some are None/NaN
            # This is implicitly handled if keys are added consistently above.
            pd.DataFrame([summary]).to_csv(os.path.join(d,'summary.csv'), index=False)
            self.get_logger().info(f' Wrote summary.csv in attempt_{n}')

            # duration bar + highlight outliers
            plt.figure()
            bars = df.plot.bar(x='plant', y='duration_s', legend=False).patches
            for patch, plant in zip(bars, df['plant']):
                if plant in out:
                    patch.set_color('r')
            plt.ylabel('Duration (s)')
            plt.title(f'Attempt {n} Task Duration')
            plt.tight_layout()
            p1 = os.path.join(d,'duration.png')
            plt.savefig(p1); plt.clf()
            self.get_logger().info(f' Wrote {p1}')

            # error bar + tolerance line
            plt.figure()
            df.plot.bar(x='plant', y='error_m', legend=False)
            if self.goal_tol is not None:
                plt.axhline(self.goal_tol, color='r', linestyle='--',
                            label=f'tolerance={self.goal_tol:.2f}')
                plt.legend()
            plt.ylabel('Error (m)')
            plt.title(f'Attempt {n} Stopping Error')
            plt.tight_layout()
            p2 = os.path.join(d,'error.png')
            plt.savefig(p2); plt.clf()
            self.get_logger().info(f' Wrote {p2}')

            # success → pie chart
            plt.figure()
            counts = df['success'].value_counts().sort_index()
            # Dynamically set labels to match counts' indices
            label_map = {0: 'Fail', 1: 'OK'}
            labels = [label_map.get(idx, str(idx)) for idx in counts.index]
            plt.pie(counts, labels=labels, autopct='%d', startangle=90)
            plt.title(f'Attempt {n} Success Rate')
            plt.tight_layout()
            p3 = os.path.join(d,'success.png')
            plt.savefig(p3); plt.clf()
            self.get_logger().info(f' Wrote {p3}')

        # write comparison.csv
        comp = pd.DataFrame(summaries).sort_values('attempt')
        cmpf = os.path.join(self.exp_dir,'comparison.csv')
        comp.to_csv(cmpf, index=False)
        self.get_logger().info(f'Updated comparison.csv with attempts {[s["attempt"] for s in summaries]}')

        # stats if ≥3
        if len(summaries)>=3:
            stats = comp[['mean_duration_s','mean_error_m','success_rate']].agg(['mean','std'])
            sp = os.path.join(self.exp_dir,'stats.csv')
            stats.to_csv(sp)
            self.get_logger().info(f'Wrote stats.csv (mean/std) based on comparison.csv')

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()