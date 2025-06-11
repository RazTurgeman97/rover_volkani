#!/usr/bin/env python3
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess
import threading
import time

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition


class ExperimentManager(LifecycleNode):
    def __init__(self):
        super().__init__('experiment_manager')

        self.declare_parameter(
            'ros2_launch_cmd',
            'ros2 launch roverrobotics_driver experiments_launch.py'
        )

        self.experiment_configs = [
            {'type': 1, 'name': 'AllPlants'},
            {'type': 2, 'name': 'Every2ndPlant'},
            {'type': 3, 'name': 'EveryNthPlant_N2', 'n_val': 2},
            {'type': 3, 'name': 'EveryNthPlant_N3', 'n_val': 3},
            {'type': 3, 'name': 'EveryNthPlant_N4', 'n_val': 4},
            {'type': 4, 'name': 'Predetermined_List1', 'infected_list': '1,5,10'},
            {'type': 4, 'name': 'Predetermined_List2', 'infected_list': '2,7,12'},
            {'type': 4, 'name': 'Predetermined_List3', 'infected_list': '3,8,14'},
            {'type': 5, 'name': 'RealTimeInfected'},
        ]

        self.imu_conditions = [True, False]
        self.total_reps = 3
        self.process = None
        self.thread = None

    # Lifecycle callbacks -------------------------------------------------
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring ExperimentManager')
        self.launch_cmd = self.get_parameter('ros2_launch_cmd').get_parameter_value().string_value
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating ExperimentManager')
        self.thread = threading.Thread(target=self.run_all_experiments)
        self.thread.start()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating ExperimentManager')
        if self.process and self.process.poll() is None:
            self.get_logger().info('Terminating running process')
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------------
    def run_all_experiments(self):
        overall_run = 0
        for use_imu in self.imu_conditions:
            for config in self.experiment_configs:
                if config['type'] in (3, 4):
                    overall_run += 1
                    self.get_logger().info(
                        f'Starting Run #{overall_run} {config["name"]} IMU={use_imu}'
                    )
                    self.run_single(config, use_imu)
                else:
                    for rep in range(1, self.total_reps + 1):
                        overall_run += 1
                        self.get_logger().info(
                            f'Starting Run #{overall_run} {config["name"]} IMU={use_imu} Rep={rep}'
                        )
                        self.run_single(config, use_imu, repetition=rep)
        self.get_logger().info('All experiments complete')
        # Transition back to inactive when done
        self.trigger_deactivate()

    def run_single(self, exp_config, use_imu, repetition=None):
        exp_type = exp_config['type']
        cmd = [self.launch_cmd]
        cmd.append(f'experiment_type:={exp_type}')
        cmd.append(f"use_imu:={'true' if use_imu else 'false'}")

        if exp_type == 3:
            cmd.append(f'n_plants:={exp_config["n_val"]}')
        elif exp_type == 4:
            cmd.append(f"infected_plants_list_str:='{exp_config['infected_list']}'")

        cmd_str = ' '.join(cmd)
        self.get_logger().info(f'Executing: {cmd_str}')
        self.process = subprocess.Popen(cmd_str, shell=True, text=True)

        if exp_type == 5:
            self.get_logger().info('Waiting for real-time infected plant data...')
            input('Press ENTER when run is complete...')
        else:
            self.process.wait()

        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()

        time.sleep(5)

    # Helper to trigger deactivate transition
    def trigger_deactivate(self):
        self.get_logger().info('Requesting deactivate transition')
        super().trigger_deactivate()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
