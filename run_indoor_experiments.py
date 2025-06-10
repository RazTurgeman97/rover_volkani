import subprocess
import time
import os

# --- Configuration ---
ROS2_LAUNCH_CMD = "ros2 launch roverrobotics_driver experiments_launch.py"
# Note: greenhouse_experiment_node is set to use_sim_time:=true internally if Nav2 is.
# experiments_launch.py should ensure Nav2 is launched with use_sim_time:=true for simulation.

EXPERIMENT_CONFIGS = [
    # Exp 1: All plants
    {"type": 1, "name": "AllPlants"},
    # Exp 2: Every 2nd plant
    {"type": 2, "name": "Every2ndPlant"},
    # Exp 3: Every Nth plant (N specified by 'n_val')
    # These three will count as the 3 reps for "Experiment 3"
    {"type": 3, "name": "EveryNthPlant_N2", "n_val": 2},
    {"type": 3, "name": "EveryNthPlant_N3", "n_val": 3},
    {"type": 3, "name": "EveryNthPlant_N4", "n_val": 4}, # Using N=4 as an example for the third variation
    # Exp 4: Pre-determined infected list (specified by 'infected_list')
    # These three will count as the 3 reps for "Experiment 4"
    {"type": 4, "name": "Predetermined_List1", "infected_list": "1,5,10"},
    {"type": 4, "name": "Predetermined_List2", "infected_list": "2,7,12"},
    {"type": 4, "name": "Predetermined_List3", "infected_list": "3,8,14"},
    # Exp 5: Real-time infected plants
    # The 3 reps for Exp 5 will be handled by running this config 3 times,
    # requiring manual/external data push for each.
    {"type": 5, "name": "RealTimeInfected"},
]

# For experiments 1, 2, 5, we need 3 repetitions each.
# For experiments 3 and 4, the variations above ALREADY constitute their "3 repetitions".
# So, we'll iterate through IMU conditions, then through the EXPLICIT experiment_configs list.

IMU_CONDITIONS = [True, False] # True for IMU enabled, False for IMU disabled
TOTAL_REPETITIONS_FOR_TYPE_1_2_5 = 3 # Renamed for clarity

# --- Main Loop ---
def run_experiment(exp_config, use_imu, repetition_num=None):
    exp_type = exp_config["type"]
    exp_name = exp_config["name"]
    
    # Build the launch command
    cmd = f"{ROS2_LAUNCH_CMD} "
    cmd += f"experiment_type:={exp_type} "
    cmd += f"use_imu:={'true' if use_imu else 'false'} "

    # Add experiment-specific parameters
    if exp_type == 3: # Every Nth
        cmd += f"n_plants:={exp_config['n_val']} "
    elif exp_type == 4: # Pre-determined list
        cmd += f"infected_plants_list_str:='{exp_config['infected_list']}' "

    run_label = f"RUN: IMU={'Enabled' if use_imu else 'Disabled'}, ExpName={exp_name}"
    if repetition_num:
        run_label += f", Repetition={repetition_num}"
    
    print(f"\n----------------------------------------------------")
    print(run_label)
    print(f"Executing: {cmd}")
    print(f"----------------------------------------------------")

    process = None
    try:
        # Start the ROS2 launch process
        # Using shell=True can be a security risk if cmd were from untrusted input,
        # but here it's constructed internally. Using it for simpler command parsing with quotes.
        process = subprocess.Popen(cmd, shell=True, text=True)

        if exp_type == 5:
            print("\n**********************************************************************************")
            print(f"EXPERIMENT TYPE 5 (Real-time) for {run_label} HAS STARTED.")
            print("The robot is now waiting for plant IDs on the /infected_plant_notification topic.")
            print("Use a separate terminal or script to publish messages of type std_msgs/Int32MultiArray.")
            print("Example: ros2 topic pub --once /infected_plant_notification std_msgs/msg/Int32MultiArray '{data: [1,2,3]}'")
            input("PRESS ENTER HERE ONCE YOU HAVE COMPLETED PUSHING DATA FOR THIS RUN and the robot has finished its tasks...")
            print("**********************************************************************************\n")
        else:
            # For other experiments, the C++ node will run to completion and exit.
            # Wait for the process to complete.
            process.wait() 
            print(f"Experiment run {run_label} completed.")

    except subprocess.CalledProcessError as e:
        print(f"Error during experiment run {run_label}: {e}")
    except KeyboardInterrupt:
        print(f"Experiment run {run_label} interrupted by user. Terminating...")
        if process:
            process.terminate()
            process.wait()
        raise # Re-raise KeyboardInterrupt to stop the whole script
    finally:
        if process and process.poll() is None: # If process is still running (e.g. for Exp5)
            print(f"Terminating process for {run_label}...")
            process.terminate()
            try:
                process.wait(timeout=10) # Wait a bit for graceful termination
            except subprocess.TimeoutExpired:
                print(f"Process for {run_label} did not terminate gracefully, killing.")
                process.kill()
            print(f"Process for {run_label} terminated.")
        time.sleep(5) # Cooldown period before next run

if __name__ == "__main__":
    overall_run_count = 0
    for use_imu in IMU_CONDITIONS:
        for config in EXPERIMENT_CONFIGS:
            # Experiments 3 and 4 have their "repetitions" baked into the config list variations.
            # They are run once per IMU setting for that specific variation.
            if config["type"] == 3 or config["type"] == 4:
                overall_run_count += 1
                print(f"\nStarting Overall Run #{overall_run_count} for config: {config['name']}...")
                run_experiment(config, use_imu)
            # Experiments 1, 2, and 5 need explicit repetition loops.
            else: 
                for i in range(1, TOTAL_REPETITIONS_FOR_TYPE_1_2_5 + 1):
                    overall_run_count += 1
                    # Construct a unique name for logging if needed, or pass repetition_num for run_label
                    # current run_label already includes repetition_num if provided
                    print(f"\nStarting Overall Run #{overall_run_count} for config: {config['name']}, Repetition {i}...")
                    run_experiment(config, use_imu, repetition_num=i)
    
    print("\n\nAll indoor experiments complete!")
