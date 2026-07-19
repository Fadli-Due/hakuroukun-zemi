#!/usr/bin/env python3
import rospy
import subprocess
import os
import sys

class MapManager:
    def __init__(self):
        rospy.init_node('map_manager', anonymous=True)
        
        # Get directory paths
        self.home_dir = os.path.expanduser('~')
        self.default_map_path = os.path.join(self.home_dir, 'map_df_area')
        
        self.slam_process = None
        self.start_slam()

    def start_slam(self):
        rospy.loginfo("[Map Manager] Starting Gmapping SLAM process...")
        try:
            # Launch your custom package and mapping configuration
            self.slam_process = subprocess.Popen(
                ["roslaunch", "hakuroukun_boustrophedon_with_cones", "df_mapping.launch"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            rospy.loginfo("[Map Manager] SLAM successfully started.")
        except Exception as e:
            rospy.logerr(f"[Map Manager] Failed to start SLAM launch file: {e}")
            sys.exit(1)

    def save_map(self, filename):
        full_path = os.path.join(self.home_dir, filename)
        rospy.loginfo(f"[Map Manager] Saving map to: {full_path}.yaml/.pgm ...")
        
        try:
            # Invokes map_saver from map_server
            result = subprocess.run(
                ["rosrun", "map_server", "map_saver", "-f", full_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            if result.returncode == 0:
                rospy.loginfo(f"[Map Manager] Map successfully saved at {full_path}")
            else:
                rospy.logerr(f"[Map Manager] Map saver returned error: {result.stderr}")
        except subprocess.TimeoutExpired:
            rospy.logerr("[Map Manager] Map saver timed out.")
        except Exception as e:
            rospy.logerr(f"[Map Manager] Error running map_saver: {e}")

    def run(self):
        print("\n" + "="*50)
        print("                 MAP MANAGER ACTIVE")
        print("="*50)
        print("  S : Save Map (defaults to ~/map_df_area)")
        print("  C : Save Map with Custom Name")
        print("  Q : Quit and Close SLAM")
        print("="*50 + "\n")

        while not rospy.is_shutdown():
            try:
                user_input = input("Enter command [S, C, Q]: ").strip().lower()
            except KeyboardInterrupt:
                break

            if user_input == 's':
                self.save_map('map_df_area')
            elif user_input == 'c':
                custom_name = input("Enter custom filename: ").strip()
                if custom_name:
                    self.save_map(custom_name)
                else:
                    print("Invalid filename.")
            elif user_input == 'q':
                rospy.loginfo("[Map Manager] Shutting down SLAM process...")
                break
            else:
                print("Unknown command.")

        self.shutdown_cleanup()

    def shutdown_cleanup(self):
        if self.slam_process:
            self.slam_process.terminate()
            try:
                self.slam_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.slam_process.kill()
        rospy.loginfo("[Map Manager] Cleanup complete.")


if __name__ == '__main__':
    try:
        manager = MapManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass