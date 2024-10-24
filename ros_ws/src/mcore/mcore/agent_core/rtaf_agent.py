import sys
from classes.custom_agent_command_mgr import CustomAgentCommandManager  # Import your custom command manager
from classes.agent_core import AgentCore



class Agent(AgentCore):
    


    def agent_core_loop_functions(self):

        # Simulated placeholders for agent_hub and mav_connection (replace these with actual objects in your code)
        config = self.config
        agent_hub = self.agent_hub
        mav_connection = self.agent_hub.mav_mgr.mav_connection

        # Initialize the command manager
        command_manager = CustomAgentCommandManager(config, agent_hub, mav_connection, verbose=True)

        # Prompt user for action
        print("\nSelect an action:")
        print("1. Arm")
        print("2. Disarm")
        print("3. Set Mode")
        print("4. Takeoff")
        print("5. Move")
        print("6. Go to GPS Coordinates")
        print("7. Change Speed")
        print("8. Rotate (Yaw)")
        print("9. Start Mission")
        print("0. Exit")

        choice = input("Enter the number of the action you want to perform: ").strip()

        if choice == '1':
            # Arm the vehicle
            override_safety = input("Override safety? (yes/no): ").strip().lower() == 'yes'
            delay_before_disarm = int(input("Enter delay before disarm (in seconds, default is 0): ").strip() or 0)
            command_manager.arm(override_safety=override_safety, delay_before_disarm=delay_before_disarm)
        
        elif choice == '2':
            # Disarm the vehicle
            command_manager.disarm()
        
        elif choice == '3':
            # Set flight mode
            mode = input("Enter flight mode (e.g., GUIDED, STABILIZE): ").strip()
            command_manager.set_mode(mode)

        elif choice == '4':
            # Takeoff
            altitude = float(input("Enter altitude for takeoff (in meters): ").strip())
            command_manager.takeoff(altitude=altitude)
        
        elif choice == '5':
            # Move to relative position
            x = float(input("Enter X distance (forward/backward in meters): ").strip())
            y = float(input("Enter Y distance (left/right in meters): ").strip())
            z = input("Enter Z altitude (or leave blank to maintain current altitude): ").strip()
            z = float(z) if z else None
            command_manager.move(x, y, z)

        elif choice == '6':
            # Go to specific GPS coordinates
            latitude = float(input("Enter latitude: ").strip())
            longitude = float(input("Enter longitude: ").strip())
            altitude = input("Enter altitude (or leave blank to use current altitude): ").strip()
            altitude = float(altitude) if altitude else None
            command_manager.goto(latitude, longitude, altitude)

        elif choice == '7':
            # Change speed
            speed = float(input("Enter speed (in m/s): ").strip())
            command_manager.change_speed(speed)

        elif choice == '8':
            # Rotate (yaw)
            angle = float(input("Enter yaw angle (in degrees): ").strip())
            direction = input("Rotate clockwise? (yes/no): ").strip().lower()
            clockwise = direction == 'yes'
            command_manager.yaw(angle, clockwise=clockwise)

        elif choice == '9':
            # Start mission
            command_manager.start_mission()

        elif choice == '0':
            print("Exiting program.")
            sys.exit()

        else:
            print("Invalid choice. Please select a valid action.")

if __name__ == "__main__":
    # loop delay allows you to put a delay in the agent's main loop
    # otherwise it loops as fast as the processor will allow
    my_agent = Agent(loop_delay=0, verbose=False)
