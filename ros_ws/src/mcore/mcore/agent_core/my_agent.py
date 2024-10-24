from classes.agent_core import AgentCore
from classes.ops import operation
import time


class Agent(AgentCore):

    # Overrides AgentCore class continuous loop function that
    # 1. listens for flight mode changes
    # 2. listens for arm status changes
    # 3. listens for postion changes
    # 4. listens for external commands then sends commands to the Ardupilot as desired.

    ### Available Commands ###

    # self.start_logging / self.stop_logging - used to start and stop writing parameters to a log file
    #   The file is located in the Agent Core root directory under logs
    #   The agent_configuration MESSAGE_INTERVALS->[MESSAGE]->LOG_INTERVAL defines how often that message is logged

    # 


    ############### Put your logic in this loop ###############
    def agent_core_loop_functions(self):

        flight_mode = self.agent_status_obj.flight_mode         # string with named flight mode
        arm_status = self.agent_status_obj.arm_status           # int with arm status
        position = vars(self.agent_status_obj.agent_position)   # dict with postion params

        ### Ask for status changes from the agent's status object ###
        if self.agent_status_obj.check_for_flight_mode_changed():
            print("The new flight mode = " + flight_mode)


        ### Ask for arm status changes from the agent's status object ###
        if self.agent_status_obj.check_for_arm_status_changed():
            print("The new arm status = " + str(arm_status))

            # example of managing the logging function
            if arm_status == 2:
                self.stop_logging()
            elif arm_status == 4:
                self.start_logging()

        ### Ask for position changes from the agent's status object ###
        #if self.agent_status_obj.check_for_position_change():
            # print("The new position = " + str(position))
        #    pass

            ### Scott's custom fly-to trigger and command
           # if self.agent_status_obj.agent_position.relative_alt > 5.0:
           #     destination = {'lat': 39.0174374, 'lon': -104.8931801, 'relative_alt': 15.0}
                # destination = {'lat': 39.0174374, 'lon': -104.8931801, 'alt': 2170.0}
           #      cmd = {'goto': destination}
           #     self.agent_command_mgr.process_command(cmd)

            # if agent_status_obj.agent_position.lat < 39.01733:
            #     agent_command_mgr.process_command("crc")
        myin = input("What do you want to do? Press:\n 1 for arm\n 2 for takeoff\n 3 to move\n 4 to yaw\n 5 to send an arbitrary target\n 6 to RTL\n ")

        if myin == "1":
            cmd = 'arm'
            self.agent_command_mgr.send_arm_disarm_command(cmd)
        elif myin == "2":
            cmd="GUIDED"
            self.agent_command_mgr.process_command(cmd) # guided mode
            time.sleep(0.5)
            cmd={'takeoff': {'takeoff_height':'10'}}
            self.agent_command_mgr.process_command(cmd)
            time.sleep(7)
            # todo: check takeoff status/height
        elif myin == "3": # need this to be move velocity
            x = int(input("Input distance to move in x direction: "))
            y = int(input("Input distance to move in y direction: "))
            self.agent_command_mgr.send_move_command(x,y)
            time.sleep(5)
        elif myin == "4":
            self.agent_command_mgr.send_yaw_command(90)
            time.sleep(5)
        elif myin == "5":
            destination = {'lat': 39.0174374, 'lon': -104.8931801, 'relative_alt': 15.0}
            cmd = {'goto': destination}
            self.agent_command_mgr.process_command(cmd)
            time.sleep(5)
        elif myin == 6:
            print("RTL initiated")
            self.agent_command_mgr.send_mode_change_to_flight_controller(6) # RTL mode
        elif myin =="9":
            self.agent_command_mgr.send_new_speed(20)
            time.sleep(5)
        elif myin == "special":
            input("Ready?")
            x,y = operation()
            print("X is" + str(x) + "Y is " + str(y))
            input("Press any to continue")
            self.agent_command_mgr.send_move_command(x,y)
            time.sleep(5)
        elif myin == "yaw11":
            self.agent_command_mgr.send_yaw_command(45)
            time.sleep(5)
        elif myin == "fwd":
            for i in range(20):
                self.agent_command_mgr.send_move_command2()
                time.sleep(1)

        else:
            pass

        ### Ask for incoming command messages from agent hub ###
        ### The parenthetical is the time in msec to wait for a command ###
        ### This will pause this loop for that amount of time to listen ###
        command_message = self.agent_hub.check_for_command_messages(50)
        if command_message:
            print(command_message)
            self.agent_command_mgr.process_command(command_message)


# Initialize the agent
if __name__ == "__main__":
    # loop delay allows you to put a delay in the agent's main loop
    # otherwise it loops as fast as the processor will allow
    my_agent = Agent(loop_delay=0, verbose=False)
