# How To Run SITL on Startup

If you've followed the instructions on CARRIER_BOARD_SETUP, you should have both the SITL and MavProxy packages installed.  From here, you will want to auto start the startup/startup.py file on carrier board bootup.  Here is what that file does:

1. It firsts looks to see if you have an instance of Ardupilot connected to the serial port of the Pi by reading all the active serial ports and looking for key words in the name such as Cube, CubeBlack, etc.

2. If it finds an ardupilot, it will not start the SITL as it expects to get all flight data from the instance of Ardupilot (i.e. the Cube hardware).  It then starts an instance of MavProxy that will serve the Ardupilot's data and commands to any service connected via UDP to the --out ports.

3. If it does not find and instance of ardupilot on the serial ports, it will then start the onboard SITL software.  The SITL also autostarts MavProxy and will serve the Ardupilot's data and commands to any service connected via UDP to the --out ports.

### Running an Autostart File in Raspbian/Debian

1. Make sure you are able to see hidden files and folders on the Pi by checking "Show Hidden"

    ![Show Hidden](images/Pi_Hidden_Files.png)

2. Goto the autostart folder in ~/home/swarm-ctl/.config/autostart.  If the autostart folder does not exist in .config, create it.

    ![Show Hidden](images/Pi_Autostart.png)

3. In the autostart folder, create a desktop file named "SITL at Startup.desktop" and put the fillowing lines within the code. 

        [Desktop Entry]  
        Name=SITL at Startup  
        Comment=Starts the SITL if Ardupilot is not connected via serial  
        Exec=python /home/swarm-ctl/agent_core/startup/startup.py  
        Terminal=false  
        Type=Application  
    
    *** Make sure you have the correct path for the Exec= line.  If you cloned your agent_core repository into a different folder, make sure it reflects that in this Exec= line.

4. Now, when you reboot the Pi, the 'SITL at Startup.desktop' shold start the startup.py file that will follow the steps above and start serving your Ardupilot of SITL data through MavProxy to the assigned ports.  From here, agent_core will connect to the ports MavProxy is defining as '--out' and agent_core, Mission Planner, ROS etc. can start talking to the hardware (Cube) or simulator (SITL)