# Setting Up the Carrier Board for Simulation

The Ardupilot Simulator (SITL) can be run directly on the carrier-board (such as a Pi) versus on an external pc/laptop.  This allows you to use onboard software scripts with an onboard simulator which removes the requirement for Mission Planner to generate SITL instances offboard as well as an off-board instance of MavProxy to connect the simulators to the code running on the carrier boards.  By adding the SITL to the carrier board in conjunction with running MavProxy on each carrier-bard, you can then have multiple processes/tasks running on the carrier-board that connects to either the Cube if it's connected or the SITL if it's running.  Below gives the steps for setting up both the SITL and MavProxy on the Pi as well as how to connect it to processes like agent_core.  This should work for most/all Linux based carrier board distributions.

### Install SITL on Carrier Board

1. In the folder where you want to install the SITL:

    (assuming git is alread installed on the carrier board and you are connected to the wwww)
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot
    
    cd ardupilot

In case some firewalls do not allow ssh access which can cause the above submodule updates to fail, in this instance you can tell git to unilaterally use https through the following command:

    gitconfig--globalurl."https://"

2. Install the required prerequisites:

    (Pi-Linux) Tools/environment_install/install-prereqs-arch.sh -y
    (Ubuntu) Tools/environment_install/install-prereqs-ubuntu.sh -y

3. Install the following libraries

    python -m pip install empy
    pip install pymavlink
    pip install future

4. Build the SITL (This could take a while)

    ./waf configure --board sitl 
    ./waf copter
    ./waf clean 

5. Rewrite the available preset starting locations in the ardupilot/Tools/autotest/locations.txt file by replacing the content with the following:

#NAME=latitude,longitude,absolute-altitude,heading  
USAFA_01=39.018782,-104.893970,2160,180  
USAFA_02=39.018782,-104.893769,2159,180  
USAFA_03=39.018775,-104.893557,2159,180  
USAFA_04=39.018775,-104.893347,2158,180  
USAFA_05=39.018775,-104.893126,2157,180  
USAFA_06=39.018774,-104.892907,2156,180  
USAFA_07=39.018770,-104.892700,2155,180  
USAFA_08=39.018774,-104.892483,2154,180  
USAFA_09=39.018674,-104.893970,2160,180  
USAFA_10=39.018675,-104.893764,2159,180  
USAFA_11=39.018675,-104.893559,2159,180  
USAFA_12=39.018677,-104.893334,2158,180  
USAFA_13=39.018674,-104.893119,2157,180  
USAFA_14=39.018681,-104.892914,2156,180  
USAFA_15=39.018679,-104.892700,2155,180  
USAFA_16=39.018675,-104.892474,2154,180  

** You can add additional custom locations by appending [name]=[lat],[lon],[alt(m)],[heading] to the list above

![Alt text](images/USAFA_Launch_Points.png)

### Install MavProxy

sudo pip install MAVProxy

### Start SITL and MavProxy

1. python ardupilot/Tools/autotest/sim_vehicle.py -v Arducopter --no-mavproxy -L USAFA_01

    This will start the SITL on the carrier board without starting the default mavproxy.  It will place it at USAFA_01.  You can choose any of the locations on the above list.

2. mavproxy.py --master=tcp:127.0.0.1:5760 --out=udpin:0.0.0.0:14550 --out=udpout:127.0.0.1:14551  --streamrate=-1

    This will start an instance of mavproxy that will:
    1. connect to the SITL as the master data source
    2. output via udpin to the local machine's 14550 port.  Mission Planner will connect here.
    3. output via udpout to the local machine's 14551 port.  This is to interact with agent_core.

    -- You can add as many out= lines with unique port numbers if you want multiple processes to interact with the SITL

### Using a Cube/Flight Control Hardware

If you are connected directly to a Cube or any Ardupilot hardware, you will not define the --master as the SITL.  Instead, you can remove the entire --master parameter and mavproxy 'should' recognize the serially connected flight controller and use it as the master while maintaining output ports to mission planner and agent_core.
