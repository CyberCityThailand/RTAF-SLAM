import os
import time
import threading
import serial

import serial.tools.list_ports


def start_sitl():
    print("Here")
    os.system('lxterminal '
              '-e '
              'python '
              '~/AgentCore/SITL/ardupilot/Tools/autotest/sim_vehicle.py '
              '-v ArduCopter '
              '-L USAFA_01 '
              # '--no-mavproxy '
              # out=udpin allows external devices like
              # mavproxy and mission planner to port into this
              # device meaning this device does
              # not need to know the ip address of the ground station.
              # The ground station must know the ip address of this device.
              # The additional --out=udp:127.0.0.1:ports
              # allows processes on this device to access
              # the mavlink datastream through the onboard instance of mavproxy
              '--out=udpin:0.0.0.0:14101 --out=udp:127.0.0.1:14551'
              )


def start_mavproxy(serial_port):

    mavproxy_string = str('lxterminal -e mavproxy.py --master=' +
                          str(serial_port.port) +
                          ' --out=udpin:0.0.0.0:14101 ' +
                          '--out=udp:127.0.0.1:14551')
    os.system(mavproxy_string)


def start_agent_core():

    os.system('lxterminal '
              '-e '
              'python ~/AgentCore/agent_core/agent_core.py'
              )


def check_for_ardupilot():

    ports = list(serial.tools.list_ports.comports())
    port_connected = False

    for p in ports:

        print(p)
        if (any(flt_ctlr in p.description for flt_ctlr in
                ["CubeBlack", "Cube", "CUBE", "USB Serial Device"]) and
                port_connected is False):
            print("######################## "
                  "Establishing Connection to Cube "
                  "##############################")
            serial_port = serial.Serial(p.device)
            serial_port.close()
            port_connected = True
            mavproxy_thread = threading.Thread(
                target=start_mavproxy, args=(serial_port,)
                )
            mavproxy_thread.start()

    # No Ardupilot, start SITL
    if not port_connected:
        print("No port - start sitl")
        sitl_thread = threading.Thread(target=start_sitl)
        sitl_thread.start()


def main():
    check_for_ardupilot()
    time.sleep(45)
    agent_core_thread = threading.Thread(target=start_agent_core)
    agent_core_thread.start()


if __name__ == "__main__":
    main()
