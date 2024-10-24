import time
import sys
import socket
from threading import Event

from queue import Queue
import netifaces as ni

import zmq

from .agent_status_class import AgentStatus
from .mavlink_manager import MavlinkManager
from .agent_heartbeat_mgr_class import AgentHeartbeatManager


def setup_agent_status(config, agent_status_obj):

    '''
    Configures the agent_status_object' AGENT_SERVER_PORT parameter with the 
    appropriate port information based on what's connected to the Pi

    -- On a Pi
    First look for an ethernet port on the pi (i.e. to the microhard)
    Second look for a wifi (wlan) connection on the pi
    -- On a development machine
    Third, look for a wifi on the PC (running in development on VSCode)
    else (based on the config file, it's all running on the dev PC as localhost)
    '''

    # agent_id = randint(10000, 99999)
    agent_id = config['AGENT_ID']
    agent_id_filter = sys.argv[2] if len(sys.argv) > 2 else str(agent_id)
    agent_status_obj.agent_id = agent_id_filter
    agent_status_obj.agent_server_port = config['AGENT_SERVER_PORT']

    if config["WORKERS"]["UI_WORKER"]["IP"] != "localhost":
        connected = False
        while not connected:
            # Try an ethernet connection (i.e. over microhard)
            try:
                ip = ni.ifaddresses('eth0')[ni.AF_INET][0]['addr']
                agent_status_obj.agent_server_host = ip
                connected = True
                print(ip)
                time.sleep(5)
            # No ethernet wire connection
            except:
                print("No Ethernet Comm Port")
                # Try to connect over linux wifi
                try:
                    ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
                    print("The Linux IP address = " + str(ip))
                    agent_status_obj.agent_server_host = ip
                    connected = True
                    time.sleep(5)
                # Not a Linux machine, try to connect over Windows wifi
                except:
                    print("No Linux WiFi Comm Port")
                    try:
                        hostname=socket.gethostname()
                        IPAddr = socket.gethostbyname(hostname)
                        print(IPAddr)
                        agent_status_obj.agent_server_host = IPAddr
                        connected = True
                        time.sleep(5)
                    except:
                        pass

    else:
         print("UI worker is on a localhost")
         agent_status_obj.agent_server_host = "localhost"

    agent_status_obj.agent_server_port = config['AGENT_SERVER_PORT']
    # agent_status_obj.agent_publisher_port = config['AGENT_PUBLISHER_PORT']

class AgentHub():

    def __init__(self, config, agent_status_obj: AgentStatus, verbose=False):

        # class settings and parameters
        self._config = config
        self.agent_status_obj: AgentStatus = agent_status_obj
        self._verbose = verbose
        self._agent_direct_request_sock = None

        setup_agent_status(self._config, self.agent_status_obj)
        self.initialize_agent_hub_zmq_connections()

        # Initialize the threads that listen for the heatbeat messages from the worker
        # and the messages from the ArduPilot
        self.initialize_agent_core_classes(config)


    def vprint(self, print_string):
        if self._verbose:
            print(print_string)

    def initialize_agent_hub_zmq_connections(self):

        ######## Setup ZMQ Connections #########

        # Setup the zmq context and initialize the zmq parameters

        ### Mavlink Manager ###
        self.mavlink_context = zmq.Context(1)
        self.mavlink_manager_messenger_binding = self.mavlink_context.socket(zmq.PAIR)
        self.mavlink_manager_messenger_binding.bind("inproc://mavlink_manager_message")
        self.mavlink_bind_poller = zmq.Poller()
        self.mavlink_bind_poller.register(self.mavlink_manager_messenger_binding, zmq.POLLIN)
        # self.mavlink_bind_socket = dict(self.mavlink_bind_poller.poll(100))
        
        self.mavlink_manager_messenger_connection = self.mavlink_context.socket(zmq.PAIR)
        self.mavlink_manager_messenger_connection.connect("inproc://mavlink_manager_message")
        self.mavlink_connect_poller = zmq.Poller()
        self.mavlink_connect_poller.register(self.mavlink_manager_messenger_connection, zmq.POLLIN)
        # self.mavlink_connect_socket = dict(self.mavlink_connect_poller.poll(100))

        ### Heartbeat to Agent Core Manager ###
        ##### in the agent_heartbeat class/thread #####
        self.network_heartbeat_context = zmq.Context(2)
        self.network_heartbeat_binding = self.network_heartbeat_context.socket(zmq.PAIR)
        self.network_heartbeat_binding.bind("inproc://network_heartbeat_thread_message")
        self.network_heartbeat_poller = zmq.Poller()
        self.network_heartbeat_poller.register(self.network_heartbeat_binding, zmq.POLLIN)
        
        ##### In the agent_core.py #####
        self.network_heartbeat_connection = self.network_heartbeat_context.socket(zmq.PAIR)
        self.network_heartbeat_connection.connect("inproc://network_heartbeat_thread_message")
        self.network_heartbeat_connection_poller = zmq.Poller()
        self.network_heartbeat_connection_poller.register(self.network_heartbeat_connection, zmq.POLLIN)
        # self.agent_core_network_heartbeat_connection_socket = dict(self.agent_core_network_heartbeat_connection_poller.poll(100))
        
        ### Heartbeat to UI Worker Manager ###
        ##### in the agent_heartbeat class/thread #####
        self.status_heartbeat_to_worker_message = self.network_heartbeat_context.socket(zmq.REQ)
        self.status_heartbeat_to_worker_connection_address = (
            str("tcp://" + self._config['WORKERS']['UI_WORKER']['IP'] +
            ":" + self._config['WORKERS']['UI_WORKER']['HEARTBEAT_REPLY_PORT']))
        self.vprint("The worker connection address = " + self.status_heartbeat_to_worker_connection_address)
        self.status_heartbeat_to_worker_message.connect(self.status_heartbeat_to_worker_connection_address)
        self.network_heartbeat_poller.register(self.status_heartbeat_to_worker_message, zmq.POLLIN)

        # Establish the Queue that tells the heartbeat to immediately send a message
        self.immediate_response_queue = Queue()
        self.immediate_response_queue.put(True)

        ### Agents Req/Rep Server ###
        self.agent_sever_context = zmq.Context(3)
        self.request_to_agent_server_messenger_binding = self.agent_sever_context.socket(zmq.REP)
        self.request_to_agent_server_messenger_binding.bind("tcp://*:" + str(self.agent_status_obj.agent_server_port))


        self.worker_dict = {}
        self.workers_context = zmq.Context(4)
        for worker in self._config['WORKERS']:

            subscription_str = str("tcp://" + str(self._config['WORKERS'][worker]['IP']) + ":" +
                                    self._config['WORKERS'][worker]['COMMAND_PUBLISHER_PORT'])

            self.worker_dict[worker] = self.workers_context.socket(zmq.SUB)
            self.worker_dict[worker].connect(subscription_str)
            self.worker_dict[worker].setsockopt(zmq.SUBSCRIBE, str(self._config['AGENT_ID']).encode())
            self.workers_poller = zmq.Poller()
            self.workers_poller.register(self.worker_dict[worker], zmq.POLLIN)

            self.vprint("The worker subscription address = " + subscription_str)


    # These are threaded processes that are initialized by the Agent Hub
    def initialize_agent_core_classes(self, config):

        self.vprint("I should be starting the Mavlink Manager")
        self.mav_mgr = MavlinkManager(config, self, verbose=self._verbose)
        self.mav_mgr.run()
        # self.vprint("I should be starting the agent_heartbeat_mgr")
        # self.agent_heartbeat_mgr = AgentHeartbeatManager(config, self, verbose=self._verbose)
        # self.agent_heartbeat_mgr.run()


    def check_for_command_messages(self, poll_delay_in_msec):

        socks = dict(self.workers_poller.poll(poll_delay_in_msec))
        for worker in self.worker_dict.values():

            if worker in socks:
                self.vprint("I should be sending a command to the flight controller")
                try:
                    command_message = worker.recv_multipart()
                    return command_message
                except:
                    return None
            else:
                return None
