#!/usr/bin/env python3

''' This script creates a heartbeat process for an agent to
    talk to the ui_worker'''

import time
import json
from datetime import datetime
from threading import Thread
from multiprocessing import Process

from .agentutils import AgentUtils
from .agent_status_class import AgentStatus
# from agent_hub import AgentHub

import zmq

class AgentHeartbeatManager:

    ''' This class is used to manage the agent's heartbeats with the UIs via the worker '''

    # def __init__(self, config, agent_hub: AgentHub, verbose=False):
    def __init__(self, config, agent_hub, verbose=False):

        self.config = config
        self._verbose = verbose
        # self._agent_status_obj: AgentStatus = agent_hub.agent_status_obj
        self.agent_hub = agent_hub

        self._agent_heartbeat_loop_thread = Thread(target=self.agent_heartbeat_loop_thread,
                    args=[self.agent_hub])
        self._agent_heartbeat_loop_thread.daemon = True

        self.send_heartbeat_connection_address = str("tcp://" + config['WORKERS']['UI_WORKER']['IP'] +
                                                ":" + config['WORKERS']['UI_WORKER']['HEARTBEAT_REPLY_PORT'])
        self._base_heartbeat = config['BASE_HEARTBEAT']
        self._base_heartbeat_wait_time_sec = config['BASE_HEARTBEAT_WAIT_TIME_SEC']
        self._heartbeat_count_fail_limit = config['HEARTBEAT_COUNT_FAIL_LIMIT']
        
        self.mavlinkmgr_lost_mavlink_connection = True

    def vprint(self, print_string):
        if self._verbose:
            print(print_string)

    def run(self):
        ''' Starts the thread that starts sending a repeating heartbeat signal to the ui_worker'''
        self._agent_heartbeat_loop_thread.start()


    def stop(self):
        ''' Stops the thread that is sending a repeating heartbeat signal to the ui_worker'''
        self._agent_heartbeat_loop_thread.join()

    # def agent_heartbeat_loop_thread(self, agent_hub: AgentHub):
    def agent_heartbeat_loop_thread(self, agent_hub):

        ''' This heatbeat function tells the worker that this agent is
            still alive.  It sends a heatbeat code with an agent
            status dict as well as a timestamp '''
        
        heartbeat_count = 0 # How many heartbeats have occured since the last accepted heartbeat
        last_heartbeat_time = time.time() # Time of last heartbeat
        send_heartbeats = True  # Should be sending heartbeats
        heartbeat_wait_time = 0

        context = zmq.Context.instance()

        # send_heartbeat_with_status = context.socket(zmq.REQ)
        # send_heartbeat_with_status.connect(self.send_heartbeat_connection_address)

        while send_heartbeats:

            if not agent_hub.immediate_response_queue.empty():
                immediate_response = agent_hub.immediate_response_queue.get()
                self.vprint("The immediate response is: " + str(immediate_response))
            else:
                immediate_response = False

            delta_time = int(time.time() - last_heartbeat_time)
            # self.vprint("heartbeat_wait_time = " + str(heartbeat_wait_time) + " / delta_time = " + str(delta_time))
            # if seconds since last heartbeat is greater than the current
            # heartbeat count then send another heartbeat
            if (delta_time >= heartbeat_wait_time) or (immediate_response is True):

                heartbeat_wait_time = self._base_heartbeat_wait_time_sec
                # # reset last_heartbeat_time to now
                last_heartbeat_time = time.time()
                # # send a heartbeat to known hub heartbeat monitor address

                immediate_response = False
                heartbeat_wait_time, heartbeat_count, send_heartbeats, agent_hub = self.send_agent_heartbeat(
                    context,
                    agent_hub,
                    heartbeat_count,
                    heartbeat_wait_time)
                
    def send_agent_heartbeat(self,
                             context,
                             agent_hub,
                             heartbeat_count,
                             heartbeat_wait_time):
        
        ''' This function sends the heartbeat with the agent status to the ui_worker '''

        send_heartbeats = True

        # Add the current timestamp to the current status of the agent.
        # This is the format of the heartbeat message
        current_agent_status_obj = agent_hub.agent_status_obj
        current_agent_status_obj.time_stamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S.%f")
        current_agent_status_obj_json = agent_hub.agent_status_obj.get_json()

        # Send heartbeat in form of json message to the ui_worker
        # agent_hub.status_heartbeat_to_worker_message.send_pyobj(agent_hub.agent_status_obj)
        agent_hub.status_heartbeat_to_worker_message.send_json(current_agent_status_obj_json)
        self.vprint("sent heartbeat: " + str(current_agent_status_obj_json))

        # if you got a response back in the Poller from your heartbeat request
        socks = dict(agent_hub.network_heartbeat_poller.poll(
            self._base_heartbeat_wait_time_sec*1000))
        if agent_hub.status_heartbeat_to_worker_message in socks:
            # if it's an acknowledgement of the REQ heartbeat ...
            msg = agent_hub.status_heartbeat_to_worker_message.recv(zmq.NOBLOCK).decode()
            if msg == "ack":
            # if send_heartbeat_with_status.recv(zmq.NOBLOCK).decode() == "ack":
                self.vprint("Heartbeat Mgr received ack from ui_worker")
                # Having confirmed a good connection with the ui_worker
                # reset the heartbeat_count to a resting heartbeat rate
                heartbeat_count = self._base_heartbeat
                heartbeat_wait_time = self._base_heartbeat_wait_time_sec
                # retain the sockets and go back and wait for the next
                # delta_time to equal the desired heartbeat interval
                # continue
            # else you got some other response back from the heartbeat
            # --- this could be where acknowledged commands are received
            else:
                self.vprint("Error")

        # else you did not get a response from the ui_worker
        else:
            # if you've reached the max number of heartbeats for failure
            # tell the agent main thread that you are no longer connected
            self.vprint("Heartbeat Count: " + str(heartbeat_count))
            if heartbeat_count >= self._heartbeat_count_fail_limit:
                # pass
                try:
                    self.vprint("Lost Comm")
                    agent_hub.network_heartbeat_connection.send("lost_comm".encode())
                    # This will kill the loop and hence the thread
                    # Do something else if you want to go down a
                    # different path for connection failer
                    # heartbeat_wait_time = BASE_HEARTBEAT_WAIT_TIME_SEC
                    # send_heartbeat_with_status.close()
                    send_heartbeats = False
                except zmq.ZMQError as heartbeat_error:
                    self.vprint(heartbeat_error)
            # else you did not get a resonse but are not at failure
            # mode yet.  Close and reopen the socket connection for
            # the next heartbeat cycle
            else:
                agent_hub.status_heartbeat_to_worker_message.close()
                agent_hub.status_heartbeat_to_worker_message = agent_hub.network_heartbeat_context.socket(zmq.REQ)
                agent_hub.status_heartbeat_to_worker_message.connect(
                    agent_hub.status_heartbeat_to_worker_connection_address)
                agent_hub.network_heartbeat_poller.register(agent_hub.status_heartbeat_to_worker_message, zmq.POLLIN)

                # bump the heartbeat count by one
                heartbeat_count += 1

        return (heartbeat_wait_time, heartbeat_count, send_heartbeats, agent_hub)
