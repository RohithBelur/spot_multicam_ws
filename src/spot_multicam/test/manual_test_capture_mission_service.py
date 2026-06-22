"""
manual_test_capture_mission_service.py

Modeled on boston-dynamics/spot-sdk's remote_mission_client.py, but targets
our CaptureMissionServicer instead of the hello-world example, and drives a
realistic start_capture -> wait -> stop_capture sequence.

This is the no-robot local test pattern this project settled on: there is no
official Spot physics simulator, so this is what "testing the mission bridge
without the real robot" actually looks like for a RemoteMissionService -
talking to the servicer directly over an insecure local gRPC channel.

Usage (server must already be running separately):
    # terminal 1:
    python3 -m spot_multicam.mission.capture_mission_service local --port 23456

    # terminal 2:
    python3 manual_test_capture_mission_service.py --host-ip 127.0.0.1 --port 23456
"""

import argparse
import time

import grpc

import bosdyn.mission.remote_client
from bosdyn.api import service_customization_pb2
from bosdyn.api.mission import remote_pb2

_COMMAND_KEY = 'command'
_INTERVAL_KEY = 'interval_s'


def _build_params(command: str, interval_s: float) -> service_customization_pb2.DictParam:
    params = service_customization_pb2.DictParam()
    params.values.get_or_create(_COMMAND_KEY).string_value.value = command
    params.values.get_or_create(_INTERVAL_KEY).double_value.value = interval_s
    return params


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host-ip', required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--interval-s', type=float, default=0.1)
    parser.add_argument('--capture-duration-s', type=float, default=2.0,
                        help='How long to "capture" before sending stop_capture.')
    options = parser.parse_args()

    client = bosdyn.mission.remote_client.RemoteClient()
    client.channel = grpc.insecure_channel(f'{options.host_ip}:{options.port}')

    print('--- EstablishSession ---')
    session_id = client.establish_session(lease_resources=())
    print(f'session_id = {session_id}')

    print('--- Tick: start_capture ---')
    params = _build_params('start_capture', options.interval_s)
    response = client.tick(session_id, lease_resources=(), params=params)
    print(f'status = {remote_pb2.TickResponse.Status.Name(response.status)}')
    assert response.status == remote_pb2.TickResponse.STATUS_SUCCESS, response.error_message

    print(f'--- "capturing" for {options.capture_duration_s}s ---')
    time.sleep(options.capture_duration_s)

    print('--- Tick: stop_capture ---')
    params = _build_params('stop_capture', options.interval_s)
    response = client.tick(session_id, lease_resources=(), params=params)
    print(f'status = {remote_pb2.TickResponse.Status.Name(response.status)}')
    assert response.status == remote_pb2.TickResponse.STATUS_SUCCESS, response.error_message

    print('--- Stop ---')
    client.stop(session_id)

    print('--- TeardownSession ---')
    client.teardown_session(session_id)

    print('\nAll RPCs succeeded.')


if __name__ == '__main__':
    main()
