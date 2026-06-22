# Modeled directly on boston-dynamics/spot-sdk's
# python/examples/remote_mission_service/{hello_world,robot_command}_mission_service.py
# (verified against the real bosdyn-mission 5.1.4 API in this project's own dev
# environment - see project README for the no-robot test commands used).
#
# This service does not need a lease - it never commands the robot body, it only
# starts/stops capture on the camera nodes via a CaptureController. That mirrors
# the original system's RemoteMissionService, which only ever talked to the
# Raspberry Pi nodes, never to Spot's body.

import logging
import random
import string
import threading

import bosdyn.client
import bosdyn.client.util
from bosdyn.api.mission import remote_pb2, remote_service_pb2_grpc
from bosdyn.client.directory_registration import (DirectoryRegistrationClient,
                                                   DirectoryRegistrationKeepAlive)
from bosdyn.client.server_util import GrpcServiceRunner, ResponseContext
from bosdyn.client.service_customization_helpers import (create_value_validator,
                                                          dict_param_coerce_to,
                                                          dict_params_to_dict,
                                                          make_dict_child_spec,
                                                          make_dict_param_spec,
                                                          make_double_param_spec,
                                                          make_string_param_spec,
                                                          make_user_interface_info,
                                                          validate_dict_spec)
from bosdyn.client.util import setup_logging

from spot_multicam.mission.capture_controller import CaptureController, LoggingCaptureController

DIRECTORY_NAME = 'spot-multicam-capture'
AUTHORITY = 'remote-mission'
SERVICE_TYPE = 'bosdyn.api.mission.RemoteMissionService'

_LOGGER = logging.getLogger(__name__)

# Mirrors the original system's documented mission interface exactly:
# "Mission commands: start_capture, stop_capture, noop"
# "Mission parameter: interval_s, clamped between 0.05 and 5.0 seconds"
_COMMAND_KEY = 'command'
_COMMAND_OPTIONS = ['start_capture', 'stop_capture', 'noop']
_COMMAND_DEFAULT = 'noop'

_INTERVAL_KEY = 'interval_s'
_INTERVAL_MIN = 0.05
_INTERVAL_MAX = 5.0
_INTERVAL_DEFAULT = 0.1


class CaptureMissionServicer(remote_service_pb2_grpc.RemoteMissionServiceServicer):
    """RemoteMissionService for triggering multi-camera capture from a Spot
    mission, without commanding the robot body (no lease required).

    Shows the same concepts as the BD examples this is modeled on:
     - Ticking
     - Using parameter dictionaries (command, interval_s)
     - Maintaining state for multiple sessions
    but delegates the actual start/stop work to a CaptureController, so this
    class has no idea whether it's talking to LoggingCaptureController (today)
    or a real ROS2 action bridge (once Phase 4 exists).
    """

    def __init__(self, capture_controller: CaptureController, logger=None, coerce=True):
        self.lock = threading.Lock()
        self.logger = logger or _LOGGER
        self.capture_controller = capture_controller
        self.coerce = coerce
        self.sessions_by_id = {}
        self._used_session_ids = []

        command_param = make_string_param_spec(options=_COMMAND_OPTIONS,
                                               default_value=_COMMAND_DEFAULT, editable=False)
        command_ui_info = make_user_interface_info(
            'Capture Command', 'start_capture, stop_capture, or noop.')

        interval_param = make_double_param_spec(min_value=_INTERVAL_MIN, max_value=_INTERVAL_MAX,
                                                default_value=_INTERVAL_DEFAULT)
        interval_ui_info = make_user_interface_info(
            'Capture Interval (s)', 'Per-camera capture interval, clamped to [0.05, 5.0]s.')

        dict_spec = make_dict_param_spec(
            {
                _COMMAND_KEY: make_dict_child_spec(command_param, command_ui_info),
                _INTERVAL_KEY: make_dict_child_spec(interval_param, interval_ui_info),
            }, is_hidden_by_default=False)
        validate_dict_spec(dict_spec)
        self.custom_params = dict_spec

    # --- Tick -----------------------------------------------------------------

    def Tick(self, request, context):
        response = remote_pb2.TickResponse()
        self.logger.debug('Ticked with session ID "%s"', request.session_id)
        with ResponseContext(response, request):
            with self.lock:
                self._tick_implementation(request, response)
        return response

    def _tick_implementation(self, request, response):
        if request.session_id not in self.sessions_by_id:
            self.logger.error('Do not know about session ID "%s"', request.session_id)
            response.status = remote_pb2.TickResponse.STATUS_INVALID_SESSION_ID
            return

        valid_param = create_value_validator(self.custom_params)(request.params)
        if valid_param is not None:
            if self.coerce:
                dict_param_coerce_to(request.params, self.custom_params)
            else:
                self.logger.error('Invalid parameter for capture mission node.')
                response.status = remote_pb2.TickResponse.STATUS_CUSTOM_PARAMS_ERROR
                response.custom_param_error.CopyFrom(valid_param)
                return

        params = dict_params_to_dict(request.params, self.custom_params, validate=True)
        command = params.get(_COMMAND_KEY, _COMMAND_DEFAULT)
        interval_s = params.get(_INTERVAL_KEY, _INTERVAL_DEFAULT)
        # Defensive clamp even though the param spec already enforces this -
        # matches the original system's documented behaviour explicitly.
        interval_s = max(_INTERVAL_MIN, min(_INTERVAL_MAX, interval_s))

        if command == 'start_capture':
            self._handle_start_capture(interval_s, response)
        elif command == 'stop_capture':
            self._handle_stop_capture(response)
        elif command == 'noop':
            self.logger.info('noop tick')
            response.status = remote_pb2.TickResponse.STATUS_SUCCESS
        else:
            self.logger.error('Unknown command "%s"', command)
            response.status = remote_pb2.TickResponse.STATUS_FAILURE
            response.error_message = f'Unknown command "{command}"'

        self.sessions_by_id[request.session_id]['command'] = command

    def _handle_start_capture(self, interval_s, response):
        try:
            self.capture_controller.start(interval_s)
        except Exception as exc:  # noqa: BLE001 - surface any backend failure as a Tick failure
            self.logger.error('start_capture failed: %s', exc)
            response.status = remote_pb2.TickResponse.STATUS_FAILURE
            response.error_message = str(exc)
            return
        # Matches the original system's documented behaviour: "start_capture
        # starts every configured node in parallel and returns quickly" -
        # this is a fire-and-forget trigger, not a long-running Tick.
        self.logger.info('start_capture triggered (interval_s=%.3f)', interval_s)
        response.status = remote_pb2.TickResponse.STATUS_SUCCESS

    def _handle_stop_capture(self, response):
        try:
            stats = self.capture_controller.stop()
        except Exception as exc:  # noqa: BLE001
            self.logger.error('stop_capture failed: %s', exc)
            response.status = remote_pb2.TickResponse.STATUS_FAILURE
            response.error_message = str(exc)
            return
        self.logger.info('stop_capture complete: %s', stats)
        response.status = remote_pb2.TickResponse.STATUS_SUCCESS

    # --- Session lifecycle -------------------------------------------------------

    def EstablishSession(self, request, context):
        response = remote_pb2.EstablishSessionResponse()
        with ResponseContext(response, request):
            with self.lock:
                session_id = self._get_unique_random_session_id()
                self.sessions_by_id[session_id] = {'command': None}
                self._used_session_ids.append(session_id)
                response.session_id = session_id
                response.status = remote_pb2.EstablishSessionResponse.STATUS_OK
        return response

    def _get_unique_random_session_id(self):
        while True:
            session_id = ''.join(random.choice(string.ascii_letters) for _ in range(16))
            if session_id not in self._used_session_ids:
                return session_id

    def Stop(self, request, context):
        """Safety net: if the mission stops ticking us mid-capture (the next
        node activates, the mission is paused, etc.), make sure capture is
        not left running. Mirrors how the original system relied on SIGINT
        on shutdown to avoid orphaned capture processes.
        """
        response = remote_pb2.StopResponse()
        with ResponseContext(response, request):
            with self.lock:
                session = self.sessions_by_id.get(request.session_id)
                if session is None:
                    response.status = remote_pb2.StopResponse.STATUS_INVALID_SESSION_ID
                    return response
                if session.get('command') == 'start_capture':
                    self.logger.info(
                        'Stop() called while capture was active - stopping as a safety net.')
                    try:
                        self.capture_controller.stop()
                    except Exception as exc:  # noqa: BLE001
                        self.logger.error('Safety-net stop_capture failed: %s', exc)
                response.status = remote_pb2.StopResponse.STATUS_OK
        return response

    def TeardownSession(self, request, context):
        response = remote_pb2.TeardownSessionResponse()
        with ResponseContext(response, request):
            with self.lock:
                if request.session_id in self.sessions_by_id:
                    del self.sessions_by_id[request.session_id]
                    response.status = remote_pb2.TeardownSessionResponse.STATUS_OK
                else:
                    response.status = remote_pb2.TeardownSessionResponse.STATUS_INVALID_SESSION_ID
        return response

    def GetRemoteMissionServiceInfo(self, request, context):
        response = remote_pb2.GetRemoteMissionServiceInfoResponse()
        with ResponseContext(response, request):
            response.custom_params.CopyFrom(self.custom_params)
        return response


def run_service(port, capture_controller: CaptureController, logger=None):
    add_servicer_to_server_fn = remote_service_pb2_grpc.add_RemoteMissionServiceServicer_to_server
    service_servicer = CaptureMissionServicer(capture_controller, logger=logger)
    return GrpcServiceRunner(service_servicer, add_servicer_to_server_fn, port, logger=logger)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='Select how this service will be accessed.',
                                       dest='host_type')

    # 'local': Boston Dynamics' official no-robot test mode - exactly what's
    # used during development of this project, since there is no Spot
    # physics simulator and no physical robot is being used here.
    local_parser = subparsers.add_parser(
        'local', help='Run locally, no robot involved (BD\'s official no-robot test mode).')
    bosdyn.client.util.add_service_hosting_arguments(local_parser)

    # 'robot': for later, once this is actually deployed against real Spot hardware.
    robot_parser = subparsers.add_parser('robot', help='Run with a real robot in the loop.')
    bosdyn.client.util.add_base_arguments(robot_parser)
    bosdyn.client.util.add_service_endpoint_arguments(robot_parser)

    options = parser.parse_args()

    capture_controller = LoggingCaptureController(logger=_LOGGER)

    if options.host_type == 'local':
        setup_logging()
        service_runner = run_service(options.port, capture_controller, logger=_LOGGER)
        print(f'{DIRECTORY_NAME} service running.\nCtrl + C to shutdown.')
        service_runner.run_until_interrupt()
        return

    setup_logging(options.verbose)
    sdk = bosdyn.client.create_standard_sdk('SpotMulticamCaptureMissionServiceSDK')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    service_runner = run_service(options.port, capture_controller, logger=_LOGGER)
    dir_reg_client = robot.ensure_client(DirectoryRegistrationClient.default_service_name)
    keep_alive = DirectoryRegistrationKeepAlive(dir_reg_client, logger=_LOGGER)
    keep_alive.start(DIRECTORY_NAME, SERVICE_TYPE, AUTHORITY, options.host_ip, service_runner.port)

    with keep_alive:
        service_runner.run_until_interrupt()


if __name__ == '__main__':
    main()
