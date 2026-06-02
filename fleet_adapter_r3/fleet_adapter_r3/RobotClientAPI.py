# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''
import json
import websocket
import requests
import logging
from urllib.error import HTTPError
import numpy as np
import math
from typing import Dict
import uuid

from .utils.Coordinate import LionsbotCoord
from .utils import constants

from .enums.enums import ActiveMissionType
from .enums.enums import NavigationStatus
from .enums.enums import RobotStatus
from .enums.enums import ResponseCode
from .enums.enums import RobotMissionStatus
from .enums.enums import OperationEndStatus

from .models.NavigateContent import NavigateContent
from .models.CleanProcessContent import CleanProcessContent
from .models.DockProcessContent import DockProcessContent
import time
from datetime import datetime
from datetime import timezone

from .models.StopProcessContent import StopProcessContent
from .models.Zone import Zone
import threading

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


def _mask_token(token: str | None) -> str:
    if not token:
        return '<none>'
    if len(token) <= 16:
        return '<short_token>'
    return f'{token[:8]}...{token[-8:]}'


def _redact_mapping(data: dict, secret_keys: tuple = ('password', 'token')) -> dict:
    return {k: ('***' if k in secret_keys else v) for k, v in data.items()}


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.connected = False
        self.token = None
        self.token_expiry = None
        self.robot_status_ws_connection = None
        self.robot_pose_ws_connection = None
        self.robot = None
        self.connected = None

        self.robot_current_map = {}
        self.robot_current_building = {}
        self.robot_status = {}
        self.robot_pose: Dict[str, LionsbotCoord] = {}
        self.robot_mission = {}
        self.robot_operation = {}
        self.robot_operation_end_status = {}

        # Tracking of robot state changes
        self.robot_current_state_id = {}
        self.robot_navigate_state_id = {}

        self.xy_goal_tolerance = 5

        self._lock = threading.Lock()
        self._token_lock = threading.Lock()
        self._subscribed_robots: set[str] = set()

        # Test connectivity
        connected = self.check_connection()
        self.connected = connected
        if connected:
            self.connected = True
        else:
            self.connected = False

    # ------------------------------------------------------------------------------
    # Static Variables
    # ------------------------------------------------------------------------------
    TERMINAL_MISSION_STATUSES = {RobotMissionStatus.CLEANING_FINISHED.value,
                                 RobotMissionStatus.MOVING_FINISHED.value,
                                 RobotMissionStatus.APP_STOPPED.value,
                                 RobotMissionStatus.MOVING_STOPPED.value,
                                 RobotMissionStatus.MOVING_FINISHED.value,
                                 RobotMissionStatus.E_STOP_PRESSED.value,
                                 RobotMissionStatus.IN_CRITICAL.value,
                                 RobotMissionStatus.DOCKED.value,
                                 RobotMissionStatus.APP_MOVING_TO_DOCK_STOPPED.value,
                                 RobotMissionStatus.APP_DOCKING_STOPPED.value,
                                 RobotMissionStatus.DOCKING_STOPPED.value,
                                 RobotMissionStatus.APP_MOVING_TO_WORK_STOPPED.value,
                                 RobotMissionStatus.MOVING_TO_WORK_STOPPED.value,
                                 RobotMissionStatus.APP_CLEANING_STOPPED.value,
                                 RobotMissionStatus.CLEANING_STOPPED.value}
    
    ROBOT_ACTIVE_STATUSES = {RobotStatus.CLEANING.value, 
                            RobotStatus.MOVING.value, 
                            RobotStatus.RESTING.value}
    # ------------------------------------------------------------------------------
    # Websocket Functions
    # ------------------------------------------------------------------------------
    def check_connection(self):
        self.request_token()
        if self.token is None:
            return False

        connect_to_robot_status_thread = threading.Thread(target=self.connect_to_robot_status_ws, daemon=True)
        connect_to_robot_pose_thread = threading.Thread(target=self.connect_to_robot_position_ws, daemon=True)

        connect_to_robot_status_thread.start()
        connect_to_robot_pose_thread.start()
        time.sleep(3)
        return True
    
    def request_token(self):
        '''Login without Authorization; store token from JSON field "token".'''
        path = constants.SECURITY_PATH
        payload = {'email': self.user, 'password': self.password, 'applicationName': 'DASHBOARD'}

        with self._token_lock:
            try:
                logger.debug('HTTP POST login request body=%s', _redact_mapping(payload))
                r = self._post(
                    path=f'{constants.OPEN_API_PREFIX}{path}',
                    headers=None,
                    json=payload,
                )
                r.raise_for_status()
                data = r.json()
                logger.debug('HTTP POST login response body=%s', _redact_mapping(data))
                token = data['token']
                expiry_raw = data['tokenExpiryIsoUtcTime']
                for fmt in ("%Y-%m-%dT%H:%M:%S.%fZ", "%Y-%m-%dT%H:%M:%SZ"):
                    try:
                        token_expiry = datetime.strptime(expiry_raw, fmt).replace(tzinfo=timezone.utc)
                        break
                    except ValueError:
                        continue
                else:
                    raise ValueError(f'Unrecognized tokenExpiryIsoUtcTime format: {expiry_raw!r}')

                self.token = token
                self.token_expiry = token_expiry.timestamp()
                logger.info(
                    'Login OK: token=%s expires_utc=%s (in %.0fs)',
                    _mask_token(token),
                    expiry_raw,
                    self.token_expiry - time.time(),
                )
            except requests.exceptions.ConnectionError as connection_error:
                logger.error('Login connection error: %s', connection_error)
            except HTTPError as http_err:
                logger.error('Login HTTP error: %s', http_err)
            except (KeyError, ValueError) as parse_err:
                logger.error('Login response parse error: %s', parse_err)

        return self.token

    def _bearer_headers(self) -> dict[str, str]:
        '''HTTP REST only: Authorization: Bearer <token> (see Postman REST requests).'''
        return {'Authorization': f'Bearer {self.token}'}

    def _ws_subprotocols(self) -> list[str]:
        '''Token as WebSocket subprotocol → Sec-WebSocket-Protocol header on handshake.'''
        return [self.token]

    def _log_ws_auth(self, channel: str, ws_url: str, subprotocols: list[str]):
        logger.debug(
            'WS CONNECT [%s] url=%s origin=https://%s offered_subprotocols=%s',
            channel,
            ws_url,
            self.prefix,
            [_mask_token(p) for p in subprotocols],
        )
        if self.token is None:
            logger.warning('WS CONNECT [%s]: token is None — handshake will fail', channel)

    def _log_ws_subprotocol_selected(self, channel: str, wsc: websocket.WebSocketApp):
        '''Log server-selected subprotocol after handshake (client only offers; server picks).'''
        offered = [_mask_token(p) for p in (wsc.subprotocols or [])]
        selected = None
        if wsc.sock is not None:
            selected = wsc.sock.subprotocol
        logger.debug(
            'WS CONNECT [%s] offered_subprotocols=%s selected_subprotocol=%s',
            channel,
            offered,
            _mask_token(selected),
        )
        print(
            f'[{channel}] WebSocket subprotocol selected by server: '
            f'{_mask_token(selected)} (offered: {offered})'
        )

    def _log_ws_send(self, channel: str, body: str):
        logger.info('WS SEND [%s] body=%s', channel, body)

    def refresh_expired_token(self):
        if self.token_expiry is None or self.token_expiry <= time.time():
            self.request_token()

    def _build_https_url(self, path: str) -> str:
        return f'https://{self.prefix}{path}'

    def _build_wss_url(self, path: str) -> str:
        return f'wss://{self.prefix}{path}'

    def _get(self, path: str, headers=None):
        url = self._build_https_url(path)
        logger.debug('HTTP GET %s', url)
        logger.debug('HTTP GET headers=%s', headers)
        return requests.get(url, headers=headers)

    def _post(self, path: str, headers=None, json=None):
        url = self._build_https_url(path)
        logger.debug('HTTP POST %s', url)
        logger.debug('HTTP POST headers=%s', headers)
        logger.debug('HTTP POST json=%s', json)
        return requests.post(url, headers=headers, json=json)

    def _put(self, path: str, headers=None, json=None):
        url = self._build_https_url(path)
        logger.debug('HTTP PUT %s', url)
        logger.debug('HTTP PUT headers=%s', headers)
        logger.debug('HTTP PUT json=%s', json)
        return requests.put(url, headers=headers, json=json)

    def connect_to_robot_status_ws(self):
        self.refresh_expired_token()
        if self.token is None:
            logger.error('Cannot open robotstatus WebSocket: no token after login')
            return

        def on_message(wsc, message):
            logger.debug('WS RECV status message=%s', message)
            json_message = json.loads(message)

            with self._lock:
                operation_fb = json_message.get('operation_fb')
                if operation_fb == 'ping':
                    return
                robot_id = json_message.get('robot_id')
                if robot_id is None:
                    return

                if operation_fb == 'touchscreen_robot_status':
                    robot_status = json_message['content']

                    # Update the robot current state id if robot has changed state
                    prev_status = self.robot_status.get(robot_id, {})
                    if prev_status.get('state') != robot_status['state']:
                        self.robot_current_state_id[robot_id] = uuid.uuid4()

                    response_codes = robot_status['response_codes']
                    alert_ids = [code['code'] for code in response_codes]
                    self.robot_status[robot_id] = {
                        'eta': robot_status['time_to_complete'],
                        'alertIds': alert_ids,
                        'progress': robot_status['mission_progress'],
                        'localized': robot_status['localized'],
                        'batterySoc': robot_status['battery_soc'],
                        'state': robot_status['state'],
                        'status': robot_status['status']
                    }
                elif operation_fb == 'operation_status':
                    operation_status = json_message['content']
                    status_snapshot = self.robot_status.get(robot_id, {})
                    self.robot_mission[robot_id] = {
                        'missionStatus': {
                            'activeMissionType': operation_status['activeMissionType'],
                            'mission': {
                                'status': operation_status['mission']['status'],
                                'x': operation_status['mission'].get('x', None), 
                                'y': operation_status['mission'].get('y', None), 
                            }
                        },
                        'eta': status_snapshot.get('eta'),
                        'alertIds': status_snapshot.get('alertIds'),
                        'progress': status_snapshot.get('progress')
                    }
                elif operation_fb == OperationEndStatus.P2P_END_STATUS:
                    self.robot_operation_end_status[robot_id] = json_message
                else:
                    robot_operation = self.robot_operation.get(robot_id, None)
                    if robot_operation is not None and \
                            robot_operation['operation'] == operation_fb and \
                            robot_operation['time_stamp'] < time.time_ns() / 1000000:
                        operation_status = json_message['content']
                        self.robot_operation[robot_id]['status'] = operation_status['status']

        def on_error(wsc, error):
            print(error)

        def on_close(wsc, close_status_code, close_msg):
            print("### Status Websocket Connecton Closed ###")

        def on_open(wsc):
            print("Opened Status Websocket Connection")

        path = f'{constants.WS_OPEN_API_PREFIX}/robotstatus'
        subprotocols = self._ws_subprotocols()
        status_ws_url = self._build_wss_url(path)
        self._log_ws_auth('robotstatus', status_ws_url, subprotocols)
        self.robot_status_ws_connection = websocket.WebSocketApp(
            status_ws_url,
            subprotocols=subprotocols,
            on_open=on_open,
            on_close=on_close,
            on_error=on_error,
            on_message=on_message,
        )
        self.robot_status_ws_connection.run_forever(suppress_origin=True)

    def connect_to_robot_position_ws(self):
        self.refresh_expired_token()
        if self.token is None:
            logger.error('Cannot open robotpose WebSocket: no token after login')
            return

        def on_message(wsc, message):
            log_msg = message
            if len(log_msg) > 16:
                log_msg = log_msg[:8] + '...' + log_msg[-8:]
            logger.debug('WS RECV pose message=%s', log_msg)
            json_message = json.loads(message)
            with self._lock:
                if json_message.get('operation_fb') == 'ping':
                    return
                robot_id = json_message.get('robot_id')
                if json_message.get('operation_fb') == 'robot_pose' and robot_id is not None:
                    robot_pose = json_message['content']
                    self.robot_pose[robot_id] = LionsbotCoord(x=robot_pose['x'],
                                                                y=robot_pose['y'],
                                                                orientation_radians=math.radians(robot_pose['heading']))

        def on_error(wsc, error):
            logger.error(error)

        def on_close(wsc, close_status_code, close_msg):
            print("### Position Websocket Connecton Closed ###")

        def on_open(wsc):
            print("Opened Position Websocket Connection")

        path = f'{constants.WS_OPEN_API_PREFIX}/robotpose'
        subprotocols = self._ws_subprotocols()
        pose_ws_url = self._build_wss_url(path)
        self._log_ws_auth('robotpose', pose_ws_url, subprotocols)
        self.robot_pose_ws_connection = websocket.WebSocketApp(
            pose_ws_url,
            subprotocols=subprotocols,
            on_open=on_open,
            on_close=on_close,
            on_error=on_error,
            on_message=on_message,
        )
        # TODO: check if origin should be set
        self.robot_pose_ws_connection.run_forever(suppress_origin=True)

    def is_subscribed(self, robot_encoding_id: str) -> bool:
        return robot_encoding_id in self._subscribed_robots

    def subscribe_to_robot(self, robot_encoding_id: str, time_stamp: float):
        if robot_encoding_id in self._subscribed_robots:
            return True

        payload = {'operation_cmd': 'subscribe', 'robot_id': robot_encoding_id}
        subscribe_status_message = json.dumps(payload)
        subscribe_pose_message = json.dumps(payload)

        if self.robot_status_ws_connection is None or self.robot_pose_ws_connection is None:
            logger.error('WS SEND subscribe skipped: WebSocket connection not established')
            return False

        self._log_ws_send('robotstatus', subscribe_status_message)
        self._log_ws_send('robotpose', subscribe_pose_message)

        self.robot_status_ws_connection.send(subscribe_status_message)
        self.robot_pose_ws_connection.send(subscribe_pose_message)
        self._subscribed_robots.add(robot_encoding_id)

        time.sleep(2.5)

        return True

    def _build_ws_command_payload(self, operation_cmd: str, robot_name: str, time_stamp: float, content: dict):
        return {
            'operation_cmd': operation_cmd,
            'robot_id': robot_name,
            'time_stamp': time_stamp,
            'content': content
        }

    # ------------------------------------------------------------------------------
    # Robot Information Accessors
    # ------------------------------------------------------------------------------
    def position(self, robot_name: str) -> LionsbotCoord:
        ''' Return Coordinate:LionsbotCoord expressed in the robot's coordinate frame or
            None if any errors are encountered'''
        position = self.robot_pose.get(robot_name, None)

        return position

    def get_mission_status(self, robot_name: str):
        mission_status = self.robot_mission.get(robot_name, None)

        if mission_status is None:
            self.refresh_expired_token()

            path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/mission-status'
            headers = self._bearer_headers()

            try:
                r = self._get(path=path, headers=headers)
                r.raise_for_status()
                data = r.json()

                self.robot_mission[robot_name] = data

                return data
            except requests.exceptions.ConnectionError as connection_error:
                print(f'Connection error: {connection_error}')
            except HTTPError as http_err:
                print(f'HTTP error: {http_err}')

            return None

        return mission_status

    def get_robot_info(self, robot_name: str):
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}'
        headers = self._bearer_headers()

        try:
            r = self._get(path=path, headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_robot_status(self, robot_name: str):
        return self.robot_status.get(robot_name, None)

    def navigation_remaining_duration(self, robot_name: str):
        ''' Return the number of seconds remaining for the robot to reach its
            destination'''
        self.refresh_expired_token()

        mission_status = self.get_mission_status(robot_name=robot_name)
        while mission_status is None:
            time.sleep(1)
            mission_status = self.get_mission_status(robot_name=robot_name)

        if mission_status is None or mission_status['eta'] is None:
            return 0.0

        return mission_status['eta']

    def battery_soc(self, robot_name: str):
        ''' Return the state of charge of the robot as a value between 0.0
            and 1.0. Else return None if any errors are encountered'''
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name)
        while robot_status is None:
            robot_status = self.get_robot_status(robot_name)

        if robot_status is None:
            return None

        return robot_status['batterySoc'] / 100

    def get_robot_maps(self, robot_name: str):
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/map'
        headers = self._bearer_headers()

        try:
            r = self._get(path=path, headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_map(self, map_name: str, robot_name: str):
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/map'
        headers = self._bearer_headers()

        try:
            r = self._get(path=path, headers=headers)
            r.raise_for_status()
            data = r.json()

            robot_current_building = self.robot_current_building[robot_name]
            maps = data['maps']
            for m in maps:
                if m['name'] == f'{map_name}_{robot_current_building}' and m['level'] == f'{map_name}':
                    return m
                if m['name'] == map_name or m['level'] == map_name:
                    return m
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_zones_by_map(self, map_id: str):
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/robot/map/{map_id}/zones'
        headers = self._bearer_headers()

        try:
            r = self._get(path=path, headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    def get_zone_equalizers(self, map_id: str, robot_name: str):
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/robot/map/{map_id}/equalizer-configs?robotId={robot_name}'
        headers = self._bearer_headers()

        try:
            r = self._get(path=path, headers=headers)
            r.raise_for_status()
            data = r.json()

            return data
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return None

    # ------------------------------------------------------------------------------
    # Robot Operations
    # ------------------------------------------------------------------------------
    def navigate(self, robot_name: str, pose: LionsbotCoord, map_name: str):
        ''' Request the robot to navigate to pose:LionsbotCoord 
            Return True if the robot has accepted the request, else False'''
        self.refresh_expired_token()

        navigate_content = NavigateContent(
            heading_radians=pose.orientation_radians,
            x=pose.x,
            y=pose.y,
            waypoint='Custom Move Point',
            waypoint_id=''
        )

        self.navigate_robot(robot_encoding_id=robot_name, time_stamp=time.time_ns() / 1000000, content=navigate_content)

        time_window_seconds = 5
        while time_window_seconds > 0:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return False
            
            robot_mission_details = robot_mission_status['missionStatus']['mission'] 
            mission_status = robot_mission_details['status']
            
            x = robot_mission_details.get('x', pose.x)
            y = robot_mission_details.get('y', pose.y)

            # Ensure mission status is from the mission moving to the current waypoint
            # Margin of error round to 1 between rmf waypoint coordinates and robot mission status coordinates
            # as there are times mission status feedback rounds of position to whole number
            is_correct_mission = int(x) == int(pose.x) and int(y) == int(pose.y) 

            curr_x = self.robot_pose.get(robot_name).x
            curr_y = self.robot_pose.get(robot_name).y
            is_robot_within_target_coord = abs(x - curr_x) <= 10 and abs(y - curr_y) <= 10

            # When coordinate is too near, possible to receive MOVING -> MOVING_FINISHED in quick succession. In this case, 
            # we take navigation to be successful 
            # if robot encountered error but robot is already near target position, treat as success as robot will not send
            # P2P_STARTED anymore when position is too close to target
            if (mission_status == RobotMissionStatus.MOVING.value or mission_status == RobotMissionStatus.MOVING_FINISHED.value) \
                and is_correct_mission and (ResponseCode.P2P_STARTED in robot_mission_status['alertIds'] or is_robot_within_target_coord):
                self.robot_current_map[robot_name] = map_name
                return True
            
            time.sleep(1)
            time_window_seconds -= 1

        return False

    def navigate_robot(self, robot_encoding_id: str, time_stamp: float, content: NavigateContent):
        payload = self._build_ws_command_payload(
            operation_cmd='p2p_start',
            robot_name=robot_encoding_id,
            time_stamp=time_stamp,
            content=content.__dict__)
        navigate_message = json.dumps(payload)
        self._log_ws_send('robotstatus', navigate_message)
        self.robot_status_ws_connection.send(navigate_message)

        # Save robot current state when command was sent
        # This is used to track if the robot has changed state or it has completed the command
        self.robot_navigate_state_id[robot_encoding_id] = self.robot_current_state_id.get(robot_encoding_id, uuid.uuid4())

        return True

    def navigation_completed(self, robot_name: str) -> NavigationStatus:
        self.refresh_expired_token()
        
        with self._lock:
            mission_status = self.get_mission_status(robot_name=robot_name)
            robot_status = self.get_robot_status(robot_name=robot_name)

        if mission_status is None or robot_status is None:
            return NavigationStatus.EMPTY
        
        robot_mission_status = mission_status['missionStatus']['mission']['status']
        robot_mission_details = mission_status['missionStatus']['mission']
        goal_x = robot_mission_details.get('x', 0)
        goal_y = robot_mission_details.get('y', 0)
        curr_x = self.robot_pose.get(robot_name).x
        curr_y = self.robot_pose.get(robot_name).y

        navigation_status = NavigationStatus.NAVIGATION_SUCCESS

        if robot_mission_status == RobotMissionStatus.MOVING_FINISHED.value:
            # Wait for state to change
            if self.robot_current_state_id.get(robot_name) == self.robot_navigate_state_id.get(robot_name):
                retries = 5
                # Return error if the robot fails to change state after 5[s]
                while self.robot_current_state_id.get(robot_name) == self.robot_navigate_state_id.get(robot_name):
                    if retries <= 0:
                        return NavigationStatus.NAVIGATION_ERROR
                    
                    time.sleep(1)
                    retries -= 1

                # If robot state has changed, indicates robot is navigating to the goal
                navigation_status = NavigationStatus.NAVIGATING
            
            # Wait for p2p_end_status to be received from the robot
            retries = 5
            while retries > 0:
                if self.robot_operation_end_status:
                    break
                
                time.sleep(1)
                retries -= 1

            # If robot is outside of goal tolerance, assume robot failed the navigation
            if (abs(goal_x-curr_x) > self.xy_goal_tolerance) or (abs(goal_y-curr_y) > self.xy_goal_tolerance):
                navigation_status = NavigationStatus.NAVIGATION_ERROR
            else:
                # If p2p end status was false, robot failed the navigation
                if not self.robot_operation_end_status.get('content', {}).get('status'):
                    navigation_status = NavigationStatus.NAVIGATION_ERROR
        else:
            navigation_status = NavigationStatus.NAVIGATING if robot_status['status'] == RobotStatus.MOVING.value \
            else NavigationStatus.NAVIGATION_ERROR
        
        return navigation_status

    def start_process(self, robot_name: str, process: str, map_name: str):
        ''' Request the robot to begin a process. This is specific to the robot
            and the use case. For example, load/unload a cart for Deliverybot
            or begin cleaning a zone for a cleaning robot.
            Return True if the robot has accepted the request, else False'''
        self.refresh_expired_token()

        self.clean(robot_encoding_id=robot_name,
                   clean_zone_name=process,
                   map_name=map_name,
                   time_stamp=time.time_ns() / 1000000)

        timeout = 5
        while True:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return False

            if robot_mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.CLEANING.value:
                return True
            else:
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    return False

    def clean(self, robot_encoding_id: str, clean_zone_name: str, map_name: str, time_stamp: float):
        clean_process_content = self.build_clean_process_content(robot_encoding_id=robot_encoding_id,
                                                                 map_name=map_name,
                                                                 clean_zone_name=clean_zone_name)

        payload = self._build_ws_command_payload(
            operation_cmd='clean_start',
            robot_name=robot_encoding_id,
            time_stamp=time_stamp,
            content=clean_process_content.__dict__)
        clean_message = json.dumps(payload)
        self._log_ws_send('robotstatus', clean_message)
        self.robot_status_ws_connection.send(clean_message)

        return True

    def build_clean_process_content(self, robot_encoding_id: str, map_name: str, clean_zone_name: str):
        robot_info = self.get_robot_info(robot_encoding_id)
        robot_type = robot_info['robotType']

        map_data = self.get_map(map_name=map_name, robot_name=robot_encoding_id)
        if map_data is None:
            return None

        map_id = map_data['id']
        map_level = map_data['level']
        map_zones = self.get_zones_by_map(map_id=map_id)

        while map_zones is None:
            map_zones = self.get_zones_by_map(map_id=map_id)
        
        filtered_zones = list(filter(lambda x: x['name'] == clean_zone_name, map_zones))

        clean_zone = filtered_zones[0]

        all_zone_equalizers = self.get_zone_equalizers(map_id=map_id, robot_name=robot_encoding_id)
        while all_zone_equalizers is None:
            all_zone_equalizers = self.get_zone_equalizers(map_id=map_id, robot_name=robot_encoding_id)

        selected_mode_id = all_zone_equalizers['selectedModeId']

        filtered_zone_equalizers = list(filter(lambda x: x['id'] == selected_mode_id, all_zone_equalizers['modes']))
        selected_zone_equalizers = filtered_zone_equalizers[0]

        configs = {}
        for config in selected_zone_equalizers['configs']:
            configs[config['configName']] = config['value']

        zones = [Zone(area=clean_zone['area'],
                      configs=configs,
                      selected_mode_name=selected_zone_equalizers['name'],
                      zone_name=clean_zone['name'],
                      zone_id=clean_zone['id']).__dict__]

        clean_process_content = CleanProcessContent(
            mode=0,
            robot_type=robot_type,
            working_type='point2clean',
            section_id='',
            section_name='',
            map_id=map_id,
            map_name=map_name,
            map_level=map_level,
            zones=zones,
            operator=self.user
        )

        return clean_process_content

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False'''
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status['status'] == RobotStatus.DOCKED.value or \
                robot_status['status'] == RobotStatus.RESTING.value:
            return True

        stop_process_content = StopProcessContent(
            status='true'
        )

        self.stop_robot_moving(robot_name=robot_name, time_stamp=time.time_ns() / 1000000,
                               content=stop_process_content)
        self.stop_robot_cleaning(robot_name=robot_name, time_stamp=time.time_ns() / 1000000,
                                 content=stop_process_content)
        self.stop_robot_docking(robot_name=robot_name, time_stamp=time.time_ns() / 1000000,
                                content=stop_process_content)

        time.sleep(0.5)

        timeout = 5
        while True:
            robot_mission_status = self.get_mission_status(robot_name=robot_name)
            if robot_mission_status is None:
                return False

            if robot_mission_status['missionStatus']['activeMissionType'] == ActiveMissionType.IDLE.value or \
                robot_mission_status['missionStatus']['mission']['status'] in RobotAPI.TERMINAL_MISSION_STATUSES:
                return True
            else:
                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    self.robot_mission[robot_name] = None
                    return False

    def stop_robot_moving(self, robot_name: str, time_stamp: float, content: StopProcessContent):
        payload = self._build_ws_command_payload(
            operation_cmd='p2p_stop',
            robot_name=robot_name,
            time_stamp=time_stamp,
            content=content.__dict__)
        stop_message = json.dumps(payload)
        self._log_ws_send('robotstatus', stop_message)
        self.robot_status_ws_connection.send(stop_message)

        return True

    def stop_robot_cleaning(self, robot_name: str, time_stamp: float, content: StopProcessContent):
        payload = self._build_ws_command_payload(
            operation_cmd='clean_stop',
            robot_name=robot_name,
            time_stamp=time_stamp,
            content=content.__dict__)
        stop_message = json.dumps(payload)
        self._log_ws_send('robotstatus', stop_message)
        self.robot_status_ws_connection.send(stop_message)

        return True

    def stop_robot_docking(self, robot_name: str, time_stamp: float, content: StopProcessContent):
        payload = self._build_ws_command_payload(
            operation_cmd='dock_stop',
            robot_name=robot_name,
            time_stamp=time_stamp,
            content=content.__dict__)
        stop_message = json.dumps(payload)
        self._log_ws_send('robotstatus', stop_message)
        self.robot_status_ws_connection.send(stop_message)

        return True

    def process_completed(self, robot_name: str):
        ''' Return True if the robot has successfully completed cleaning. Else False.'''
        self.refresh_expired_token()

        mission_status = self.get_mission_status(robot_name)

        if mission_status is None:
            return False

        if mission_status['missionStatus']['mission']['status'] == RobotMissionStatus.CLEANING_FINISHED.value :
            return True

        return False

    def undocking_completed(self, robot_name: str):
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name)
        while robot_status is None:
            robot_status = self.get_robot_status(robot_name)

        if robot_status is None:
            return False

        if robot_status['status'] != RobotStatus.DOCKED.value or \
                robot_status['status'] == RobotStatus.RESTING.value:
            return True
        return False

    def docking_completed(self, robot_name: str):
        self.refresh_expired_token()

        robot_status = self.get_robot_status(robot_name)
        while robot_status is None:
            robot_status = self.get_robot_status(robot_name)

        if robot_status is None:
            return False
        
        if robot_status['status'] == RobotStatus.DOCKED.value:
            return True
        return False

    def e_stop_robot(self, robot_name: str, time_stamp: float):
        payload = self._build_ws_command_payload(
            operation_cmd='mode_estop',
            robot_name=robot_name,
            time_stamp=time_stamp,
            content={'status': 'true'})
        estop_message = json.dumps(payload)
        self._log_ws_send('robotstatus', estop_message)
        self.robot_status_ws_connection.send(estop_message)

        return True

    def pause_robot(self, robot_name: str):
        time_stamp = time.time_ns() / 1000000

        payload = {
            'operation_cmd': None,
            'robot_id': robot_name,
            'time_stamp': time_stamp,
            'content': {'status': True}
        }

        robot_status = self.get_robot_status(robot_name=robot_name)

        operation_cmd = None

        if robot_status['status'] == RobotStatus.MOVING.value:
            operation_cmd = 'p2p_pause'
        elif robot_status['status'] == RobotStatus.CLEANING.value:
            operation_cmd = 'clean_pause'
        else:
            return False

        payload['operation_cmd'] = operation_cmd
        pause_message = json.dumps(payload)
        self._log_ws_send('robotstatus', pause_message)

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': operation_cmd,
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(pause_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False
            elif self.robot_cleaning_paused(robot_name) or self.robot_moving_paused(robot_name):
                with self._lock:
                    self.robot_operation[robot_name] = None
                    return True

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def resume_robot(self, robot_name: str):
        time_stamp = time.time_ns() / 1000000

        payload = {
            'operation_cmd': None,
            'robot_id': robot_name,
            'time_stamp': time_stamp,
            'content': {'status': True}
        }

        operation_cmd = None

        if self.robot_moving_paused(robot_name):
            operation_cmd = 'p2p_continue'
        elif self.robot_cleaning_paused(robot_name=robot_name):
            operation_cmd = 'clean_continue'
        else:
            return False

        payload['operation_cmd'] = operation_cmd
        resume_message = json.dumps(payload)
        self._log_ws_send('robotstatus', resume_message)

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': operation_cmd,
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(resume_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)
            robot_status = self.get_robot_status(robot_name=robot_name)

            if robot_operation is None:
                return False

            elif robot_status['status'] in RobotAPI.ROBOT_ACTIVE_STATUSES:
                with self._lock:
                    self.robot_operation[robot_name] = None
                    return True

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def robot_cleaning(self, robot_name:str):
        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is None: 
            return False
        
        return robot_status['status'] == RobotStatus.CLEANING.value
        
    def robot_cleaning_paused(self, robot_name:str):
        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is None: 
            return False
        
        return robot_status['status'] == RobotStatus.CLEANING_PAUSED.value

    def robot_moving_paused(self, robot_name:str):
        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is None: 
            return False
        
        return robot_status['status'] == RobotStatus.MOVING_PAUSED.value   

    def dock_robot(self, robot_name: str, time_stamp: float, content: DockProcessContent):
        robot_status = self.get_robot_status(robot_name=robot_name)
        if robot_status is not None and robot_status['status'] == RobotStatus.DOCKED.value:
            return True

        payload = self._build_ws_command_payload(
            operation_cmd='dock_start',
            robot_name=robot_name,
            time_stamp=time_stamp,
            content=content.__dict__)
        dock_message = json.dumps(payload)
        self._log_ws_send('robotstatus', dock_message)

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': 'dock_start',
                'time_stamp': time_stamp,
                'status': None
            }
            
        self.robot_status_ws_connection.send(dock_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False
            elif robot_operation['operation'] == 'dock_start' and robot_operation['status'] is not None:
                with self._lock:
                    if robot_operation['status']:
                        self.robot_operation[robot_name] = None
                        return True

                    self.robot_operation[robot_name] = None
                    return False
            elif self.docking_completed(robot_name=robot_name):
                return True

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def undock_robot(self, robot_name: str, time_stamp: float):
        payload = self._build_ws_command_payload(
            operation_cmd='undock_start',
            robot_name=robot_name,
            time_stamp=time_stamp,
            content={'status': 'true'})
        dock_message = json.dumps(payload)
        self._log_ws_send('robotstatus', dock_message)

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': 'undock_start',
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(dock_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False

            elif robot_operation['operation'] == 'undock_start' and robot_operation['status'] is not None:
                with self._lock:
                    if robot_operation['status']:
                        self.robot_operation[robot_name] = None
                        return True

                    self.robot_operation[robot_name] = None
                    return False
            elif self.undocking_completed(robot_name=robot_name):
                return True

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def stop_docking(self, robot_name: str, time_stamp: float):
        payload = self._build_ws_command_payload(
            operation_cmd='dock_stop',
            robot_name=robot_name,
            time_stamp=time_stamp,
            content={'status': 'true'})
        stop_dock_message = json.dumps(payload)
        self._log_ws_send('robotstatus', stop_dock_message)

        with self._lock:
            self.robot_operation[robot_name] = {
                'operation': 'dock_stop',
                'time_stamp': time_stamp,
                'status': None
            }
        self.robot_status_ws_connection.send(stop_dock_message)

        timeout = 5
        while True:
            robot_operation = self.robot_operation.get(robot_name, None)

            if robot_operation is None:
                return False

            elif robot_operation['operation'] == 'dock_stop' and robot_operation['status'] is not None:
                with self._lock:
                    if robot_operation['status']:
                        self.robot_operation[robot_name] = None
                        return True

                    self.robot_operation[robot_name] = None
                    return False

            time.sleep(1)
            timeout -= 1
            if timeout == 0:
                with self._lock:
                    self.robot_operation[robot_name] = None
                return False

    def localize(self, position: LionsbotCoord, robot_name: str) -> bool:
        self.refresh_expired_token()

        path = f'{constants.OPEN_API_PREFIX}/robot/command/hot-localize/{robot_name}'
        headers = self._bearer_headers()

        payload = {'x': position.x, 'y': position.y, 'heading': position.orientation_radians}

        try:
            r = self._put(path=path, headers=headers, json=payload)
            r.raise_for_status()
            data = r.json()

            return data['success']
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return False

    def change_map(self, robot_name: str, map_name: str):
        self.refresh_expired_token()

        map_data = self.get_map(map_name=map_name, robot_name=robot_name)
        if map_data is None:
            return False

        map_id = map_data['id']

        path = f'{constants.OPEN_API_PREFIX}/robot/{robot_name}/map/{map_id}'
        headers = self._bearer_headers()

        try:
            r = self._put(path=path, headers=headers)
            r.raise_for_status()

            timeout = 4
            while True:
                robot_maps = self.get_robot_maps(robot_name=robot_name)
                if robot_maps is None:
                    return False

                current_map_id = robot_maps['selectedMapId']
                if current_map_id == map_id:
                    return True

                time.sleep(1)
                timeout -= 1
                if timeout == 0:
                    return False
        except requests.exceptions.ConnectionError as connection_error:
            print(f'Connection error: {connection_error}')
        except HTTPError as http_err:
            print(f'HTTP error: {http_err}')

        return False
