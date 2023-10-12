#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
""" This module is responsible for the management of the sumo simulation. """

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import collections
import enum
import logging
import os

import carla  # pylint: disable=import-error
import sumolib  # pylint: disable=import-error
import traci  # pylint: disable=import-error

from opencda.co_simulation.sumo_integration.constants import INVALID_ACTOR_ID

import lxml.etree as ET  # pylint: disable=import-error

# ==================================================================================================
# -- sumo definitions ------------------------------------------------------------------------------
# ==================================================================================================


# https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html#signal_state_definitions
class SumoSignalState(object):
    """
    SumoSignalState contains the different traffic light states.
    """
    RED = 'r'
    YELLOW = 'y'
    GREEN = 'G'
    GREEN_WITHOUT_PRIORITY = 'g'
    GREEN_RIGHT_TURN = 's'
    RED_YELLOW = 'u'
    OFF_BLINKING = 'o'
    OFF = 'O'


# https://sumo.dlr.de/docs/TraCI/Vehicle_Signalling.html
class SumoVehSignal(object):
    """
    SumoVehSignal contains the different sumo vehicle signals.
    """
    BLINKER_RIGHT = 1 << 0
    BLINKER_LEFT = 1 << 1
    BLINKER_EMERGENCY = 1 << 2
    BRAKELIGHT = 1 << 3
    FRONTLIGHT = 1 << 4
    FOGLIGHT = 1 << 5
    HIGHBEAM = 1 << 6
    BACKDRIVE = 1 << 7
    WIPER = 1 << 8
    DOOR_OPEN_LEFT = 1 << 9
    DOOR_OPEN_RIGHT = 1 << 10
    EMERGENCY_BLUE = 1 << 11
    EMERGENCY_RED = 1 << 12
    EMERGENCY_YELLOW = 1 << 13


# https://sumo.dlr.de/docs/Definition_of_Vehicles,_Vehicle_Types,_and_Routes.html#abstract_vehicle_class
class SumoActorClass(enum.Enum):
    """
    SumoActorClass enumerates the different sumo actor classes.
    """
    IGNORING = "ignoring"
    PRIVATE = "private"
    EMERGENCY = "emergency"
    AUTHORITY = "authority"
    ARMY = "army"
    VIP = "vip"
    PEDESTRIAN = "pedestrian"
    PASSENGER = "passenger"
    HOV = "hov"
    TAXI = "taxi"
    BUS = "bus"
    COACH = "coach"
    DELIVERY = "delivery"
    TRUCK = "truck"
    TRAILER = "trailer"
    MOTORCYCLE = "motorcycle"
    MOPED = "moped"
    BICYCLE = "bicycle"
    EVEHICLE = "evehicle"
    TRAM = "tram"
    RAIL_URBAN = "rail_urban"
    RAIL = "rail"
    RAIL_ELECTRIC = "rail_electric"
    RAIL_FAST = "rail_fast"
    SHIP = "ship"
    CUSTOM1 = "custom1"
    CUSTOM2 = "custom2"


SumoActor = collections.namedtuple('SumoActor', 'type_id vclass transform signals extent color')

# ==================================================================================================
# -- sumo traffic lights ---------------------------------------------------------------------------
# ==================================================================================================


class SumoTLLogic(object):
    """
    SumoTLLogic holds the data relative to a traffic light in sumo.
    """
    def __init__(self, tlid, states, parameters, net):
        self.net = net  # ADDITION: passed down the .net.xml object originally read in SumoSimulation obj. This will be used to associate coordinates for all SUMO traffic lights
        self.tlid = tlid
        self.states = states
        self._landmark2link = {}
        self._link2landmark = {}

        for link_index, landmark_id in parameters.items():
            # Link index information is added in the parameter as 'linkSignalID:x'
            link_index = int(link_index.split(':')[1])

            if landmark_id not in self._landmark2link:
                self._landmark2link[landmark_id] = []
            self._landmark2link[landmark_id].append((tlid, link_index))
            self._link2landmark[(tlid, link_index)] = landmark_id

    def get_number_signals(self):
        """
        Returns number of internal signals of the traffic light.
        """
        if len(self.states) > 0:
            return len(self.states[0])
        return 0

    def get_all_signals(self):
        """
        Returns all the signals of the traffic light.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        return [(self.tlid, i) for i in range(self.get_number_signals())]

    def get_all_landmarks(self):
        """
        Returns all the landmarks associated with this traffic light.
        """
        return self._landmark2link.keys()

    def get_associated_signals(self, landmark_id):
        """
        Returns all the signals associated with the given landmark.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        return self._landmark2link.get(landmark_id, [])



class SumoTLManager(object):
    """
    SumoTLManager is responsible for the management of the sumo traffic lights (i.e., keeps control
    of the current program, phase, ...)
    """
    def __init__(self, net):
        self.net = net
        self._tls = {}  # {tlid: {program_id: SumoTLLogic}
        self._current_program = {}  # {tlid: program_id}
        self._current_phase = {}  # {tlid: index_phase}

        for tlid in traci.trafficlight.getIDList():
            self.subscribe(tlid)
            self._tls[tlid] = {}
            for tllogic in traci.trafficlight.getAllProgramLogics(tlid):
                states = [phase.state for phase in tllogic.getPhases()]
                parameters = tllogic.getParameters()
                tl = SumoTLLogic(tlid, states, parameters, self.net)
                self._tls[tlid][tllogic.programID] = tl

            # Get current status of the traffic lights.
            self._current_program[tlid] = traci.trafficlight.getProgram(tlid)
            self._current_phase[tlid] = traci.trafficlight.getPhase(tlid)

        self._off = False

    @staticmethod
    def subscribe(tlid):
        """
        Subscribe the given traffic ligth to the following variables:

            * Current program.
            * Current phase.
        """
        traci.trafficlight.subscribe(tlid, [
            traci.constants.TL_CURRENT_PROGRAM,
            traci.constants.TL_CURRENT_PHASE,
        ])

    @staticmethod
    def unsubscribe(tlid):
        """
        Unsubscribe the given traffic ligth from receiving updated information each step.
        """
        traci.trafficlight.unsubscribe(tlid)

    def get_all_signals(self):
        """
        Returns all the traffic light signals.
        """
        signals = set()
        for tlid, program_id in self._current_program.items():
            signals.update(self._tls[tlid][program_id].get_all_signals())
        return signals

    def get_all_landmarks(self):
        """
        Returns all the landmarks associated with a traffic light in the simulation.
        """
        landmarks = set()
        for tlid, program_id in self._current_program.items():
            landmarks.update(self._tls[tlid][program_id].get_all_landmarks())
        return landmarks

    def get_all_associated_signals(self, landmark_id):
        """
        Returns all the signals associated with the given landmark.
            :returns list: [(tlid, link_index), (tlid, link_index), ...]
        """
        signals = set()
        for tlid, program_id in self._current_program.items():
            signals.update(self._tls[tlid][program_id].get_associated_signals(landmark_id))
        return signals

    def get_state(self, landmark_id):
        """
        Returns the traffic light state of the signals associated with the given landmark.
        """
        states = set()
        for tlid, link_index in self.get_all_associated_signals(landmark_id):
            current_program = self._current_program[tlid]
            current_phase = self._current_phase[tlid]

            tl = self._tls[tlid][current_program]
            states.update(tl.states[current_phase][link_index])

        if len(states) == 1:
            return states.pop()
        elif len(states) > 1:
            logging.warning('Landmark %s is associated with signals with different states',
                            landmark_id)
            return SumoSignalState.RED
        else:
            return None

    def set_state(self, landmark_id, state):
        """
        Updates the state of all the signals associated with the given landmark.
        """
        for tlid, link_index in self.get_all_associated_signals(landmark_id):
            traci.trafficlight.setLinkState(tlid, link_index, state)
        return True
    
    def set_states_from_sumo(self, world):   
        """
        Sets a CARLA traffic light's state from the corresponding SUMO traffic light
            1) Get all carla tls from self._tls
            2) Get all SUMO signal states
            3) For each carla tl, set it to the corresponding sumo tl signal
        :param world: The CARLA world object containing data and access methods for the active simulation environment
        """

        # LOOP THROUGH NETWORK JUNCTIONS WITH TRAFFIC LIGHTS (c_tl). Labeled c_tl because it represents a traffic light group in CARLA and corresponds to a single junction in SUMO
        # Number of iterations equal to the number of signalized intersections in a map
        for c_tl in self._tls.items():
            # c_tl structure is like: 
            # ('725', {'0': <opencda.co_simulation.sumo_integration.sumo_simulation.SumoTLLogic object at 0x000001F884D84A08>})
            # Thus c_tl[0] is the SUMO Traffic Light ID visible in SUMO-GUI
            # c_tl[1] is a dictionary with the traffic program index (just 0 unless multiple programs are defined) and the SumoTLLogic object created to represent that light program in CARLA. This object contains a list of individual lights and their associations between the programs.
            # Furthermore, the landmark2link list of each program (e.g. {'1032': [('918', 0), ('918', 1)], '1034': [('918', 2), ('918', 3)], ...}) can be accessed with c_tl[1][i]._landmark2link
            # This dictionary says CARLA landmark id 1032 (a signal asset) controls flow on linkSignalID:0 and 1 for SUMO junction 918

            # LOOP THROUGH PROGRAMS ASSOCIATED WITH EACH JUNCTION.
            # Access the landmark2link list of each program associated with this traffic light (in case multiple are assigned)
            for i in c_tl[1]:
                # Get sumo traffic light state for this whole junction - states are in order of link index e.g. GGgrrrGGgrrr = 0(G), 1(G), 2(g), 3(r), ...
                traci_junction_state = traci.trafficlight.getRedYellowGreenState(c_tl[0])
                sumo_junction_program = c_tl[1][i]
                landmark2link = sumo_junction_program._landmark2link

                # LOOP THROUGH TRAFFIC LIGHTS CONTROLLING EACH ENTRY POINT INTO THE JUNCTION
                # Parse the landmark2link list in order to assign individual carla lights (c_tlid) their corresponding signal states from SUMO
                for c_tlid in landmark2link:
                    # Re-sort the list by the second value in each tuple (the link index) because they were somehow sorted by ACII character
                    # e.g. [('725', 10), ('725', 11), ('725', 9)]  --> [('725', 9), ('725', 10), ('725', 11)]
                    resorted_idx_list = sorted(landmark2link[c_tlid], key=lambda x: x[1])
                    link2sumostate = []

                    # LOOP THROUGH POSSIBLE OUTLET DIRECTIONS FROM EACH ENTRY POINT INTO THE JUNCTION
                    # Populate a list of SUMO light states for this particular CARLA traffic light in right, straight, left turn order
                    for junction_plus_idx in resorted_idx_list:
                        # Get a traffic light Actor object corresponding to this CARLA tlid from the world object.
                        carla_tl_to_change = world.get_traffic_light_from_opendrive_id(sumo_junction_program._link2landmark[junction_plus_idx])
                        sumo_state_for_this_linkSignalID = traci_junction_state[junction_plus_idx[1]]     # e.g. 'GGrrGG'[2] --> 'r'
                        # NOTE: this is where a granular function to set the state for every traffic link would be implemented
                        # Instead I infer what the majority of light states are and determine what the whole landmark state should be
                        link2sumostate.append(sumo_state_for_this_linkSignalID)    # Create a simple list for the sumo state chars e.g. 'r' or 'G' in order of their link index for this one light

                    # Handle light state translation to carla based on whether there are 2 or 3 directions of travel. 
                    # NOTE: Not currently able to handle n-directional lights.
                    # NOTE: This involves some loss of precision and may lead to non-yielding green behavior among carla vehicles where there are conflicting traffic streams
                    if len(resorted_idx_list) == 3:
                        # I assume the equivalent carla state from the middle[1] (straight) lane traffic state from SUMO, as there is no granular control (currently) for controlling each direction
                        target_carla_tl_state = self.get_carla_traffic_light_state(link2sumostate[1])
                        carla_tl_to_change.set_state(target_carla_tl_state)

                    elif len(resorted_idx_list) == 2:
                        print("2 directions at this light")
                        print(f"link2sumostate: {link2sumostate}")
                        # For lanes with two possible directions and AT LEAST ONE direction is red, set both options in CARLA to red. Conservative option in case of split green/red scenarios
                        if 'r' in link2sumostate:
                            carla_tl_to_change.set_state(carla.TrafficLightState.Red)
                        # If there are two possible directions and neither is red (may be yellow, green, yielding green, or other) set the carla tl to whatever state get_carla_traffic_light_state() interprets the first SUMO state to be
                        else:
                            target_carla_tl_state = self.get_carla_traffic_light_state(link2sumostate[0])
                            carla_tl_to_change.set_state(target_carla_tl_state)
                            
                    else:
                        print("something other than 2 or 3 light directions ...")
                        print(f"link2sumostate: {link2sumostate}")
                        raise Exception("There are either less than two or greater than three directions of travel from this light.")


    # Copied this method to this object to avoid circular dependency. Originally found in bridge_helper.py as a static method of BridgeHelper
    @staticmethod
    def get_carla_traffic_light_state(sumo_tl_state):
        """
        Returns carla traffic light state based on sumo traffic light state.
        """
        if sumo_tl_state == SumoSignalState.RED or sumo_tl_state == SumoSignalState.RED_YELLOW:
            return carla.TrafficLightState.Red

        elif sumo_tl_state == SumoSignalState.YELLOW:
            return carla.TrafficLightState.Yellow

        elif sumo_tl_state == SumoSignalState.GREEN or \
             sumo_tl_state == SumoSignalState.GREEN_WITHOUT_PRIORITY:
            return carla.TrafficLightState.Green

        elif sumo_tl_state == SumoSignalState.OFF:
            return carla.TrafficLightState.Off

        else:  # SumoSignalState.GREEN_RIGHT_TURN and SumoSignalState.OFF_BLINKING
            return carla.TrafficLightState.Unknown

    def switch_off(self):
        """
        Switch off all traffic lights.
        """
        for tlid, link_index in self.get_all_signals():
            traci.trafficlight.setLinkState(tlid, link_index, SumoSignalState.OFF)
        self._off = True

    def tick(self):
        """
        Tick to traffic light manager
        """
        if self._off is False:
            for tl_id in traci.trafficlight.getIDList():
                results = traci.trafficlight.getSubscriptionResults(tl_id)
                current_program = results[traci.constants.TL_CURRENT_PROGRAM]
                current_phase = results[traci.constants.TL_CURRENT_PHASE]

                if current_program != 'online':
                    self._current_program[tl_id] = current_program
                    self._current_phase[tl_id] = current_phase


# ==================================================================================================
# -- sumo simulation -------------------------------------------------------------------------------
# ==================================================================================================

def _get_sumo_net(cfg_file):
    """
    Returns sumo net.

    This method reads the sumo configuration file and retrieve the sumo net filename to create the
    net.
    """
    cfg_file = os.path.join(os.getcwd(), cfg_file)

    tree = ET.parse(cfg_file)
    tag = tree.find('//net-file')
    if tag is None:
        return None

    net_file = os.path.join(os.path.dirname(cfg_file), tag.get('value'))
    logging.debug('Reading net file: %s', net_file)

    sumo_net = sumolib.net.readNet(net_file)
    return sumo_net

class SumoSimulation(object):
    """
    SumoSimulation is responsible for the management of the sumo simulation.
    """
    def __init__(self, cfg_file, step_length, host=None, port=None, sumo_gui=False, client_order=1):
        if sumo_gui is True:
            sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            sumo_binary = sumolib.checkBinary('sumo')

        if host is None or port is None:
            logging.info('Starting new sumo server...')
            if sumo_gui is True:
                logging.info('Remember to press the play button to start the simulation')

            traci.start([sumo_binary,
                '--configuration-file', cfg_file,
                '--step-length', str(step_length),
                '--collision.check-junctions'
            ])

        else:
            logging.info('Connection to sumo server. Host: %s Port: %s', host, port)
            traci.init(host=host, port=port)

        traci.setOrder(client_order)

        # Retrieving net from configuration file.
        self.net = _get_sumo_net(cfg_file)

        # Creating a random route to be able to spawn carla actors.
        traci.route.add("carla_route", [traci.edge.getIDList()[0]])

        # Variable to asign an id to new added actors.
        self._sequential_id = 0

        # Structures to keep track of the spawned and destroyed vehicles at each time step.
        self.spawned_actors = set()
        self.destroyed_actors = set()

        # Traffic light manager.
        self.traffic_light_manager = SumoTLManager(self.net)
        # ADDITION: Modified SumoTLManager to pass the read .net.xml obj to the TLManager upon instantiation

    @property
    def traffic_light_ids(self):
        return self.traffic_light_manager.get_all_landmarks()

    @staticmethod
    def subscribe(actor_id):
        """
        Subscribe the given actor to the following variables:

            * Type.
            * Vehicle class.
            * Color.
            * Length, Width, Height.
            * Position3D (i.e., x, y, z).
            * Angle, Slope.
            * Speed.
            * Lateral speed.
            * Signals.
        """
        traci.vehicle.subscribe(actor_id, [
            traci.constants.VAR_TYPE, traci.constants.VAR_VEHICLECLASS, traci.constants.VAR_COLOR,
            traci.constants.VAR_LENGTH, traci.constants.VAR_WIDTH, traci.constants.VAR_HEIGHT,
            traci.constants.VAR_POSITION3D, traci.constants.VAR_ANGLE, traci.constants.VAR_SLOPE,
            traci.constants.VAR_SPEED, traci.constants.VAR_SPEED_LAT, traci.constants.VAR_SIGNALS
        ])

    @staticmethod
    def unsubscribe(actor_id):
        """
        Unsubscribe the given actor from receiving updated information each step.
        """
        traci.vehicle.unsubscribe(actor_id)

    def get_net_offset(self):
        """
        Accessor for sumo net offset.
        """
        if self.net is None:
            return (0, 0)
        return self.net.getLocationOffset()

    @staticmethod
    def get_actor(actor_id):
        """
        Accessor for sumo actor.
        """
        results = traci.vehicle.getSubscriptionResults(actor_id)

        type_id = results[traci.constants.VAR_TYPE]
        vclass = SumoActorClass(results[traci.constants.VAR_VEHICLECLASS])
        color = results[traci.constants.VAR_COLOR]

        length = results[traci.constants.VAR_LENGTH]
        width = results[traci.constants.VAR_WIDTH]
        height = results[traci.constants.VAR_HEIGHT]

        location = list(results[traci.constants.VAR_POSITION3D])
        rotation = [results[traci.constants.VAR_SLOPE], results[traci.constants.VAR_ANGLE], 0.0]
        transform = carla.Transform(carla.Location(location[0], location[1], location[2]),
                                    carla.Rotation(rotation[0], rotation[1], rotation[2]))

        signals = results[traci.constants.VAR_SIGNALS]
        extent = carla.Vector3D(length / 2.0, width / 2.0, height / 2.0)

        return SumoActor(type_id, vclass, transform, signals, extent, color)

    def spawn_actor(self, type_id, color=None):
        """
        Spawns a new actor.

            :param type_id: vtype to be spawned.
            :param color: color attribute for this specific actor.
            :return: actor id if the actor is successfully spawned. Otherwise, INVALID_ACTOR_ID.
        """
        actor_id = 'carla' + str(self._sequential_id)
        try:
            traci.vehicle.add(actor_id, 'carla_route', typeID=type_id)
        except traci.exceptions.TraCIException as error:
            logging.error('Spawn sumo actor failed: %s', error)
            return INVALID_ACTOR_ID

        if color is not None:
            color = color.split(',')
            traci.vehicle.setColor(actor_id, color)

        self._sequential_id += 1

        return actor_id

    @staticmethod
    def destroy_actor(actor_id):
        """
        Destroys the given actor.
        """
        traci.vehicle.remove(actor_id)

    def get_traffic_light_state(self, landmark_id):
        """
        Accessor for traffic light state.

        If the traffic ligth does not exist, returns None.
        """
        return self.traffic_light_manager.get_state(landmark_id)

    def switch_off_traffic_lights(self):
        """
        Switch off all traffic lights.
        """
        self.traffic_light_manager.switch_off()

    def synchronize_vehicle(self, vehicle_id, transform, signals=None):
        """
        Updates vehicle state.

            :param vehicle_id: id of the actor to be updated.
            :param transform: new vehicle transform (i.e., position and rotation).
            :param signals: new vehicle signals.
            :return: True if successfully updated. Otherwise, False.
        """
        loc_x, loc_y = transform.location.x, transform.location.y
        yaw = transform.rotation.yaw

        traci.vehicle.moveToXY(vehicle_id, "", 0, loc_x, loc_y, angle=yaw, keepRoute=2)
        if signals is not None:
            traci.vehicle.setSignals(vehicle_id, signals)
        return True

    def synchronize_traffic_light(self, landmark_id, state):
        """
        Updates traffic light state.

            :param landmark_id: id of the traffic light to be updated (logic id, link index).
            :param state: new traffic light state.
            :return: True if successfully updated. Otherwise, False.
        """
        self.traffic_light_manager.set_state(landmark_id, state)

    def tick(self):
        """
        Tick to sumo simulation.
        """
        traci.simulationStep()
        self.traffic_light_manager.tick()

        # Update data structures for the current frame.
        self.spawned_actors = set(traci.simulation.getDepartedIDList())
        self.destroyed_actors = set(traci.simulation.getArrivedIDList())

    @staticmethod
    def close():
        """
        Closes traci client.
        """
        traci.close()
