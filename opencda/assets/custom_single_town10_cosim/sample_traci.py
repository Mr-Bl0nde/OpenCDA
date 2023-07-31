# https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html
import os
import sys
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import traci.constants as tc

# Set up the SUMO executable (e.g. SUMO or SUMO-GUI) and your .sumocfg file
sumoBinary = "C:/Apps/SUMO/sumo-win64extra-1.16.0/bin/sumo-gui.exe"
sumoCfg = "C:/Apps/OpenCDA/opencda/assets/custom_single_town10_cosim/custom_single_town10_cosim_modded.sumocfg"
sumoCmd = [sumoBinary, "-c", sumoCfg]

# Run the virtual SUMO CLI command defined above to start sumo-gui and initialize it with the .sumocfg parameters 
# Since sumo-gui is configured to run, you need to hit 'play' in order for the simulation to actually run
traci.start(sumoCmd)

# Various tracking variables
step = 0                        # Initial step value to increment on
stepLength = 0.05               # Step length in seconds
stepCount = 0                   # Step counter to find n whole integer steps
current_vehicle_count = 0       # Count of vehicles from the previous step leading into a new step for comparison 
current_vehicle_id_list = []    # List of vehicle IDs from the previous step leading into a new step for comparison
removed_vehicles = []           # List of vehicles that have been removed from the simulation for any reason at any point during the simulation

# Currently limits simulation time to 1000s, which tbh is probably sufficient for my research, so I don't really need to be doing so much to avoid deadlock after 1500s or so
# https://sumo.dlr.de/docs/TraCI/Interfacing_TraCI_from_Python.html#subscriptions
while step < 1000:
    # Record the count and IDs of vehicles in the current step
    vehicle_count = traci.vehicle.getIDCount()
    vehicle_id_list = traci.vehicle.getIDList()

    # Stringify the current step integer, time (step), and vehicle count for display
    # map() takes a function and an iterable list. It runs the function on every member of the list. 
    # I passed an anonymous lambda function, whose syntax is like --> lambda RETURN VARIABLE "x": DOSOMETHING(to "x")
    strList = map(lambda x:str(x), [stepCount, step, vehicle_count])
    # print(f"STEP: {strList[0]}\tTIME: {strList[1]}\tN_VEHICLES: {strList[2]}") # idk why f-strings aren't working
    print('STEP: {0}\tTIME: {1}\tN_VEHICLES: {2}'.format(strList[0], strList[1], strList[2]))

    # Create a check for any vehicle removals because I can't find any native TraCI functions to do this lol
    # Logic: if this step's vehicle count is less than the last step, a vehicle has been removed...
    # Compute the difference between the list of vehicles from the last step (current_vehicle_id_list) and the list of vehicles from this step (vehicle_id_list)
    # List comparison logic courtesy of https://stackoverflow.com/questions/3428536/how-do-i-subtract-one-list-from-another
    # N_VEHICLES drops from 78 to 77 at t=521.95
    if vehicle_count < current_vehicle_count:
        removed_vehicles = [v for v in current_vehicle_id_list if v not in vehicle_id_list]
        print("\n\nREMOVED: {}\n\n".format(removed_vehicles))

    # Update "last step's values" for the next step
    current_vehicle_count = vehicle_count
    current_vehicle_id_list = traci.vehicle.getIDList()

    # Execute the next step and increase the step tracker
    traci.simulationStep()
    step += stepLength
    stepCount += 1

traci.close()