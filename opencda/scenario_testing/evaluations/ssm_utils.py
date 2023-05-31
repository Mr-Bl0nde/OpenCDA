# -*- coding: utf-8 -*-
"""
Tools for processing surrogate safety measure (SSM) data produced in OpenCDA scenarios
"""

# Author: Dustin Smith <dustinsmith720@gmail.com>
# License: TDG-Attribution-NonCommercial-NoDistrib

# Imports
from bs4 import BeautifulSoup
from csv import writer

def ssm_log_to_csv(log_path, out_path):
    """
    Convert SSM log from XML to CSV format for selected attributes. Not flexible to different SSM parameters at this point.

    Args:
        - log_path (str): string path to the xml log FILE being processed
        - out_path (str): string path to the output csv DIRECTORY
    """

    # Check log_path and out_path for format and file existence
    try:
        print('Converting SSM logs...')

        # Parse the xml document into an object we can work with in this script
        with open(log_path, newline='') as log_file:
            log = log_file.read()

        bs_data = BeautifulSoup(log, "xml")
        conflicts = bs_data.find_all('conflict')
        # Accessing a beautifulsoup object is documented at the following:
        # https://beautiful-soup-4.readthedocs.io/en/latest/index.html?highlight=beautifulsoup()#beautifulsoup
        # https://beautiful-soup-4.readthedocs.io/en/latest/index.html?highlight=beautifulsoup()#searching-the-tree
        # https://beautiful-soup-4.readthedocs.io/en/latest/index.html?highlight=beautifulsoup()#navigating-the-tree


        # Get all values of all SSM log tags
        i = 0
        for conflict in conflicts:
            # Parse the incoming file output dir to accommodate multiple conflict files
            time_coordinated_out_path = out_path + '\conflict_' + str(i) + '_time_coordinated.csv'
            extremals_out_path = out_path + '\conflict_' + str(i) + '_extremals.csv'
                
            # Create a csv object at out_path
            with open(time_coordinated_out_path, 'w', newline='') as out_csvfile:
                log_writer = writer(out_csvfile)

                # Get a list of time step-coordinated elements that will only have one attribute "values"; extract specific attributes from the others
                single_attrs = [
                    conflict.timeSpan, 
                    conflict.typeSpan, 
                    conflict.egoPosition,
                    conflict.egoLane, 
                    conflict.egoLanePosition, 
                    conflict.egoVelocity, 
                    conflict.foePosition, 
                    conflict.foeLane, 
                    conflict.foeLanePosition, 
                    conflict.foeVelocity, 
                    conflict.conflictPoint, 
                    conflict.TTCSpan,
                    conflict.DRACSpan
                ]

                # Get a big list of values where each attribute comes in as a row, which we will want to transpose
                step_list = []
                header_row = []
                for attr in single_attrs:
                    # Traversing down the element tree can be done with dot notation.
                    # Getting attribute values requires string slicing
                    step_list.append(attr['values'].split(' '))

                    # Get a list of the attribute names for a header row
                    header_row.append(attr.name)

                # Write the header row first
                log_writer.writerow(header_row)

                # Transpose the big list of attribute values and write them to this conflict's CSV file object. Outputs 'zip' type object
                step_list = zip(*step_list)
                for step in step_list:
                    # Stop writing rows if a row contains 'NA' values for conflictPoint (for any conflict type) 
                    # Disabled until I get more logs to verify whether it's common for TTC situations to re-emerge in a single conflict. 
                    # This would probably be rendered moot by decreasing the 'extratime' parameter in SUMO settings
                    # NOTE: Consider how to disaggregate SSM conflict points from DRAC and PET conflict points within a conflict, apparently multiple can be in effect at a time
                    if (step[-3]) == 'NA':
                        break

                    # Write the transposed list of attr values to     
                    log_writer.writerow(step)

            # Write a separate file for each conflict with the extremal (most acute) value for each SSM type 
            with open(extremals_out_path, 'w', newline='') as out_csvfile:
                log_writer = writer(out_csvfile)

                # Write a header row
                header_row = [
                    'minTTC_time', 
                    'minTTC_position',
                    'minTTC_type',
                    'minTTC_value',
                    'maxDRAC_time',
                    'maxDRAC_position',
                    'maxDRAC_type',
                    'maxDRAC_value',
                    'PET_time',
                    'PET_position',
                    'PET_type',
                    'PET_value'
                ]

                # Write content for extremal values of each SSM type (TTC, DRAC, PET). This will be one wide row.
                log_writer.writerow(header_row)
                log_writer.writerow([
                    conflict.minTTC['time'], 
                    conflict.minTTC['position'], 
                    conflict.minTTC['type'], 
                    conflict.minTTC['value'],
                    conflict.maxDRAC['time'], 
                    conflict.maxDRAC['position'], 
                    conflict.maxDRAC['type'], 
                    conflict.maxDRAC['value'],
                    conflict.PET['time'], 
                    conflict.PET['position'], 
                    conflict.PET['type'], 
                    conflict.PET['value']
                    
                ])

            # Iterate conflict number
            i += 1
        print('Done converting SSM logs')

    except Exception as e:
        print(type(e))
        print(e)
        exit()


# Bootleg main() function; running this function should take place externally
# ssm_log_to_csv('C:\Apps\OpenCDA\opencda\scenario_testing\evaluations\ssm_logs\ssm_v1_archive.xml', 'C:\Apps\OpenCDA\opencda\scenario_testing\evaluations\ssm_logs')