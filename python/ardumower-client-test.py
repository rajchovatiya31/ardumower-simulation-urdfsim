import os
import sys
import keyboard
import shutil
import time
import numpy as np
import pandas as pd
import json
import jsonpickle
import ArdumowerClient

import setup_path
import airsim
import ClientUtil

def main():

    client = ArdumowerClient.ArdumowerClient(0.36, 0.25, 4)
    output_folder_name = 'ardumower_out'
    print('Driving forward')
    client.diff_drive(0.2, 0)
    time.sleep(5)
    print('Stopping')
    client.diff_drive(0, 0)
    time.sleep(5)
    print('Driving in a circle')
    client.diff_drive(0.2, 0.2)
    time.sleep(5)
    print('Stopping')
    client.drive(0, 0)
    
    print('Graceful termination.')

main()
