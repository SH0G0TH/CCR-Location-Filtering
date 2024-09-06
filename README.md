# CCR-Location-Filtering
This repository is a Seattle Aquarium project, aiming to create a Kalman filter which ingests DVL and USBL data to generate a more accurate prediction of the location of the ROV.

USE:

When in the CRR-Location-Filtering directory, run main.py.

To specify a different GPS log to read from, use --GPS. 
ex.

main.py --GPS otherDive.csv
The default is set to use the provided log from the one pager.

To specify output csv file for filtered results, use --log.
ex.

main.py --log output
The default is set to display both the measured GPS position and the filter output in a matplotlib window.
