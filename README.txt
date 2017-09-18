
1. Save: pedometer.c and pedometer.h in a folder
2. Run: gcc -Wall -o EXEC_FNAME pedometer.c where EXEC_FNAME is the desired filename for the executible
There should not be any warning or error. The folder should have a new file EXEC_FNAME created.
3. Usage: EXEC_FNAME input_file.csv output_file_csv
4. Create and save appropriate input and outfiles in the same working folder.

Here is an example on Windows PC using gcc and provided example data files:

C:\[PATH]>dir
02/09/2016  09:39 AM           233,099 SensData_run_walk_OUT.csv
02/04/2016  01:37 PM           161,557 SensData_run_walk_stripped.csv
02/09/2016  09:40 AM           347,254 SensData_walk_hop_walk_run_OUT.csv
02/04/2016  01:37 PM           240,752 SensData_walk_hop_walk_run_stripped.csv
02/09/2016  09:39 AM           195,691 SensData_walk_run_OUT.csv
02/04/2016  01:37 PM           135,914 SensData_walk_run_stripped.csv
02/09/2016  09:38 AM            15,576 pedometer.c
02/09/2016  02:53 AM             1,511 pedometer.h

C:\[PATH]>gcc -Wall -o StepDetNCount pedometer.c

C:\[PATH]>dir
02/09/2016  09:39 AM           233,099 SensData_run_walk_OUT.csv
02/04/2016  01:37 PM           161,557 SensData_run_walk_stripped.csv
02/09/2016  09:40 AM           347,254 SensData_walk_hop_walk_run_OUT.csv
02/04/2016  01:37 PM           240,752 SensData_walk_hop_walk_run_stripped.csv
02/09/2016  09:39 AM           195,691 SensData_walk_run_OUT.csv
02/04/2016  01:37 PM           135,914 SensData_walk_run_stripped.csv
02/09/2016  09:38 AM            15,576 pedometer.c
02/09/2016  02:53 AM             1,511 pedometer.h
02/09/2016  09:39 AM            74,302 StepDetNCount.exe

C:\[PATH]>StepDetNCount.exe SensData_run_walk_stripped.csv SensData_run_walk_OUT.csv
Total motion duration is 18.855299 sec, which contains approximatly:
 28 Total number of steps including
 |---> 13 steps of WALKING,
 |---> 14 steps of RUNNING, and
 |---> 1 steps of HOPPING.
Done.

C:\[PATH]>StepDetNCount.exe SensData_walk_run_stripped.csv SensData_walk_run_OUT.csv
Total motion duration is 15.932355 sec, which contains approximatly:
 27 Total number of steps including
 |---> 11 steps of WALKING,
 |---> 16 steps of RUNNING, and
 |---> 0 steps of HOPPING.
Done.

C:\[PATH]>StepDetNCount.exe SensData_walk_hop_walk_run_stripped.csv SensData_walk_hop_walk_run_OUT.csv
Total motion duration is 27.960651 sec, which contains approximatly:
 40 Total number of steps including
 |---> 16 steps of WALKING,
 |---> 16 steps of RUNNING, and
 |---> 8 steps of HOPPING.
Done.

