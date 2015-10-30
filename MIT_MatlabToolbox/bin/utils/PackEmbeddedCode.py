#! /usr/bin/python

"""File: PackEmbeddedCode.c
/*
* AUTHOR Fabian Riether
* CREATE DATE 2015/09/04
* PURPOSE This program takes the SIMULINK auto-generated c-files for the DroneRS_Compensator (in trunk/matlab/Simulation/DronesRS_Compensator_ert_shrlib_rtw and merges them with the existing c-code for control
* optical flow and image processing (in trunk/embcode)
* SPECIAL NOTES
* ===============================
* Change History
* 2015/09/04 created
* ==================================
*/
"""
from __future__ import print_function
import os
import shutil

print("C-files are being packed...")

trunk = os.path.join(os.path.dirname(__file__), '../../trunk')

# 1 Copy all files from trunk/matlab/Simulation/DronesRS_Compensator_ert_shrlib_rtw to trunk/embcode
emb_dir = os.path.join(trunk, "embcode")
rtw_dir = os.path.join(trunk, "matlab/Simulation/DroneRS_Compensator_ert_shrlib_rtw")
rtw_main_fname = "ert_main.c"
mainFile = os.path.join(rtw_dir, rtw_main_fname)

if not os.path.exists(mainFile):
    raise SystemExit('ERROR auto-generated c-files not found in {}!'.format(rtw_dir))

try:
    for fname in os.listdir(rtw_dir):
        shutil.copy(
            os.path.join(rtw_dir, fname),
            os.path.join(emb_dir, fname)
        )
except OSError:
    raise SystemExit("ERROR Copying SIMULINK auto-generated files! Exiting. \n")


#2 Replace parameter set in rsedu_control.c with new parameters from ert_main.c
# create copy of rsedu_control.c
control_fname = os.path.join(emb_dir, "rsedu_control.c")
control_copy_fname = os.path.join(emb_dir, "rsedu_control_copy.c")
shutil.copy(control_fname, control_copy_fname)


with open(control_fname, 'w') as newf, open(control_copy_fname) as oldf:
    old_lines = iter(oldf)

    for line in old_lines:
        if "static RT_MODEL_" in line:
            break
        newf.write(line)

    # Open ert_main.c for read, find first line of parameter block, start appending parameters to rsedu_control.
    emb_main_fname = os.path.join(emb_dir, rtw_main_fname)
    with open(emb_main_fname) as fp_ertmain_inemb:

        in_param_section = False
        for line in fp_ertmain_inemb:
            if "static RT_MODEL_" in line:
                in_param_section = True

            if in_param_section:
                newf.write(line)

            if "};                                     /* Modifiable parameters */\n" in line:
                break

    # Now read copy of rseducontrol till end of parameter section, then start appending rest of that code to rsedu_control
    after_params = False
    for line in old_lines:
        if after_params:
            newf.write(line)

        if "};                                     /* Modifiable parameters */\n" in line:
            after_params = True

os.remove(emb_main_fname);
os.remove(control_copy_fname);

print("C-files packed and ready to be built!")