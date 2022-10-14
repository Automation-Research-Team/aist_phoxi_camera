uint8 gACT  # echo of rACT
uint8 gMOD  # echo of rMOD
uint8 gGTO  # echo of rGTO
uint8 gSTA  # status (0: inactive, 3: active)
uint8 gOBJ  # object detection (0: unknown object detection, 1: object detected with minimum vacuum value reached, 2: object detected with maximum vacuum value reached, 3: no object detected)
uint8 gVAS  # vacuum actuator status (0: standby, 1: gripping, 2: passive releasing, 3: active releasing)
uint8 gFLT  # fault (0: ok, other: see manual for errors)
uint8 gPR   # echo of rPR
uint8 gPO   # actual position (Pdiff = gPO - 100)
