/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature Vision4__GOAL_RED = vex::vision::signature (1, 8439, 9057, 8748, -479, 449, -15, 6.7, 0);
vex::vision::signature Vision4__GOAL_BLUE = vex::vision::signature (2, -2027, -1361, -1694, 9471, 11057, 10264, 4.6, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision4 = vex::vision (vex::PORT5, 50, Vision4__GOAL_RED, Vision4__GOAL_BLUE, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/