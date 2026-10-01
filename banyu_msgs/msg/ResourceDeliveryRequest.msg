# RobotX 2026 resource delivery request (handbook 3.3.3 Advanced tier, 3.4.12): the colour the
# pipeline light flashed after the repair names the platform circle a UAV must deliver a tin to.
# The surfaced UUV publishes it on /banyu/task2/resource_request; the OCS relays it to
# RoboCommand (banyu_robocommand report_bridge) and the UAV takes its tasking from the same topic.
#
# Values mirror RoboCommand's RxTask and Color enums, so the relay copies them as they are.

std_msgs/Header header

uint8 TASK_INFRA_SURVEY_REPAIR=3    # Task 2
uint8 TASK_COORDINATED_LOGISTICS=4  # Task 3
uint8 task

uint8 COLOR_RED=1
uint8 COLOR_GREEN=2
uint8 COLOR_BLUE=3
uint8 COLOR_ANY=4                   # Advanced tier: any tin will do
uint8 resource_color
uint8 delivery_circle_color
