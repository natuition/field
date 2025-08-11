from notification import NotificationClient
import utility
import time
from shared_class.robot_synthesis import RobotSynthesis

notification_client = NotificationClient(utility.get_current_time())
notification_client.set_treated_weed_types({'Plantain_great', 'Dandellion', 'Daisy', 'Plantain_narrowleaf', 'Porcelle'})
notification_client.set_field(
    [
        [46.157208550055564, -1.1349983142896134],
        [46.15737906845645, -1.135145715584571],
        [46.15780593305399, -1.1341165063153453],
        [46.15763541501675, -1.133969113564744]
    ],
    "Some field name")
notification_client.set_input_voltage(13.2)
notification_client.set_current_coordinate([46.157208550055564, -1.1349983142896134])
time.sleep(2)
notification_client.set_robot_state(RobotSynthesis.ANTI_THEFT)
time.sleep(3)
notification_client.set_robot_state(RobotSynthesis.ENABLED)
time.sleep(3)
notification_client.close()
