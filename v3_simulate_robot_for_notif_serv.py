from notification import NotificationClient, SyntheseRobot
import utility
import time

notification = NotificationClient(utility.get_current_time())
notification.set_treated_plant(['Plantain_great', 'Dandellion', 'Daisy', 'Plantain_narrowleaf', 'Porcelle'])
notification.set_field([[46.157208550055564, -1.1349983142896134], [46.15737906845645, -1.135145715584571], [46.15780593305399, -1.1341165063153453], [46.15763541501675, -1.133969113564744]])
notification.set_input_voltage(13.2)
notification.set_current_coordinate([46.157208550055564, -1.1349983142896134])
time.sleep(2)
notification.setStatus(SyntheseRobot.ANTI_THEFT)
time.sleep(3)
notification.setStatus(SyntheseRobot.OP)
time.sleep(3)
notification.stop()
