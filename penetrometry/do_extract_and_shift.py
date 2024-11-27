import time
import sys
sys.path.append('../')

from config import config
from adapters import SmoothieAdapter
from adapters import VescAdapterV4
from penetrometry.PenetrometryAnalyse import PenetrometryAnalyse
import utility

# This script will do the number of extraction that you specified.
# You can also specified if the actuator has to shift lightly between each extraction, the shift will make a diamond.
# You can also specified to do  penetrometry analyse which will save data from vesc (see 'penetrometry' in config.py)

NUMBER_OF_EXTRACTION = 20 # The number of extraction you want to perform
EXTRACTION_Z_FOR_THIS_TEST = 10 # The time of the descending phase of an extraction, custom for this test
Z_F_EXTRACTION_DOWN_FOR_THIS_TEST = 1950 # The speed of the descending phase of an extraction, custom for this test
SHIFTING = False # True if you want to shift between each extraction
SAVE_IN_FILE = False # True if you want to save datas from VESC in a file

# Check if the smoothie response is valid or not
def check_res_smoothie(aRes: str):
    if(aRes != SmoothieAdapter.RESPONSE_OK):
        exit(f"Response de la smoothie non conforme : {aRes}")

def main():
    ports = utility.get_smoothie_vesc_addresses()
    if SAVE_IN_FILE:
        if("vesc" not in ports):
            exit("Couldn't get vesc's USB address!")
        myVesc = VescAdapterV4(ports["vesc"], config.VESC_BAUDRATE, config.VESC_ALIVE_FREQ, config.VESC_CHECK_FREQ, config.VESC_STOPPER_CHECK_FREQ)
        myPenetrometryAnalyse = PenetrometryAnalyse(myVesc, SAVE_IN_FILE)
    mySmoothie = SmoothieAdapter(ports["smoothie"], True)
    extraction_count = 0
    input("Le robot va placer la tête d'extraction au centre des axes X et Y. Entrée pour continuer.")
    start_time = None
    try:
        check_res_smoothie(mySmoothie.custom_move_to(X_F=config.X_F_MAX, Y_F=config.Y_F_MAX, X=config.X_MAX/2, Y=config.Y_MAX/2)) # Place to the center
        input("Vous pouvez placer votre matière en dessous, l'actionneur au centre du récipient. Prendre un récipient d'au moins 5cm de rayon. Entrée pour continuer.")
        start_time = time.time()

        if SHIFTING:
            check_res_smoothie(mySmoothie.custom_move_to(X_F=config.X_F_MAX, X=(config.X_MAX/2)+20)) # Shift from the center
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())

        while(extraction_count < NUMBER_OF_EXTRACTION):
            check_res_smoothie(mySmoothie.custom_move_for(Z_F=Z_F_EXTRACTION_DOWN_FOR_THIS_TEST, Z=EXTRACTION_Z_FOR_THIS_TEST))
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            check_res_smoothie(mySmoothie.ext_cork_up())
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            extraction_count = extraction_count + 1

            if SHIFTING:
                check_res_smoothie(mySmoothie.custom_move_to(X_F=config.X_F_MAX, X=(config.X_MAX/2), Y_F=config.Y_F_MAX, Y=(config.Y_MAX/2)+20))
                check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            check_res_smoothie(mySmoothie.custom_move_for(Z_F=Z_F_EXTRACTION_DOWN_FOR_THIS_TEST, Z=EXTRACTION_Z_FOR_THIS_TEST))
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            check_res_smoothie(mySmoothie.ext_cork_up())
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            extraction_count = extraction_count + 1

            if SHIFTING:
                check_res_smoothie(mySmoothie.custom_move_to(X_F=config.X_F_MAX, X=(config.X_MAX/2)-20, Y_F=config.Y_F_MAX, Y=(config.Y_MAX/2)))
                check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            check_res_smoothie(mySmoothie.custom_move_for(Z_F=Z_F_EXTRACTION_DOWN_FOR_THIS_TEST, Z=EXTRACTION_Z_FOR_THIS_TEST))
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            check_res_smoothie(mySmoothie.ext_cork_up())
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            extraction_count = extraction_count + 1

            if SHIFTING:
                check_res_smoothie(mySmoothie.custom_move_to(X_F=config.X_F_MAX, X=(config.X_MAX/2), Y_F=config.Y_F_MAX, Y=(config.Y_MAX/2)-20))
                check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            check_res_smoothie(mySmoothie.custom_move_for(Z_F=Z_F_EXTRACTION_DOWN_FOR_THIS_TEST, Z=EXTRACTION_Z_FOR_THIS_TEST))
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            check_res_smoothie(mySmoothie.ext_cork_up())
            check_res_smoothie(mySmoothie.wait_for_all_actions_done())
            extraction_count = extraction_count + 1

            if SHIFTING:
                check_res_smoothie(mySmoothie.custom_move_to(X_F=config.X_F_MAX, X=(config.X_MAX/2)+20, Y_F=config.Y_F_MAX, Y=(config.Y_MAX/2)))
                check_res_smoothie(mySmoothie.wait_for_all_actions_done())

        if SHIFTING:
            print(f"Le programme s'est terminé avec succès, l'actionneur a fait {extraction_count} extractions en se décalant à chaque fois.")
        else:
            print(f"Le programme s'est terminé avec succès, l'actionneur a fait {extraction_count} extractions sans se décaler à chaque fois.")

    except:
        if SHIFTING:
            print(f"Le programme s'est intérompu, l'actionneur a fait {extraction_count} extractions en se décalant à chaque fois.")
        else:
            print(f"Le programme s'est intérompu, l'actionneur a fait {extraction_count} extractions sans se décaler à chaque fois.")
    finally:
        mySmoothie.disconnect()
        if start_time:
            elapsed_time = time.time() - start_time
            print(f"Temps total pour effectuer {extraction_count} extractions : {elapsed_time:.2f} secondes.")
       

if __name__ == '__main__':
    main()