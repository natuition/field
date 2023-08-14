from config import config
import adapters
import time
import utility

def frange(start, stop=None, step=None):
    # if set start=0.0 and step = 1.0 if not specified
    start = float(start)
    if stop == None:
        stop = start + 0.0
        start = 0.0
    if step == None:
        step = 1.0

    count = 0
    while True:
        temp = float(start + count * step)
        if step > 0 and temp >= stop:
            break
        elif step < 0 and temp <= stop:
            break
        yield temp
        count += 1

def main():
    smoothie_vesc_addr = utility.get_smoothie_vesc_addresses()
    if config.SMOOTHIE_BACKEND == 1:
        smoothie_address = config.SMOOTHIE_HOST
    else:
        if "smoothie" in smoothie_vesc_addr:
            smoothie_address = smoothie_vesc_addr["smoothie"]
        else:
            msg = "Couldn't get smoothie's USB address!"
            print(msg)
            exit(1)

    with adapters.SmoothieAdapter(smoothie_address) as smoothie:
        for i in frange(0,2.5,0.1):
            t1 = time.time()
            smoothie.custom_move_to(A_F=1000, A=i)
            smoothie.wait_for_all_actions_done()
            t2 = time.time()
            print(f"Time for execute G0 A{i} F{1000} : {round((t2-t1)*1000,2)} ms.")
            #reset
            smoothie.custom_move_to(A_F=config.A_F_MAX, A=-i)
            smoothie.wait_for_all_actions_done()

if __name__ == "__main__":
    main()