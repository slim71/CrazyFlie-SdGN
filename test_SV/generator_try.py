import time
import threading
import datetime

count = 0


def repeat_fun(period, func, *args):
    def time_tick():
        t = time.time()
        while True:
            t += period
            yield max(t - time.time(), 0)

    tick = time_tick()
    while True:
        time.sleep(next(tick))
        func(*args)


def attempt(vector, string):
    global count
    print("the vector is ", str(vector))
    count += 1
    # time.sleep(1)
    print("the string is ", string)
    print("count = ", count)
    print(str(datetime.datetime.now()))


with open("HL_withConversion_WRONG.py", 'r') as ff:
    repeat_thread = threading.Thread(target=repeat_fun,
                                     args=(0.1,
                                           attempt,
                                           (1, 2, 3), "ciao"
                                           )
                                     )
    repeat_thread.start()
    while 1:
        print("this is the main while")
        time.sleep(5)
        print("some time has passed...")
