# main.py - this file gets run directly.
import sched
import time
# scheduler = sched.scheduler(time.time, time.sleep)
#
# print("before definition")
#
#
# def do_something():
#     print("do_something is being executed.")
#     scheduler.enter(0.01, 5, do_something)
#
#
# print("post definition")
# scheduler.enter(0.01, 5, do_something)
#
# # Do something, so that the program does not terminate.
# do_something()
# scheduler.run()

from threading import Timer

def x():
    print("Hello")

Timer(10, x, ()).start()
print("World!")
