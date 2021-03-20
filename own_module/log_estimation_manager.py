import logging
import time
import cflib.crtp
from cflib.crazyflie.log import LogConfig


class LogEstimationManager:

    # We create a configuration and add all the variables to log and their logging period
    def __init__(self):
        self._logconf = LogConfig(name='Estimation', period_in_ms=500)
        self._position_estimate = [0, 0, 0]
        self._attitude_estimate = [0, 0, 0]

    def __enter__(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        self.config_logging()
        # self.simple_log_async()
        # self.reset_estimator()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("disconnecting")
        self._logconf.stop()

    def config_logging(self):
        self._logconf.add_variable('stateEstimate.x', 'float')
        self._logconf.add_variable('stateEstimate.y', 'float')
        self._logconf.add_variable('stateEstimate.z', 'float')
        # self._logconf.add_variable('stabilizer.roll', 'float')
        # self._logconf.add_variable('stabilizer.pitch', 'float')
        # self._logconf.add_variable('stabilizer.yaw', 'float')
        logging.info("variables added")

    # Callback function to print the contents of logs
    def callback(self, timestamp, data, _logconf):
        print(data)
        self._position_estimate[0] = data['stateEstimate.x']
        self._position_estimate[1] = data['stateEstimate.y']
        self._position_estimate[2] = data['stateEstimate.z']
        self._attitude_estimate[0] = data['stabilizer.roll']
        self._attitude_estimate[1] = data['stabilizer.pitch']
        self._attitude_estimate[2] = data['stabilizer.yaw']

    # We add to the Drone framework the previously defined logging configuration
    def simple_log_async(self, cf):
        cf.log.add_config(self._logconf)    # check if variables are allowed (i.e. are in TOC)
        if self._logconf.valid:
            self._logconf.data_received_cb.add_callback(self.callback)
            self._logconf.start()
        else:
            logging.error("Could not add logconfig since some variables are not in TOC")

    def reset_estimator(self, cf):
        cf.param.set_value('stabilizer.estimator', 2)
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

    def get_estimation(self):
        return self._position_estimate, self._attitude_estimate


'''
Limitations to be taken into account:
1)  Each packet is limited to 32bytes, which means that both the data that is logged 
    and the packet that is sent cannot be larger than this. 
2)  The minimum period for a log configuration is multiples of 10ms.
'''
