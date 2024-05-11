
from donkeycar.parts.controller import Joystick, JoystickController


class pocket(Joystick):
    #An interface to a physical joystick available at /dev/input/js0
    def __init__(self, *args, **kwargs):
        super(pocket, self).__init__(*args, **kwargs)


        self.button_names = {
            0x101 : 'right',
        }


        self.axis_names = {
            0x0 : 'steer',
            0x4 : 'throttle',
        }



class MyJoystickController(JoystickController):
    #A Controller object that maps inputs to actions
    def __init__(self, *args, **kwargs):
        super(MyJoystickController, self).__init__(*args, **kwargs)


    def init_js(self):
        #attempt to init joystick
        try:
            self.js = pocket(self.dev_fn)
            self.js.init()
        except FileNotFoundError:
            print(self.dev_fn, "not found.")
            self.js = None
        return self.js is not None


    def init_trigger_maps(self):
        #init set of mapping from buttons to function calls

        self.button_down_trigger_map = {
        }


        self.axis_trigger_map = {
            'steer' : self.set_steering,
            'throttle' : self.set_throttle,
        }

