import interfaces as iface

#
# Exceptions
#

class SimulationException(Exception):
    pass

#
# Actual class implementations
#

class Simulation(object):
    def __init__(self, world):
        self._world = world
        self._pre_callbacks = {}
        self._post_callbacks = {}

    def __del__(self):
        pass

    def step(self, time_step=None):
        for callback in self._pre_callbacks.values():
            callback.call(self._world)
        self._world.step(time_step)
        for callback in self._post_callbacks.values():
            callback.call(self._world)

    def steps(self, num_steps=1, time_step=None):
        for _ in range(0, num_steps):
            self.step(time_step)

    def run(self, duration, time_step=None):
        if time_step is None:
            num_steps = duration / self.get_time_step()
        else:
            num_steps = duration / time_step
        self.steps(int(num_steps), time_step)

    def get_time_step(self):
        return self._world.get_time_step()

    def set_time_step(self, time_step):
        self._world.set_time_step(time_step)

    def register_pre_callback(self, name, callback):
        self._register_callback(name, callback, "_pre_callbacks",
                                 "A pre-callback with name '" + name + "' already exists.")

    def register_post_callback(self, name, callback):
        self._register_callback(name, callback, "_post_callbacks",
                                 "A post-callback with name '" + name + "' already exists.")

    def _register_callback(self, callback_name, callback, attr_name, error_string):
        if not iface.CallbackInterface in callback.__class__.__bases__:
            raise TypeError("Given callback with name '" + callback_name + "' has no base class CallbackInterface.")
        if callback_name in getattr(self, attr_name):
            raise SimulationException(error_string)
        getattr(self, attr_name)[callback_name] = callback