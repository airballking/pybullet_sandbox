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
        self.__world = world
        self.__pre_callbacks = {}
        self.__post_callbacks = {}

    def __del__(self):
        pass

    def step(self):
        for callback in self.__pre_callbacks.values():
            callback(self.__world)
        self.__world.step()
        for callback in self.__post_callbacks.values():
            callback(self.__world)

    def get_time_step(self):
        return self.__world.get_time_step()

    def register_pre_callback(self, name, callback):
        self.__register_callback(name, callback, "__pre_callbacks",
                                 "A pre-callback with name '" + name + "' already exists.", )

    def register_post_callback(self, name, callback):
        self.__register_callback(name, callback, "__post_callbacks",
                                 "A post-callback with name '" + name + "' already exists.", )

    def __register_callback(self, callback_name, callback, attr_name, error_string):
        if callback_name in getattr(self, attr_name):
            raise SimulationException(error_string)
        getattr(self, attr_name)[callback_name] = callback