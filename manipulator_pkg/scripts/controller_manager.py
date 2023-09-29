import controller_manager_msgs.srv
import rospy

class ControllerManager:
    def __init__(self):
        self.c_manager_l = rospy.ServiceProxy('/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
        self.c_manager_s = rospy.ServiceProxy('/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController)
        self.controllers = {}
        self.controllers["joint_config"] = "scaled_pos_traj_controller"
        self.controllers["compliant"] = "cartesian_compliance_controller"

    def switch(self, controller):
        if controller not in self.controllers:
            print('Controller is not known:', controller)
            return False
        c_list = self.c_manager_l(controller_manager_msgs.srv.ListControllersRequest())
        c_states = {}
        for c in c_list.controller:
            if c.name in self.controllers.values():
                c_states[c.name] = c.state
        if self.controllers[controller] not in c_states:
            print("Desired controller '{}' not known to controller manager".format(self.controllers[controller]))
            return False
        if c_states[self.controllers[controller]] == "running":
            print("Controller '{}' is already running.".format(controller))
            return True

        # Switch controllers
        req = controller_manager_msgs.srv.SwitchControllerRequest()
        req.start_controllers.append(self.controllers[controller])
        running_c = list(c_states.keys())[list(c_states.values()).index("running")]
        req.stop_controllers.append(running_c)
        req.strictness = 2  # Strict
        req.timeout = 1.    # Seconds

        if self.c_manager_s(req):
            print("Switched to", controller)
            return True
        else:
            print("Failed to switch to", controller)
            return False