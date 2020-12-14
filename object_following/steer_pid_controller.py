steer_pub = rospy.Publisher("/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1)
steer_cmd = PositionWithSpeed()

class steer_pid_controller:
    def __init__(
        self,
        p=0.0,
        i=0.0,
        d=0.0,
        wg=20.0,
        avl=2.5,
        max_steering_angle=0.8,
        max_steering_angle=-0.8,
        desired_x
    ):
        self.kp = p
        self.ki = i
        self.kd = d
        self.windup_guard = wg
        self.prev_error = 0.0
        self.prev_time = time.time()

        self.pterm = 0.0
        self.iterm = 0.0
        self.dterm = 0.0

        self.max_steering_angle = max_steering_angle
        self.min_steering_angle = min_steering_angle

        self.angular_velocity_limit = avl
        self.desired_x = desired_x

        global steer_pub
        global steer_cmd

    def steer_control(self, curr_x):
        global steer_cmd
        if steer_flag:
            current_time = time.time()
            delta_time = current_time - self.prev_time

            current_error = self.desired_x - curr_x
            delta_error = current_error - self.prev_error
            error_dot = delta_error / delta_time

            self.pterm = current_error
            self.dterm = error_dot
            self.iterm += current_error * delta_time

            if self.iterm > self.windup_guard:
                self.iterm = self.windup_guard
            if self.iterm < -self.windup_guard:
                self.iterm = -self.windup_guard

            self.prev_time = current_time
            self.prev_error = current_error

            output = (
                self.kp * self.pterm
                + self.kd * self.dterm
                + self.ki * self.iterm
            )

            output = max(min(output, self.max_steering_angle), self.min_steering_angle)
            output = np.pi * output

            steer_cmd.angular_position = output
            steer_cmd.angular_velocity_limit = (np.pi / 2) / 2
            steer_pub.publish(steer_cmd)

        else:
            accel_cmd.f64_cmd = 0.0
            accel_pub.publish(accel_cmd)