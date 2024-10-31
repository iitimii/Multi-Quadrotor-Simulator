class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, dt: float = 0.004, i_max: float = 400):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.i_max = i_max
        
        self.integral = 0
        self.prev_error = 0

    def calculate(self, setpoint: float, state: float) -> float:
        error = setpoint - state
        proportional = self.kp * error

        if self.ki == 0.0 and self.kd == 0.0:
            return proportional

        # Update integral with trapezoidal integration and constrain it
        self.integral += self.ki * (error + self.prev_error) * self.dt * 0.5
        self.integral = max(min(self.integral, self.i_max), -self.i_max)

        # Calculate the derivative component
        derivative = self.kd * (error - self.prev_error) / self.dt

        self.prev_error = error

        return proportional + self.integral + derivative

    def reset(self):
        self.integral = 0
        self.prev_error = 0
