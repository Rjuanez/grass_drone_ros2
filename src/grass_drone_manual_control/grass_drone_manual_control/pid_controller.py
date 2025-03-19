class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.prev_error = 0
        self.integral = 0

    def update(self, measurement):
        error = self.setpoint - measurement
        self.integral += error  # Acumulador integral
        derivative = error - self.prev_error  # Diferencia de errores sucesivos
        self.prev_error = error

        # Calcular la salida PID
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output

    def set_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint
        self.integral = 0  # Reinicia la acción integral para evitar acumulaciones no deseadas
        self.prev_error = 0  # Evita grandes saltos en la derivada

    def set_kp(self,kp):
        self.kp = kp

