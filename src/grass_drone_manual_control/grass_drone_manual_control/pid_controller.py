class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.prev_error = 0
        self.integral = 0
        self.prev_measurement = 0


    def update(self, measurement):
        error = self.setpoint - measurement
        
        self.integral += error  # Acumulador integral 
        self.integral = max(min(self.integral, 10), -10) # Para no acumular demasiado error
 
        derivative = measurement - self.prev_measurement  # En vez de sobre el error, sobre la distancia/velocidad
        self.prev_measurement = measurement

        # Calcular la salida PID
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output
    

    def update_from_error(self, error):
        # PID exclusivo para YAW.
        self.integral += error
        self.integral = max(min(self.integral, 10), -10)
        derivative = error - self.prev_error
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


    def set_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint
        self.integral = 0  # Reinicia la acción integral para evitar acumulaciones no deseadas
        self.prev_error = 0  # Evita grandes saltos en la derivada


    def set_kp(self,kp):
        self.kp = kp