# e-puck_proximity_stop.py
from controller import Robot

# Inisialisasi robot
robot = Robot()

# Dapatkan timestep dunia Webots
timestep = int(robot.getBasicTimeStep())

# Inisialisasi sensor proximity
proximity_sensors = []
sensor_names = [
    "ps0", "ps1", "ps2", "ps3", 
    "ps4", "ps5", "ps6", "ps7"
]

# Aktifkan semua sensor proximity
for name in sensor_names:
    sensor = robot.getDistanceSensor(name)
    sensor.enable(timestep)
    proximity_sensors.append(sensor)

# Dapatkan referensi ke motor kiri dan kanan
left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")

# Set motor untuk mode velocity
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set kecepatan awal motor untuk bergerak maju
left_motor_speed = 6.28
right_motor_speed = 6.28
left_motor.setVelocity(left_motor_speed)
right_motor.setVelocity(right_motor_speed)

# Threshold untuk mendeteksi objek di depan (sensor depan: ps0, ps7)
distance_threshold = 80.0  # Nilai threshold untuk mendeteksi jarak objek

# Loop simulasi
while robot.step(timestep) != -1:
    # Baca nilai sensor proximity depan
    front_sensor_left = proximity_sensors[0].getValue()  # Sensor ps0
    front_sensor_right = proximity_sensors[7].getValue()  # Sensor ps7

    # Jika ada objek di depan (sensor mendeteksi di atas threshold)
    if front_sensor_left > distance_threshold or front_sensor_right > distance_threshold:
        # Hentikan robot
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
    else:
        # Jika tidak ada objek, lanjutkan bergerak maju
        left_motor.setVelocity(left_motor_speed)
        right_motor.setVelocity(right_motor_speed)
