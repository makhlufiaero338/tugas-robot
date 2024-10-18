# e-puck_open_loop.py
from controller import Robot

# Inisialisasi robot
robot = Robot()

# Dapatkan timestep dunia Webots
timestep = int(robot.getBasicTimeStep())

# Set kecepatan roda (konstan)
left_motor_speed = 6.28  # Kecepatan maksimum motor kiri
right_motor_speed = 6.28  # Kecepatan maksimum motor kanan

# Dapatkan referensi ke motor kiri dan kanan
left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")

# Set mode motor ke Velocity untuk kontrol kecepatan
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set kecepatan awal motor untuk bergerak maju
left_motor.setVelocity(left_motor_speed)
right_motor.setVelocity(right_motor_speed)

# Loop simulasi tanpa henti
while robot.step(timestep) != -1:
    # Robot akan terus bergerak maju
    pass  # Tidak ada logika lain karena ini open-loop control

# Setelah simulasi selesai (tidak akan terjadi dalam skenario ini)
left_motor.setVelocity(0)
right_motor.setVelocity(0)
