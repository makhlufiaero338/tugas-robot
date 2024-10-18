# e-puck_circle_motion.py
from controller import Robot

# Inisialisasi robot
robot = Robot()

# Dapatkan timestep simulasi dari Webots
timestep = int(robot.getBasicTimeStep())

# Dapatkan referensi ke motor kiri dan kanan
left_motor = robot.getMotor("left wheel motor")
right_motor = robot.getMotor("right wheel motor")

# Set posisi motor ke 'infinity' agar dapat mengendalikan kecepatan (velocity control)
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Atur kecepatan untuk gerakan melingkar
left_motor_speed = 3.14  # Kecepatan lebih rendah untuk roda kiri
right_motor_speed = 6.28  # Kecepatan lebih tinggi untuk roda kanan

# Set kecepatan roda
left_motor.setVelocity(left_motor_speed)
right_motor.setVelocity(right_motor_speed)

# Loop simulasi, robot akan terus bergerak melingkar
while robot.step(timestep) != -1:
    # Robot akan bergerak melingkar tanpa kontrol feedback
    pass  # Tidak ada logika tambahan karena ini adalah kontrol open-loop

# Setelah simulasi selesai (tidak akan berhenti secara otomatis)
left_motor.setVelocity(0)
right_motor.setVelocity(0)
