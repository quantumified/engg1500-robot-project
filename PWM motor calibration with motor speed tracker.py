# TODO: Import necessary modules
# TODO: Initalise motors

uart1 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

pwm = 100

def count():                          # function for retrieving combined total counts
    msg = uart1.read(150)
    while msg is None:
        msg = uart1.read(150)
    msg = msg.decode('utr-8')         # Turn bytes object into a Python string
    Total_L = int(msg.split(' ')[0])  # Left Encoder count
    Total_R = int(msg.split(' ')[1])  # Right Encoder count
    Total = Total_L + Total_R
    return Total
def Lspeed():                         # function for retrieving left motor speed
    msg = uart1.read(150)
    while msg is None:
        msg = uart1.read(150)
    msg = msg.decode('utr-8')         # Turn bytes object into a Python string
    Rate_L = int(msg.split(' ')[2])   #Left Encoder speed
    return Rate_L
# TODO write a function for right motor speed

while True:
    # TODO set PWM of left and right motor

    Icount = count()                  # Initial encoder count
    Pcount = Icount
    while Icount >= (Pcount - 100):   # Present encoder count
        Pcount = count()              # Calling left motor speed

    speed_left = Lspeed()
    #TODO call right speed function

    print("{:3d}, {:3d}, {:3d}".format(pwm, speed_left, speed_right))
    pwm -= 5                          # Reducing PWM