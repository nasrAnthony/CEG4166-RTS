import time, sys, tty, termios

#Function to capture keyboard input
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#while True:
#    #Capture keyboard input
#    char = getch()
#    if char == "w":
#        print("Char W pressed")
#
#    #Exits Program
#    elif char == "s":
#        exit()