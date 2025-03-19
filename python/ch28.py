# chapter 28 in python

# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

import serial
import matplotlib.pyplot as plt
from statistics import mean
from genref import *

ser = serial.Serial('/dev/ttyUSB0', 230400)
print('Opening port: ')
print(ser.name)

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options; this list will grow
    print('\ta: Read current sensor (ADC counts) \tb: Read current sensor (mA)\n')
    print('\tc: Read encoder (counts) \t\td: Read encoder (degs)\n')
    print('\te: Reset encoder \t\t\tf: Set PWM (-100 to 100)\n')
    print('\tg: Set current gains \t\t\th: Get current gains\n')
    print('\ti: Set position gains \t\t\tj: Get position gains\n')
    print('\tk: Test current control \t\tl: Go to angle (deg)\n')
    print('\tm: Load step trajectory \t\tn: Load cubic trajectory\n')
    print('\to: Execute trajectory \t\t\tp: Unpower the motor\n')
    print('\tq: Quit client \t\t\t\tr: Get mode\n')
    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'

    # send the command to the PIC32
    # .encode() turns the string into a char array
    ser.write(selection_endline.encode())

    # take the appropriate action
    # there is no switch() in python, using if elif instead
    if (selection == 'd'):
        # Read encoder (deg)
        n_str = ser.read_until(b'\n')  # get the incremented number back
        n_float = float(n_str)  # turn it into an int
        print('The motor angle is ' + str(n_float) +
              ' degrees\n')  # print it to the screen
    elif (selection == 'a'):
        # Read current sensor (ADC counts)
        pass
    elif (selection == 'b'):
        # Read current sensor (mA)
        pass
    elif (selection == 'c'):
        # Read encoder (counts)
        n_str = ser.read_until(b'\n')  # get the incremented number back
        n_int = int(n_str)  # turn it into an int
        print('The motor angle is ' + str(n_int) +
              ' counts\n')  # print it to the screen
    elif (selection == 'e'):
        # Reset encoder
        n_str = ser.read_until(b'\n')  # get the incremented number back
        n_str = str(n_str)  # turn it into an int
        print(n_str)
    elif (selection == 'f'):
        # Set PWM (-100 to 100)
        # get the number to send
        n_str = input('What PWM value would you like [-100 to 100]? ')
        n_int = int(n_str)  # turn it into an int

        ser.write((str(n_int)+'\n').encode())  # send the number
        n_str = ser.read_until(b'\n')
        print(str(n_str) + '\n')
    elif (selection == 'g'):
        # Set current gains
        n_str = input('Enter your desired Kp current gain: ')
        n_float = float(n_str)  # turn it into an int
        ser.write((str(n_float)+'\n').encode())

        n_str = input('Enter your desired Ki current gain: ')
        n_float = float(n_str)  # turn it into an int
        ser.write((str(n_float)+'\n').encode())

        n_str = ser.read_until(b'\n')  # get the incremented number back
        print(str(n_str) + '\n')  # print it to the screen
    elif (selection == 'h'):
        # Get current gains
        n_str = ser.read_until(b'\n')
        n_str = str(n_str)  # turn it into an int
        print(n_str)  # print it to the screen
    elif (selection == 'i'):
        # Set position gains
        n_str = input('Enter your desired Kp position gain: ')
        n_float = float(n_str)  # turn it into an int
        ser.write((str(n_float)+'\n').encode())

        n_str = input('Enter your desired Ki position gain: ')
        n_float = float(n_str)  # turn it into an int
        ser.write((str(n_float)+'\n').encode())

        n_str = input('Enter your desired Kd position gain: ')
        n_float = float(n_str)  # turn it into an int
        ser.write((str(n_float)+'\n').encode())

        n_str = ser.read_until(b'\n')
        print(str(n_str) + '\n')  # print it to the screen
    elif (selection == 'j'):
        # Get position gains
        n_str = ser.read_until(b'\n')
        n_str = str(n_str)  # turn it into an int
        print(n_str)  # print it to the screen
    elif (selection == 'k'):
        # Test current control
        # get the number of data points to receive
        n_str = ser.read_until(b'\n')
        n_int = int(n_str)  # turn it into an int
        print('Data length = ' + str(n_int))
        ref = []
        data = []
        data_received = 0
        while data_received < n_int:
            # get the data as a string, ints seperated by spaces
            dat_str = ser.read_until(b'\n')
            dat_f = list(map(float, dat_str.split()))  # now the data is a list
            ref.append(dat_f[0])
            data.append(dat_f[1])
            data_received = data_received + 1
        meanzip = zip(ref, data)
        meanlist = []
        for i, j in meanzip:
            meanlist.append(abs(i-j))
        score = mean(meanlist)
        t = range(len(ref))  # index array
        plt.plot(t, ref, 'r*-', t, data, 'b*-')
        plt.title('Score = ' + str(score))
        plt.ylabel('Current (mA)')
        plt.xlabel('Sample')
        plt.show()
    elif (selection == 'l'):
        # Go to angle (deg)
        n_str = input("Enter the desired motor angle in degrees: ")
        n_int = float(n_str)  # turn it into an int
        ser.write((str(n_int)+'\n').encode())

        n_str = ser.read_until(b'\n')
        n_str = str(n_str)  # turn it into an int
        print(n_str)  # print it to the screen
    elif (selection == 'm'):
        # Load step trajectory
        ref = genRef('step')
        print(len(ref))
        t = range(len(ref))
        plt.plot(t, ref, 'r*-')
        plt.ylabel('value')
        plt.xlabel('index')
        plt.show()

        # Send the number of points first
        ser.write((str(len(ref))+'\n').encode())

        # Send each point of the trajectory
        for point in ref:
            ser.write((str(point)+'\n').encode())

        # Read confirmation from PIC
        response = ser.read_until(b'\n')
        print(response.decode().strip())
    elif (selection == 'n'):
        # Load cubic trajectory
        ref = genRef('cubic')
        print(len(ref))
        t = range(len(ref))
        plt.plot(t, ref, 'r*-')
        plt.ylabel('value')
        plt.xlabel('index')
        plt.show()

        # Send the number of points first
        ser.write((str(len(ref))+'\n').encode())

        # Send each point of the trajectory
        for point in ref:
            ser.write((str(point)+'\n').encode())

        # Read confirmation from PIC
        response = ser.read_until(b'\n')
        print(response.decode().strip())
    elif (selection == 'o'):
        # Execute trajectory
        n_str = ser.read_until(b'\n')
        n_int = int(n_str)  # turn it into an int
        print('Data length = ' + str(n_int))
        ref = []
        data = []
        data_received = 0
        while data_received < n_int:
            # get the data as a string, ints seperated by spaces
            dat_str = ser.read_until(b'\n')
            dat_f = list(map(float, dat_str.split()))  # now the data is a list
            ref.append(dat_f[0])
            data.append(dat_f[1])
            data_received = data_received + 1
        meanzip = zip(ref, data)
        meanlist = []
        for i, j in meanzip:
            meanlist.append(abs(i-j))
        score = mean(meanlist)
        t = range(len(ref))  # index array
        plt.plot(t, ref, 'r*-', t, data, 'b*-')
        plt.title('Score = ' + str(score))
        plt.ylabel('Angle (deg)')
        plt.xlabel('Sample')
        plt.show()
    elif (selection == 'p'):
        # Unpower the motor
        print("Powering down motor.")
    elif (selection == 'q'):
        print('Exiting client')
        has_quit = True  # exit client
        # be sure to close the port
        ser.close()
    elif (selection == 'r'):
        # Get mode
        n_str = ser.read_until(b'\n')
        n_str = str(n_str)
        print(n_str)
    else:
        print('Invalid Selection ' + selection_endline)
