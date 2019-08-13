import os
import rospy
import rospkg
import mapping
import pyqtgraph as pg
from scipy.io.wavfile import read,write
from numpy.fft import fft
from numpy import int8,int16, float, linspace, meshgrid, pi, sin, cos
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QButtonGroup
import sys

from std_msgs.msg import Bool, Int8, String

from omnirobotmessages.msg import Arduinostate
import time
import subprocess

audiopath = "/home/eric/omnirobot/audiomeasurements/"

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument(action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # These directions are matched to the Arduino code
        self.directions = {0:"Forward", 1:"Back", 2:"Left", 3:"Right",
                      4:"ForwardLeft", 5:"BackRight", 6:"ForwardRight",
                       7:"BackLeft", 8:"CW", 9:"CCW"}

        self.directionsinv = {"Forward":0, "Back":1, "Left":2, "Right":3,
                      "ForwardLeft":4, "BackRight":5, "ForwardRight":6,
                       "BackLeft":7, "CW":8, "CCW":9}



        self.states = {0:"Status : Stopped", 1:"Status : Moving", 2:"Status : Calibration Mode",
                  3:"Status : Mapping", 4:"Status : Measuring"}
        self.robotstate = 1


        self._widget.Forward.pressed.connect(self.on_direction_press)
        self._widget.Back.pressed.connect(self.on_direction_press)
        self._widget.Left.pressed.connect(self.on_direction_press)
        self._widget.Right.pressed.connect(self.on_direction_press)
        self._widget.ForwardLeft.pressed.connect(self.on_direction_press)
        self._widget.ForwardRight.pressed.connect(self.on_direction_press)
        self._widget.BackLeft.pressed.connect(self.on_direction_press)
        self._widget.BackRight.pressed.connect(self.on_direction_press)
        self._widget.CW.pressed.connect(self.on_direction_press)
        self._widget.CCW.pressed.connect(self.on_direction_press)

        self._widget.Go.pressed.connect(self.on_go_pressed)
        self._widget.Stop.pressed.connect(self.on_stop_pressed)


        #
        # self._widget.EncoderLinearSpinBox.valueChanged.connect(self.on_encoder_linear_spinbox_pressed)
        # self._widget.EncoderLinearSlider.valueChanged.connect(self.on_encoder_linear_slider_pressed)
        #
        # self.on_encoder_linear_spinbox_pressed()
        #
        # self._widget.EncoderRotationSpinBox.valueChanged.connect(self.on_encoder_rotary_spinbox_pressed)
        # self._widget.EncoderRotationSlider.valueChanged.connect(self.on_encoder_rotary_slider_pressed)
        #
        # self.on_encoder_rotary_spinbox_pressed()
        #



        self._widget.ServoSpinBox.valueChanged.connect(self.on_servo_spinbox_pressed)
        self._widget.ServoSlider.valueChanged.connect(self.on_servo_slider_pressed)
        self._widget.ServoButton.pressed.connect(self.on_servo_button_pressed)

        self.on_servo_spinbox_pressed()

        self._widget.MicSpinBox.valueChanged.connect(self.on_mic_spinbox_pressed)
        self._widget.MicSlider.valueChanged.connect(self.on_mic_slider_pressed)
        self._widget.MicButton.pressed.connect(self.on_mic_button_pressed)

        self.on_mic_spinbox_pressed()

        self._widget.MicCalibrateButton.pressed.connect(self.on_mic_calibrate_button_pressed)

        self._widget.MicDirectionButton.pressed.connect(self.on_mic_direction_button_pressed)
        self.micdirection = 0

        self._widget.CameraButton.pressed.connect(self.on_camera_button_pressed)
        self._widget.SweepButton.pressed.connect(self.on_sweep_button_pressed)
        self._widget.ImpulseButton.pressed.connect(self.on_impulse_button_pressed)
        self._widget.CalibrateEncoderButton.pressed.connect(self.on_calibrate_encoder_button_pressed)

        self._widget.FrequencySlider.valueChanged.connect(self.on_frequency_slider_pressed)
        self._widget.FrequencySpinBox.valueChanged.connect(self.on_frequency_spinbox_pressed)


        # Camera image selection button groups
        self.bg1 = QButtonGroup()
        self.bg2 = QButtonGroup()

        self.bg1.addButton(self._widget.LeftImageCheckBox)
        self.bg1.addButton(self._widget.RightImageCheckBox)
        self.bg1.addButton(self._widget.DisparityImageCheckBox)
        self.bg2.addButton(self._widget.NormalImageCheckBox)
        self.bg2.addButton(self._widget.EdgeImageCheckBox)
        self.bg2.addButton(self._widget.BWImageCheckBox)
        self.bg2.addButton(self._widget.KeypointsImageCheckBox)

        self._widget.LeftImageCheckBox.pressed.connect(self.on_camera_type_changed)
        self._widget.RightImageCheckBox.pressed.connect(self.on_camera_type_changed)
        self._widget.DisparityImageCheckBox.pressed.connect(self.on_camera_type_changed)
        self._widget.EdgeImageCheckBox.pressed.connect(self.on_image_type_changed)
        self._widget.BWImageCheckBox.pressed.connect(self.on_image_type_changed)
        self._widget.KeypointsImageCheckBox.pressed.connect(self.on_image_type_changed)
        self._widget.NormalImageCheckBox.pressed.connect(self.on_image_type_changed)
        self.bg1.buttonClicked.connect(self.on_camera_type_changed)
        self.bg2.buttonClicked.connect(self.on_image_type_changed)

        self.on_camera_type_changed()
        self.on_image_type_changed()
        self.camera_type = 2
        self.image_type = 2

        self.micHeight = 15
        self.encoderCalibMode = False

        # Connect to Arduino's subscriber to control it
        self.pub2ard = rospy.Publisher('arduinoSub', Arduinostate,queue_size=10)


        self.pub2pi  = rospy.Publisher('imagetrigger', Int8, queue_size=10)
        self.pub2spkr = rospy.Publisher('speakerListener', Int8, queue_size=10)
        self.pub2mic = rospy.Publisher('audiotrigger', String, queue_size=10)
        self.r = rospy.Rate(5)
        self.arduinomsg = Arduinostate()
        #self.pimsg = Bool()
        self.pimsg = Int8()
        self.spkrmsg = Int8()
        self.micmsg = String()

        self.mapSub = rospy.Subscriber("mapPub", String, self.plotCallback)
        self.mapPub = rospy.Publisher("mapSub", String,queue_size=10)

        self.mapmsg = String()
        self.mapUtility = mapping.Map()
        self.mapUtility.z = 5

        self.freqPlot = pg.plot(title="Frequency Spectrum")
        self.distFreqPlot = pg.plot(title="Frequency vs. Distance")
        self.contourPlot = pg.ImageView(name="2D Power - Single Frequency")

        S = audiopath + "AAsaw.wav"
        fs,audio = read(S)
        self.freqPlot.plot(abs(fft(audio)))

        self.lastaudio = ""
        self.audiofiles=[]


    def plotCallback(self,data):
        S =  data.data
        if(S != self.lastaudio):
            fs,audio = read(audiopath + S + ".wav")
            self.audiofiles.append(abs(fft(audio)))
            self.freqPlot.clear()
            self.freqPlot.plot(abs(fft(audio)))
            self.lastaudio = S
            print("Showing plot of " + self.lastaudio)


            x = linspace(-100,100,201)
            xx,yy = meshgrid(x,x)
            # Go through audio files at the given height
            # Get power at the given frequency for all x,y

            #self.contourPlot.setImage(z)

    # Need separate function for Freq, amplitude distance in 1D and 2D
    # Might want one for updating files and another for updating frequency
    def distancePlot(self,axis):
        if(axis == 0): # x
            # Search for all files in path that have different x values
            g = os.listdir(audiopath)
            A=[]
            x=[]
            for k in range(len(g)):
                if "wav" in g:
                    fs,audio = read(g[k])
                    x.append(int(g[0]))
                    A.append(20*log10(abs(fft(audio))))


        self.distancePlot.plot(x,A)


    def talker(self,msgtype):
        if(msgtype == 0):
            self.arduinomsg.toggleVelocity = True
            self.arduinomsg.toggleServo = False
            self.arduinomsg.toggleFunction = False
            self.arduinomsg.vel_x = 0
            self.arduinomsg.vel_y = 0
            self.arduinomsg.vel_theta = 0
        elif(msgtype == 1): # Go in a direction for some time
            self.arduinomsg.toggleFunction = True
            self.arduinomsg.toggleVelocity = False
            self.arduinomsg.toggleServo = False
            direction = self.directionsinv[self._widget.DirectionTextBox.text()]
            time = float(self._widget.TimeLineEdit.text())
            self.arduinomsg.functionSelect = 1
            self.arduinomsg.functionIntParam = direction
            self.arduinomsg.functionFloatParam = time
            self.mapUtility.updatePosition(direction,time)
            self.mapmsg = "%d,%d,%d,%d,0" % (self.mapUtility.x, self.mapUtility.y, self.mapUtility.t, self.mapUtility.z)
            self.mapPub.publish(self.mapmsg)
            # Need additional part of message to tell map to make mic marker
            self.r.sleep()
            S = self.mapmsg.split(",")
            self.arduinomsg.x = int(S[0])
            self.arduinomsg.y = int(S[1])
            self.arduinomsg.theta = int(S[2])
            self.arduinomsg.micPosition = int(S[3])
            # For mapping, need to update coordinates here+
        elif(msgtype == 2): # Velocity control
            self.arduinomsg.toggleVelocity = True
            self.arduinomsg.toggleFunction = False
            self.arduinomsg.toggleServo = False
        elif(msgtype == 3): # Servo
            self.arduinomsg.toggleServo = True
            self.arduinomsg.toggleVelocity = False
            self.arduinomsg.toggleFunction = False
            self.arduinomsg.servoangle = int8(self._widget.ServoSpinBox.value())
        elif(msgtype == 4): # Mic
            self.arduinomsg.toggleFunction = True
            self.arduinomsg.toggleServo = False
            self.arduinomsg.toggleVelocity = False
            self.arduinomsg.functionSelect = 3
            self.arduinomsg.functionIntParam = int8(self.micdirection)
            self.arduinomsg.functionFloatParam = self._widget.MicSpinBox.value()
            self.mapUtility.z += (-1)**(self.micdirection)
            self.mapmsg = "%d,%d,%d,%d,0" % (self.mapUtility.x, self.mapUtility.y, self.mapUtility.t, self.mapUtility.z)
            self.mapPub.publish(self.mapmsg)
            self.r.sleep()
            S = self.mapmsg.split(",")
            self.arduinomsg.x = int(S[0])
            self.arduinomsg.y = int(S[1])
            self.arduinomsg.theta = int(S[2])
            self.arduinomsg.micPosition = int(S[3])

        elif(msgtype == 5): # Mic calibrate
            self.arduinomsg.toggleFunction = True
            self.arduinomsg.toggleServo = False
            self.arduinomsg.toggleVelocity = False
            self.arduinomsg.functionSelect = 4
            self.arduinomsg.functionIntParam = int8(self._widget.MicCalibrateSpinBox.value())

        elif(msgtype == 6): # Measure with mic
            pass
            # Get position of robot and mic height
            # Send trigger then start recording
            # Place marker on the map at the location and save WAV file
        elif(msgtype == 7): # Encoder calibrate
            self.arduinomsg.toggleFunction = True
            self.arduinomsg.toggleServo = False
            self.arduinomsg.toggleVelocity = False
            direction = self.directionsinv[self._widget.DirectionTextBox.text()]
            time = float(self._widget.TimeLineEdit.text())
            self.arduinomsg.functionSelect = 2
            self.arduinomsg.functionIntParam = direction
            self.arduinomsg.functionFloatParam = time
            numTrials = 10
            for k in range(numTrials):
                x1 = int16(subprocess.check_output("rostopic echo /arduinoPub/x -n1", shell=True).split('\n')[0])
                y1 = int16(subprocess.check_output("rostopic echo /arduinoPub/y -n1", shell=True).split('\n')[0])
                z1 = int16(subprocess.check_output("rostopic echo /arduinoPub/theta -n1", shell=True).split('\n')[0])
                self.pub2ard.publish(self.arduinomsg)
                x2 = int16(subprocess.check_output("rostopic echo /arduinoPub/x -n1", shell=True).split('\n')[0])
                y2 = int16(subprocess.check_output("rostopic echo /arduinoPub/y -n1", shell=True).split('\n')[0])
                z2 = int16(subprocess.check_output("rostopic echo /arduinoPub/theta -n1", shell=True).split('\n')[0])
                dx = abs(x2)-abs(x1)
                dy = abs(y2)-abs(y1)
                dz = abs(z2)-abs(z1)
                # Error is larger with long times, not reading output correctly
                #print("Trial %d : %d " %(k,dy - dz))
                # Reverse to return to original position
                if(direction == 0):
                    print("Trial %d: Error : %d , Total y: %d, theta: %d" %(k,dy - dz, dy, dz))
                    self.arduinomsg.functionIntParam = 1
                    self.pub2ard.publish(self.arduinomsg)
                    #time.sleep(time)
                    self.arduinomsg.functionIntParam = 0

                    # Right back minus left back
                    # Positive error means increase left wheel speed or decrease right wheel speed
                if(direction == 1):
                    print("Trial %d: Error : %d , Total y: %d, theta: %d" %(k,dy - dz, dy, dz))
                    self.arduinomsg.functionIntParam = 0
                    self.pub2ard.publish(self.arduinomsg)
                    #time.sleep(time)
                    self.arduinomsg.functionIntParam = 1

                # Left and right might be tricky...

                if(direction == 8 or direction == 9):
                    print("Trial %d : %d, %d, %d " %(k,dx,dy,dz))



        rospy.loginfo(self.arduinomsg) # Logging sent message, no received!
        self.pub2ard.publish(self.arduinomsg)
        self.r.sleep()

    def shutdown_plugin(self):
        sys.exit()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def updateState(self,newstate):
        self.robotstate = self.states[newstate]
        self._widget.StatusTextBox.setText(self.robotstate)

    def on_direction_press(self):
        self._widget.DirectionTextBox.setText(self.sender().objectName())

    def on_stop_pressed(self):
        self.updateState(0)
        self.talker(0)

    def on_go_pressed(self):
        self.updateState(1)
        self.talker(1)

    def on_frequency_spinbox_pressed(self):
        self._widget.FrequencySlider.setValue(self._widget.FrequencySpinBox.value())

    def on_frequency_slider_pressed(self):
        self._widget.FrequencySpinBox.setValue(self._widget.FrequencySlider.value())


    def on_servo_spinbox_pressed(self):
        self._widget.ServoSlider.setValue(self._widget.ServoSpinBox.value())

    def on_servo_slider_pressed(self):
        self._widget.ServoSpinBox.setValue(self._widget.ServoSlider.value())

    def on_mic_spinbox_pressed(self):
        self._widget.MicSlider.setValue(self._widget.MicSpinBox.value())


    def on_mic_slider_pressed(self):
        self._widget.MicSpinBox.setValue(self._widget.MicSlider.value())

    def on_mic_button_pressed(self):
        self.talker(4)

    def on_mic_calibrate_button_pressed(self):
        self._widget.MicCalibrateButton.setText("Calibrate %i" % self._widget.MicCalibrateSpinBox.value())
        self.talker(5)

    def on_mic_direction_button_pressed(self):
        if(self.micdirection): # If true, set to down
            self._widget.MicDirectionButton.setText("Mic Up" )
            self.micdirection = 0
        else:
            self._widget.MicDirectionButton.setText("Mic Down")
            self.micdirection = 1

    def on_servo_button_pressed(self):
        self.talker(3)


    # Loop it here instead of in arduino
    def on_calibrate_encoder_button_pressed(self):
        if(self.encoderCalibMode):
            self.encoderCalibMode = False
            self._widget.CalibrateMicButton.setText("Calibrate: Off")
        else:
            self.encoderCalibMode = True
            self.talker(6)
            self._widget.CalibrateEncoderButton.setText("Calibrate: On")

    def on_camera_button_pressed(self):
        if(self.camera_type == 2):
            if(self.image_type == 2):
                self.pimsg = 1
            elif(self.image_type == 3):
                self.pimsg = 2
            elif(self.image_type == 4):
                self.pimsg = 3
            elif(self.image_type == 5):
                self.pimsg = 4

        elif(self.camera_type == 3): # Right
            if(self.image_type == 2):
                self.pimsg = 5
            elif(self.image_type == 3):
                self.pimsg = 6
            elif(self.image_type == 4):
                self.pimsg = 7
            elif(self.image_type == 5):
                self.pimsg = 8


        elif(self.camera_type == 4):
            self.pimsg = 7 # Disparity map

        self.pub2pi.publish(self.pimsg)


    def on_camera_type_changed(self):
        self.camera_type = -1*self.bg1.checkedId()
        #self._widget.CameraButton.setText("Camera %i" % self.camera_type )

    def on_image_type_changed(self):
        self.image_type = -1*self.bg2.checkedId()
        #self._widget.CameraButton.setText("Camera %i" % self.image_type )


    '''
    When measurement button is triggered, get the current position to save as a marker
    '''
    def on_sweep_button_pressed(self):
        self.spkrmsg = 0
        self.mapmsg = "%d,%d,%d,%d,1" % (self.mapUtility.x, self.mapUtility.y, self.mapUtility.t, self.mapUtility.z)
        self.micmsg = self.mapmsg
        self.pub2mic.publish(self.micmsg)
        self.pub2spkr.publish(self.spkrmsg)
        self.r.sleep() # Wait until recording is done to add marker
        self.mapPub.publish(self.mapmsg)
        S = self.mapmsg.split(",")
        self.arduinomsg.x = int(S[0])
        self.arduinomsg.y = int(S[1])
        self.arduinomsg.theta = int(S[2])
        self.arduinomsg.micPosition = int(S[3])

    def on_impulse_button_pressed(self):
        self.spkrmsg = 2
        self.pub2spkr.publish(self.spkrmsg)


    # Calibrate cameras individually and in stereo to get camera parameters
    def calibrateCameras(self):
        self.updateState(2)

    def mappingMode(self):
        self.updateState(3)

    def measurementMode(self):
        self.updateState(4)

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
