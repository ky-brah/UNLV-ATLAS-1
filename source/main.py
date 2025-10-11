from flask_socketio import SocketIO, emit
from flask import Flask, render_template
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput
from picamera2 import Picamera2
#from enviroplus import gas
import threading
import datetime
import base64
import time
import csv
import io
import os

from sense_hat import SenseHat


#Import Sensors
#from bme280 import BME280 # Temperature / Pressure / Humidity Sensor
#from ltr559 import LTR559 # Lights / Proximity

#Setup Sensor Variables
#bme280 = BME280()
#ltr559 = LTR559()

global toggle, writing, currentConnections, readings
currentConnections = 0
toggle = False
writing = False
startTime = time.time()

sense = SenseHat()


app = Flask(__name__, template_folder="Gemini/templates", static_folder="Gemini/static") 
socket = SocketIO(app, logger=False, engineio_logger=False) 
camera = Picamera2() 
config = camera.create_video_configuration(
    buffer_count=10, #Increasing frame buffer decreases dropped frames but increases latency
    main={"size": (640, 480)},
    #controls={"FrameDurationLimits": (10_000, 10_000)} #10 FPS
    controls={"FrameDurationLimits": (10_000, 33_000)} #29 FPS
    #controls={"FrameDurationLimits": (33_333, 33_333)} #30 FPS
    )
camera.configure(config)

### Meta Functions
    
def millis(): #debugging function remove on final build
    return round((time.time()-startTime)*1000)

def setFrame(data: bytes) -> None: 
    global frame
    frame = data

def jpgToB64(data):
    img64 = "data:image/jpg;base64, " + str(base64.b64encode(data).decode("ascii")) #Converts image to base64, reformates and adds meta data.
    return(img64) #This isn't perfect but I couldn't find a different way to send frames over websockets so whatever

### Main Functions

def sensorData():
    global readings
    while True:
        try: #Sometimes the i2c bus fails and this stops that from crashing the server
            gyro = sense.get_gyroscope_raw()
            gyro_txt =f"X: {gyro['x']:.2f}, Y: {gyro['y']:.2f}, Z: {gyro['z']:.2f}"
            accel = sense.get_accelerometer_raw()
            accel_txt =f"X: {accel['x']:.2f}, Y: {accel['y']:.2f}, Z: {accel['z']:.2f}"
            mag = sense.get_compass_raw()
            mag_txt =f"X: {mag['x']:.2f}, Y: {mag['y']:.2f}, Z: {mag['z']:.2f}"
            readings = [
                str(round(sense.get_temperature(), 2))+" C",
                str(round(sense.get_pressure(), 2))+" hPa",
                str(round(sense.get_humidity(), 2))+" %",
                gyro_txt,
                accel_txt,
                mag_txt]
        except:
            print("error reading sensors, trying again.")
sensors = threading.Thread(target=sensorData) #This runs in the background because the time it takes to read sensors is enough to through off frame timing

def sendVideo(): #Sends frames to the client
    global frame, readings, currentConnections
    readings = None #If the sensors haven't read yet, this will prevent a crash
    lastFrame = None
    lastReadings = None
    while currentConnections >= 1: 
        if (frame != lastFrame):
            lastFrame = frame
            socket.emit("frame", jpgToB64(frame))
            socket.sleep(0) #Needed to stop the code from blocking
        if (readings != lastReadings):
            lastReadings = readings
            socket.emit("sensorData", readings)
            socket.sleep(0)

def cameraStart(): #Starts the camera and overwrites the stream write function to save the current frame as a variable
    global stream
    byteStream = io.BytesIO()
    byteStream.write = setFrame #Sets byteStream.write to call setFrame instead
    camera.start_recording(MJPEGEncoder(bitrate=5_000_000), FileOutput(byteStream)) #Modify bitrate to tweak performance
    print("Camera started.")

def recordLoop():
    global writing, frame, readings
    lastFrame = None
    lastReadings = None
    socket.emit("recordingIndicatorOn")
    sensor_data_file = csv.writer(open("Gemini/Sensor_Data.csv", "w"), delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
    with open("Gemini/current_recording.mjpeg", "wb") as video_file:
        sensor_data_file.writerow(["Time", "Humidity", "Pressure", "Temperature", "Light", "Oxidization", "Reduced Oxygen", "Ammonia"])
        while toggle == True:
            if (frame != lastFrame):
                lastFrame = frame
                writing = True
                video_file.write(frame) 
                writing = False
            if (readings != lastReadings):
                lastReadings = readings
                writing = True
                sensor_data_file.writerow([datetime.datetime.now().strftime("%d-%m-%Y_%I-%M-%S"), readings[0], readings[1], readings[2], readings[3], readings[4], readings[5], readings[6]])
                writing = False
    print("re-encoding recording") 
    socket.emit("recordingIndicatorOff")
    socket.emit("encodingIndicatorOn")
    #Starts an ffmpeg process to re-encode the recording to a mp4 file in the background and then deletes the file when it's done
    os.system("ffmpeg -loglevel quiet -i Gemini/current_recording.mjpeg -c:v libx264 -c:a copy Gemini/static/recording/current_recording.mp4 -y && rm Gemini/current_recording.mjpeg &")
recording = threading.Thread(target=recordLoop) #Creates a thread to record video to avoid blocking

### Websocket Functions

@socket.on("recordToggle")
def recordToggle():
    global toggle
    if toggle == False:
        toggle = True
        print("now recording")
        recording.start()
    else:
        toggle = False
        print("stopped recording")
        recording.join()
        while True:
            if os.path.isfile("Gemini/current_recording.mjpeg") == False:
                print("re-encoding complete, zipping file")
                os.system("zip -j Gemini/static/recording/" + datetime.datetime.now().strftime("%d-%m-%Y_%I:%M:%S") + ".zip Gemini/static/recording/current_recording.mp4 Gemini/Sensor_Data.csv && rm Gemini/Sensor_Data.csv && rm Gemini/static/recording/current_recording.mp4 &")
                if os.path.isfile("Gemini/current_recording.mp4") == False:
                    print("zip complete")
                    socket.emit("encodingIndicatorOff")
                    break
            else:
                socket.sleep(0)

@socket.on("connect") 
def connection():
    socket.emit("confirmConnect")
    print("New WebSocket Connection")
    if toggle == True:
        socket.emit("recordingIndicatorOn")
    else:
        socket.emit("recordingIndicatorOff")
    global currentConnections
    currentConnections += 1 

@socket.on("disconnect")
def disconnect():
    print("WebSocket Connection Closed")
    global currentConnections
    currentConnections -= 1

@socket.on("confirmConnect")
def confirmConnect():
    print("New WebSocket Connection Established")
    global currentConnections
    if currentConnections == 1: #Makes sure to only run the transmitting code when one client is connected
        sendVideo()

@socket.on("deleteCommand")
def deleteCommand():
    print("Delete command recieved")
    os.system("sudo rm Gemini/static/recording/*")

#send a list of the files inside the static/downloads folder when the client requests it
@socket.on("getFiles")
def getFiles():
    print("Sending download list")
    socket.emit("files", os.listdir("Gemini/static/recording"))

### Server Functions

@app.route('/')
def index(): #Serves main page
    return render_template("index.html")

@app.route('/downloads')
def downloads(): #Serves main page
    return render_template("downloads.html")

### Main Script
if __name__ == '__main__':
    try:
        cameraStart() 
        sensors.start()
        socket.run(app, host='0.0.0.0', port=80, debug=False) 
    finally:
        while True:
            if writing == False: #if the program exits when writing to a file, the file will be empty.
                break
            else:
                print("waiting to finish writing")
        camera.stop_recording()
