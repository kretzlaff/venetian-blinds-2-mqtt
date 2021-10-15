import paho.mqtt.client as mqtt
from rx.core.typing import Disposable
import yaml
import time
import rx
from rx.subject import Subject
from rx.disposable import SerialDisposable
from enum import Enum
import signal
import RPi.GPIO as GPIO

config = {}


class Direction(Enum):
    Stop = 0
    Up = 1
    Down = 2


class BlindGpio:
    def __init__(self, up, down):
        self.__up = up
        self.__down = down

        # Configures how we are describing our pin numbering
        GPIO.setmode(GPIO.BCM)
        # Disable Warnings
        GPIO.setwarnings(False)

        GPIO.setup(up, GPIO.OUT)
        GPIO.setup(down, GPIO.OUT)
        GPIO.output(up, GPIO.HIGH)
        GPIO.output(down, GPIO.HIGH)

    # Ensure all values are set to off, in case of a crash
    def __del__(self):
        GPIO.output(self.__up, GPIO.HIGH)
        GPIO.output(self.__down, GPIO.HIGH)

    def up(self):
        GPIO.output(self.__up, GPIO.LOW)

    def upStop(self):
        GPIO.output(self.__up, GPIO.HIGH)

    def down(self):
        GPIO.output(self.__down, GPIO.LOW)

    def downStop(self):
        GPIO.output(self.__down, GPIO.HIGH)


class MqttClient:
    def __init__(self, address, port):
        self.observable = Subject()
        self.__client = mqtt.Client()
        self.__client.on_connect = self.__on_connect
        self.__client.on_message = self.__on_message
        self.__client.connect(address, port, 60)
        self.__client.loop_start()
        for blind in config["blinds"]:
            self.__client.subscribe(
                f'{config["mqtt_prefix"]}/{str(blind)}/set')
            self.__client.subscribe(
                f'{config["mqtt_prefix"]}/{str(blind)}/position/#')

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.__client.loop_stop()
        self.__client.disconnect()
        print("Disconnected")

    def publishState(self, blind, state):
        self._publish(blind, "state", state)

    def publishPosition(self, blind, position):
        self._publish(blind, "position", position)

    def publishTiltPosition(self, blind, position):
        self._publish(blind, "tilt-position", position)

    def _publish(self, blind, topic, message):
        self.__client.publish(
            f'{config["mqtt_prefix"]}/{blind}/{topic}', message)

    # The callback for when the client receives a CONNACK response from the server.
    def __on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("$SYS/#")

    # The callback for when a PUBLISH message is received from the server.
    def __on_message(self, client, userdata, msg):
        if str(msg.topic).startswith(config["mqtt_prefix"]):
            print(msg.topic+" "+str(msg.payload))
            self.observable.on_next(msg)


class Blind:
    def __init__(self, blind, mqttClient: MqttClient):
        self.__travelDirection = Direction.Stop
        self.__position = 0
        self.__tilt = 0
        self.__blind = blind
        self.__travelStart = 0.0
        self.__mqttClient = mqttClient
        self.__cfg = config["blinds"][self.__blind]
        self.__gpio = BlindGpio(
            self.__cfg["gpio_up"], self.__cfg["gpio_down"])
        self.__finisehdRoutine = SerialDisposable()
        self.__autoTravelRoutine = SerialDisposable()
        self.__updateRoutine = SerialDisposable()
        mqttClient.observable.subscribe(on_next=self.__handleMessage)
        self.__setDefaults()

    def __setDefaults(self):
        self.__mqttClient.publishState(self.__blind, "closed")
        self.__mqttClient.publishTiltPosition(self.__blind, 0)
        self.__mqttClient.publishPosition(self.__blind, 0)

    def __handleMessage(self, msg):
        if f"/{self.__blind}/position/set" in msg.topic:
            self.__handleStop()
            self.__handlePostition(int(msg.payload.decode("ascii")))
        elif f"/{self.__blind}/position/tilt" in msg.topic:
            self.__handleStop()
            self.__handleTilt(int(msg.payload.decode("ascii")))
        elif f"/{self.__blind}/set" in msg.topic:
            self.__handleStop()
            self.__handleButton(msg.payload.decode("ascii"))

    def __handlePostition(self, percentage):
        position = self.__cfg["travel_time"] * percentage / 100
        travelTime = abs(position - self.__position)

        if position > self.__position:
            travelTime += self.__cfg["tilt_time"] - self.__tilt
            self.__travelUp(travelTime)
        else:
            travelTime += self.__tilt
            self.__travelDown(travelTime)

    def __handleTilt(self, percentage):
        newTilt = self.__cfg["tilt_time"] * percentage / 100
        travelTime = abs(self.__tilt - newTilt)

        if newTilt > self.__tilt:
            self.__travelUp(travelTime)
        elif newTilt < self.__tilt:
            self.__travelDown(travelTime)

    def __handleButton(self, command):
        if command == "open":
            travelTime = self.__cfg["travel_time"] - \
                self.__position + \
                self.__cfg["tilt_time"] - self.__tilt
            self.__travelUp(travelTime)

        elif command == "close":
            travelTime = self.__position + self.__tilt
            self.__travelDown(travelTime)

    def __handleStop(self):
        if self.__travelDirection == Direction.Up:
            self.__travelUpFinished()

        elif self.__travelDirection == Direction.Down:
            self.__travelDownFinished()

    def __travelUp(self, travelTime):
        self.__travelDirection = Direction.Up
        self.__travelStart = time.time()
        self.__mqttClient.publishState(self.__blind, "opening")

        # Move the blind up
        self.__gpio.up()

        # Start the finished routine
        self.__finisehdRoutine.set_disposable(rx.timer(travelTime).subscribe(
            on_next=lambda _: self.__travelUpFinished()))

        # Stop the up command when auto travel takes over
        self.__autoTravelRoutine.set_disposable(rx.timer(
            self.__cfg["auto_travel_time"]).subscribe(on_next=lambda _: self.__stopTravelUp()))

        # Send MQTT Updates Every 0.2sec
        self.__updateRoutine.set_disposable(rx.interval(0.2).subscribe(
            on_next=lambda _: self.__updatePositionUp()))

    def __travelDown(self, travelTime):
        self.__travelDirection = Direction.Down
        self.__travelStart = time.time()
        self.__mqttClient.publishState(self.__blind, "closing")

        # Move the blind down
        self.__gpio.down()

        # Start the finished routine
        self.__finisehdRoutine.set_disposable(rx.timer(travelTime).subscribe(
            on_next=lambda _: self.__travelDownFinished()))

        # Stop the down command when auto travel takes over
        self.__autoTravelRoutine.set_disposable(rx.timer(
            self.__cfg["auto_travel_time"]).subscribe(on_next=lambda _: self.__stopTravelDown()))

        # Send MQTT Updates Every 0.2sec
        self.__updateRoutine.set_disposable(rx.interval(0.2).subscribe(
            on_next=lambda _: self.__updatePositionDown()))

    def __stopTravelUp(self):
        self.__gpio.upStop()

    def __stopTravelDown(self):
        self.__gpio.downStop()

    def __travelUpFinished(self):
        travelTime = round(time.time() - self.__travelStart, 1)
        self.__disposeRoutines()
        self.__gpio.upStop()
        self.__updatePositionUp()
        self.__position = self.__calculatePositionUp()
        self.__tilt = self.__calculateTiltUp()

        if travelTime >= self.__cfg["auto_travel_time"] and self.__position != self.__cfg["travel_time"]:
            self.__gpio.down()
            time.sleep(0.1)
            self.__gpio.downStop()

        self.__travelDirection = Direction.Stop

        # Update State
        if self.__tilt == self.__cfg["tilt_time"] and self.__position == self.__cfg["travel_time"]:
            self.__mqttClient.publishState(self.__blind, "open")
        else:
            self.__mqttClient.publishState(self.__blind, "stop")

    def __travelDownFinished(self):
        travelTime = round(time.time() - self.__travelStart, 1)
        self.__disposeRoutines()
        self.__gpio.downStop()
        self.__updatePositionDown()
        self.__position = self.__calculatePositionDown()
        self.__tilt = self.__calculateTiltDown()

        if travelTime >= self.__cfg["auto_travel_time"] and self.__position != 0:
            self.__gpio.up()
            time.sleep(0.1)
            self.__gpio.upStop()

        self.__travelDirection = Direction.Stop

        # Update State
        if self.__tilt == 0 and self.__position == 0:
            self.__mqttClient.publishState(self.__blind, "closed")
        else:
            self.__mqttClient.publishState(self.__blind, "stop")

    def __updatePositionUp(self):
        if self.__travelDirection == Direction.Stop:
            return
        tilt = self.__calculateTiltUp()
        if tilt != 0:
            tilt = tilt / self.__cfg["tilt_time"] * 100
        position = self.__calculatePositionUp()
        if position != 0:
            position = position / self.__cfg["travel_time"] * 100
        self.__mqttClient.publishTiltPosition(self.__blind, round(tilt))
        self.__mqttClient.publishPosition(self.__blind, round(position))

    def __updatePositionDown(self):
        if self.__travelDirection == Direction.Stop:
            return
        tilt = self.__calculateTiltDown()
        if tilt != 0:
            tilt = tilt / self.__cfg["tilt_time"] * 100
        position = self.__calculatePositionDown()
        if position != 0:
            position = position / self.__cfg["travel_time"] * 100
        self.__mqttClient.publishTiltPosition(self.__blind, round(tilt))
        self.__mqttClient.publishPosition(self.__blind, round(position))

    def __calculateTiltUp(self):
        travelTime = round(time.time() - self.__travelStart, 1)
        tilt = min(self.__cfg["tilt_time"], self.__tilt + travelTime)
        return tilt

    def __calculateTiltDown(self):
        travelTime = round(time.time() - self.__travelStart, 1)
        tilt = max(0, self.__tilt - travelTime)
        return tilt

    def __calculatePositionUp(self):
        travelTime = round(time.time() - self.__travelStart, 1)
        tiltTillMaxTime = self.__cfg["tilt_time"] - self.__tilt

        if tiltTillMaxTime > travelTime:
            res = self.__position
        else:
            res = min(self.__cfg["travel_time"],
                      self.__position + travelTime - tiltTillMaxTime)
        return res

    def __calculatePositionDown(self):
        travelTime = round(time.time() - self.__travelStart, 1)
        tiltTillMinTime = self.__tilt

        if tiltTillMinTime > travelTime:
            res = self.__position
        else:
            res = max(0, self.__position - travelTime + tiltTillMinTime)
        return res

    def __disposeRoutines(self):
        self.__updateRoutine.get_disposable().dispose()
        self.__finisehdRoutine.get_disposable().dispose()
        self.__autoTravelRoutine.get_disposable().dispose()


def main():
    global config

    with open("mqttblinds.yaml", "r") as yamlfile:
        config = yaml.load(yamlfile, Loader=yaml.FullLoader)
        print("Read config successful")

    mqttClient = MqttClient(config["mqtt_address"], config["mqtt_port"])
    blinds = []

    for blind in config["blinds"]:
        instance = Blind(str(blind), mqttClient)
        blinds.append(instance)

    signal.pause()


if __name__ == "__main__":
    main()
