#!/usr/bin/env python3

import asyncio
import io
import time
from contextlib import suppress

import astropy.units as u
import picamera
import serial
from astropy.coordinates import AltAz, EarthLocation, SkyCoord
from astropy.io import fits
from astropy.time import Time
from astropy.utils import iers

from driver import calibrateMotor, controlledGOTO, get_position

from indi.device import Driver, properties
from indi.device import values as indi_values
from indi.device.events import Write, on
from indi.device.pool import default_pool
from indi.device.properties.const import DriverInterface, State, SwitchRule
from indi.message import const
from indi.routing import Router
from indi.transport.server import TTY as TTYServer


LATITUDE_DEG = 47.42
LONGITUDE_DEG = 13.22
ELEVATION_M = 750

LOCATION = EarthLocation(
    lat=LATITUDE_DEG * u.deg,
    lon=LONGITUDE_DEG * u.deg,
    height=ELEVATION_M * u.m,
)

iers.conf.auto_download = False

ALTITUDE_OFFSET_DEG = 0.0
AZIMUTH_OFFSET_DEG = 0.0
AZIMUTH_DIRECTION = 1.0


class RaspberryPiCamera:
    def __init__(self):
        self.camera = None

    def connect(self):
        self.camera = picamera.PiCamera()
        self.camera.resolution = (1920, 1080)
        self.camera.framerate = 30

    def disconnect(self):
        if self.camera is not None:
            self.camera.close()
            self.camera = None

    def capture(self, exposure_seconds):
        if self.camera is None:
            raise RuntimeError("Camera is not connected")

        buffer = io.BytesIO()

        self.camera.shutter_speed = max(
            1,
            int(exposure_seconds * 1_000_000),
        )
        self.camera.exposure_mode = "off"
        self.camera.capture(buffer, format="jpeg")

        return indi_values.BLOB(
            buffer.getvalue(),
            ".jpg",
        )


class TelescopeHardware:
    def __init__(self):
        self.port = None

    def connect(self):
        self.port = serial.Serial(
            "/dev/ttyUSB1",
            115200,
            timeout=1,
        )

        calibrateMotor(self.port, 1)
        time.sleep(3)

        calibrateMotor(self.port, 2)
        time.sleep(3)

        controlledGOTO(1, 0.0, self.port)
        time.sleep(0.1)
        controlledGOTO(2, 0.0, self.port)

    def disconnect(self):
        if self.port is not None and self.port.is_open:
            self.port.close()

        self.port = None

    def goto(self, ra_hours, dec_degrees):
        if self.port is None:
            raise RuntimeError("Telescope is not connected")

        target = SkyCoord(
            ra=(ra_hours * 15.0) * u.deg,
            dec=dec_degrees * u.deg,
            frame="icrs",
        )

        horizontal = target.transform_to(
            AltAz(
                obstime=Time.now(),
                location=LOCATION,
            )
        )

        altitude_deg = horizontal.alt.deg
        azimuth_deg = horizontal.az.deg

        if altitude_deg < 0.0:
            raise ValueError(
                f"Target is below the horizon: {altitude_deg:.2f} degrees"
            )

        altitude_motor = altitude_deg + ALTITUDE_OFFSET_DEG
        azimuth_motor = (
            AZIMUTH_DIRECTION * azimuth_deg
        ) + AZIMUTH_OFFSET_DEG

        print(
            f"GOTO RA={ra_hours:.6f} h "
            f"DEC={dec_degrees:.6f} deg "
            f"-> ALT={altitude_motor:.6f} deg "
            f"AZ={azimuth_motor:.6f} deg",
            flush=True,
        )

        controlledGOTO(1, altitude_motor, self.port)
        controlledGOTO(2, azimuth_motor, self.port)

    def position(self):
        if self.port is None:
            raise RuntimeError("Telescope is not connected")

        altitude_motor, azimuth_motor = get_position()

        altitude_deg = altitude_motor - ALTITUDE_OFFSET_DEG
        azimuth_deg = (
            azimuth_motor - AZIMUTH_OFFSET_DEG
        ) / AZIMUTH_DIRECTION

        horizontal = SkyCoord(
            alt=altitude_deg * u.deg,
            az=(azimuth_deg % 360.0) * u.deg,
            frame=AltAz(
                obstime=Time.now(),
                location=LOCATION,
            ),
        )

        equatorial = horizontal.transform_to("icrs")

        return equatorial.ra.hour, equatorial.dec.deg


@default_pool.register
class MyTelescope(Driver):
    name = "My Telescope"

    general = properties.Group(
        "GENERAL",
        vectors=dict(
            connection=properties.standard.common.Connection(),
            driver_info=properties.standard.common.DriverInfo(
                interface=(
                    DriverInterface.TELESCOPE,
                    DriverInterface.CCD,
                ),
            ),
        ),
    )

    telescope = properties.Group(
        "TELESCOPE",
        enabled=False,
        vectors=dict(
            on_coord_set=properties.SwitchVector(
                "ON_COORD_SET",
                rule=SwitchRule.ONE_OF_MANY,
                default_on="SLEW",
                elements=dict(
                    slew=properties.Switch("SLEW"),
                    track=properties.Switch("TRACK"),
                    sync=properties.Switch("SYNC"),
                ),
            ),
            coordinates=properties.NumberVector(
                "EQUATORIAL_EOD_COORD",
                elements=dict(
                    ra=properties.Number(
                        "RA",
                        format="%010.6f",
                        min=0.0,
                        max=24.0,
                    ),
                    dec=properties.Number(
                        "DEC",
                        format="%010.6f",
                        min=-90.0,
                        max=90.0,
                    ),
                ),
            ),
        ),
    )

    camera = properties.Group(
        "CAMERA",
        enabled=False,
        vectors=dict(
            exposure=properties.standard.ccd.Exposure(
                min=0.001,
                max=600.0,
            ),
            image=properties.BLOBVector(
                "CCD1",
                perm=const.Permissions.READ_ONLY,
                elements=dict(
                    image=properties.BLOB("CCD1"),
                ),
            ),
        ),
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.mount = TelescopeHardware()
        self.camera_device = RaspberryPiCamera()

        self.position_task = None
        self.goto_task = None
        self.tracking_task = None
        self.tracking_target = None

    @on(general.connection.connect, Write)
    async def connection_changed(self, event):
        connected = event.new_value == const.SwitchState.ON

        if connected:
            try:
                await asyncio.to_thread(self.mount.connect)
                await asyncio.to_thread(self.camera_device.connect)

                self.general.connection.connect.bool_value = True
                self.general.connection.state_ = State.OK

                self.telescope.enabled = True
                self.camera.enabled = True

                self.position_task = asyncio.create_task(
                    self.report_position()
                )

            except Exception as error:
                print(f"Connection failed: {error}", flush=True)

                await asyncio.to_thread(self.camera_device.disconnect)
                await asyncio.to_thread(self.mount.disconnect)

                self.general.connection.connect.bool_value = False
                self.general.connection.state_ = State.ALERT

        else:
            await self.stop_tracking()

            if self.position_task is not None:
                self.position_task.cancel()

                with suppress(asyncio.CancelledError):
                    await self.position_task

                self.position_task = None

            self.camera.enabled = False
            self.telescope.enabled = False

            await asyncio.to_thread(self.camera_device.disconnect)
            await asyncio.to_thread(self.mount.disconnect)

            self.general.connection.connect.bool_value = False
            self.general.connection.state_ = State.OK

    @on(telescope.coordinates.ra, Write)
    @on(telescope.coordinates.dec, Write)
    async def coordinates_changed(self, event):
        if self.goto_task is not None and not self.goto_task.done():
            self.goto_task.cancel()

            with suppress(asyncio.CancelledError):
                await self.goto_task

        self.goto_task = asyncio.create_task(self.do_goto())

    @on(camera.exposure.time, Write)
    async def exposure_requested(self, event):
        try:
            exposure_seconds = event.new_value
            self.camera.exposure.state_ = State.BUSY

            image = await asyncio.to_thread(
                self.camera_device.capture,
                exposure_seconds,
            )

            self.camera.image.image.value = image
            self.camera.exposure.state_ = State.OK

        except asyncio.CancelledError:
            raise

        except Exception as error:
            print(f"Camera exposure failed: {error}", flush=True)
            self.camera.exposure.state_ = State.ALERT

    async def do_goto(self):
        try:
            await asyncio.sleep(0.05)

            ra_hours = self.telescope.coordinates.ra.value
            dec_degrees = self.telescope.coordinates.dec.value

            self.tracking_target = (ra_hours, dec_degrees)

            await self.stop_tracking()

            self.telescope.coordinates.state_ = State.BUSY

            await asyncio.to_thread(
                self.mount.goto,
                ra_hours,
                dec_degrees,
            )

            self.telescope.coordinates.state_ = State.OK

            if self.telescope.on_coord_set.selected_value == "TRACK":
                self.tracking_task = asyncio.create_task(
                    self.tracking_loop()
                )

        except asyncio.CancelledError:
            raise

        except Exception as error:
            print(f"GOTO failed: {error}", flush=True)
            self.telescope.coordinates.state_ = State.ALERT

    async def tracking_loop(self):
        while True:
            try:
                if self.tracking_target is None:
                    return

                ra_hours, dec_degrees = self.tracking_target

                await asyncio.to_thread(
                    self.mount.goto,
                    ra_hours,
                    dec_degrees,
                )

                await asyncio.sleep(4.0)

            except asyncio.CancelledError:
                raise

            except Exception as error:
                print(f"Tracking failed: {error}", flush=True)
                self.telescope.coordinates.state_ = State.ALERT
                return

    async def stop_tracking(self):
        if self.tracking_task is not None:
            task = self.tracking_task
            self.tracking_task = None
            task.cancel()

            with suppress(asyncio.CancelledError):
                await task

    async def report_position(self):
        while True:
            try:
                ra_hours, dec_degrees = await asyncio.to_thread(
                    self.mount.position
                )

                self.telescope.coordinates.ra.value = ra_hours
                self.telescope.coordinates.dec.value = dec_degrees
                self.telescope.coordinates.state_ = State.OK

                await asyncio.sleep(1.0)

            except asyncio.CancelledError:
                raise

            except Exception as error:
                print(f"Position update failed: {error}", flush=True)
                self.telescope.coordinates.state_ = State.ALERT
                await asyncio.sleep(1.0)


async def main():
    router = Router()
    default_pool.init(router)

    server = TTYServer(router=router)
    await server.start()


if __name__ == "__main__":
    asyncio.run(main())