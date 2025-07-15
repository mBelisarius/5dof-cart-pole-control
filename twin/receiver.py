import abc
import firebase_admin
import json
import threading
import time

from firebase_admin import credentials
from firebase_admin import db
from imu import ImuData, ImuRawData
from vec3 import Vec3


class Receiver(abc.ABC):
    @abc.abstractmethod
    def receive_raw(self):
        return NotImplementedError()

    @abc.abstractmethod
    def receive_imu(self):
        return NotImplementedError()


class ReceiverFirebase(Receiver):
    def __init__(self, host, auth, poll_interval=0.01, proxies=None):
        self.host = host
        self._auth = credentials.Certificate(auth)
        self._proxies = proxies

        # Initialize Firebase app once
        self._app = firebase_admin.initialize_app(
            self._auth, options={"databaseURL": self.host}
        )

        # Shared state
        self._latest_raw = None
        self._latest_imu = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        # Start polling thread
        self._thread = threading.Thread(
            target=self._poll_loop,
            args=(poll_interval,),
            daemon=True
        )
        self._thread.start()

    def _get_content(self, path):
        ref = db.reference(path, app=self._app)
        return ref.get()

    def _poll_loop(self, poll_interval):
        while not self._stop_event.is_set():
            raw_data = self._fetch_raw_blocking()
            imu_data = self._fetch_imu_blocking()

            with self._lock:
                self._latest_raw = raw_data
                self._latest_imu = imu_data

            time.sleep(poll_interval)

    def _fetch_raw_blocking(self):
        data = self._get_content("/raw")
        if not data:
            return None

        ts = data.get("timestamp", 0) / 1000.0
        xdd = data.get("xdd", {})
        gd = data.get("gd", {})

        return ImuRawData(
            ts,
            Vec3(xdd.get('x',0), xdd.get('y',0), xdd.get('z',0)),
            Vec3(gd.get('x',0),   gd.get('y',0),   gd.get('z',0))
        )

    def _fetch_imu_blocking(self):
        data = self._get_content("/imu")
        if not data:
            return None

        ts = data.get("timestamp", 0) / 1000.0

        return ImuData(
            ts,
            Vec3(*data.get('x',   [0,0,0])),
            Vec3(*data.get('xd',  [0,0,0])),
            Vec3(*data.get('xdd', [0,0,0])),
            Vec3(*data.get('g',   [0,0,0])),
            Vec3(*data.get('gd',  [0,0,0])),
            Vec3(*data.get('gdd', [0,0,0]))
        )

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def receive_raw(self):
        with self._lock:
            return self._latest_raw

    def receive_imu(self):
        with self._lock:
            return self._latest_imu
