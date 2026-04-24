"""Minimal client for the existing teleop robot-node ZMQ protocol.

The matching server currently lives in teleop.zmq_core.robot_node. This module
reimplements only the read-side pickle protocol so franka_capture does not need
to import teleop.
"""

import pickle
from typing import Any, Dict, Optional

import numpy as np
import zmq


class RobotZMQClient:
    """Read-only client for the robot node.

    Only state-reading methods are exposed here. This keeps the capture script
    from accidentally sending robot motion commands.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 6001,
        timeout_ms: int = 2000,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_ms = timeout_ms
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"tcp://{host}:{port}")

    def _request(self, method: str, args: Optional[Dict[str, Any]] = None) -> Any:
        request = {"method": method, "args": args or {}}
        try:
            self._socket.send(pickle.dumps(request))
            result = pickle.loads(self._socket.recv())
        except zmq.Again as exc:
            raise TimeoutError(
                f"Timed out waiting for robot node at tcp://{self.host}:{self.port}"
            ) from exc

        if isinstance(result, dict) and "error" in result:
            raise RuntimeError(f"Robot node returned error: {result['error']}")
        return result

    def num_dofs(self) -> int:
        return int(self._request("num_dofs"))

    def get_joint_state(self) -> np.ndarray:
        return np.asarray(self._request("get_joint_state"))

    def get_observations(self) -> Dict[str, Any]:
        return self._request("get_observations")

    def close(self) -> None:
        self._socket.close(0)
        self._context.term()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
