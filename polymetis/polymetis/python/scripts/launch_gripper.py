#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import subprocess
import os
import time
import logging
import threading

import hydra

from polymetis.robot_servers import GripperServerLauncher
from polymetis.utils.grpc_utils import check_server_exists
from polymetis.utils.data_dir import BUILD_DIR

log = logging.getLogger(__name__)


def _exit_when_child_exits(child_pid):
    watched_pid, status = os.waitpid(child_pid, 0)
    if watched_pid != child_pid:
        return
    if os.WIFEXITED(status):
        exit_code = os.WEXITSTATUS(status)
        if exit_code == 0:
            log.info("Gripper client exited; shutting down gripper server.")
        else:
            log.error(
                "Gripper client exited with code %s; shutting down gripper server.",
                exit_code,
            )
        os._exit(exit_code)
    if os.WIFSIGNALED(status):
        signal_number = os.WTERMSIG(status)
        log.error(
            "Gripper client exited from signal %s; shutting down gripper server.",
            signal_number,
        )
        os._exit(128 + signal_number)


@hydra.main(config_name="launch_gripper")
def main(cfg):
    log.info(f"Adding {BUILD_DIR} to $PATH")
    os.environ["PATH"] = BUILD_DIR + os.pathsep + os.environ["PATH"]

    if cfg.gripper:
        pid = os.fork()
    else:
        pid = os.getpid()  # doesn't fork so only the server gets launched

    if pid > 0:
        # Run server
        if cfg.gripper:
            threading.Thread(
                target=_exit_when_child_exits,
                args=(pid,),
                daemon=True,
            ).start()
        gripper_server = GripperServerLauncher(cfg.ip, cfg.port)
        gripper_server.run()

    else:  # (this block does not run if gripper=none)
        # Wait for server to launch
        # Use localhost for connection check since server binds to 0.0.0.0
        t0 = time.time()
        while not check_server_exists("localhost", cfg.port):
            time.sleep(0.1)
            if time.time() - t0 > cfg.timeout:
                raise ConnectionError("Robot client: Unable to locate server.")

        # Run client
        try:
            gripper_client = hydra.utils.instantiate(cfg.gripper)
            gripper_client.run()
        except subprocess.CalledProcessError as e:
            print(f"[Subprocess failed] Command: {e.cmd}")
            print(f"Return code: {e.returncode}")
            print(f"Output: {e.output}")
            raise
        except Exception:
            log.exception("Gripper client failed.")
            raise
         
        


if __name__ == "__main__":
    main()
