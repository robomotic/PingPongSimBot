#!/usr/bin/env python3
"""Print the present position (ticks) of every ST3215 servo on every bus, read through a RUNNING NormaCore station.

Use it to capture a pose set with the leader (teleop) without stopping the station. Read-only.
Needs a norma-core checkout with station_py and the generated Python protobufs (on the Pi: ~/ncpr, venv ~/pqenv):

    ~/pqenv/bin/python tools/station_read_positions.py [--repo ~/ncpr] [--host localhost]

Output: one line per bus, "<bus serial> [M1, M2, ..., M8]". Follower = 5B14033257, leader = 5B14112350.
"""
import argparse
import asyncio
import logging
import os
import sys


async def main(a):
    sys.path.insert(0, os.path.expanduser(a.repo))
    from software.station.shared.station_py import new_station_client
    from target.gen_python.protobuf.drivers.st3215 import st3215

    c = await new_station_client(a.host, logging.getLogger("readpos"))
    q = c.read_from_tail("st3215/inference", b"\x00", 0, 1, 1)
    e = await asyncio.wait_for(q.data.get(), 10)
    s = st3215.InferenceStateReader(memoryview(bytes(e.Data)))
    for bus in s.get_buses() or []:
        pos = {}
        for m in bus.get_motors() or []:
            st = bytes(m.get_state())
            pos[m.get_id()] = int.from_bytes(st[0x38:0x3A], "little") & 0xFFF  # PresentPosition
        print(bus.get_bus().get_serial_number(), [pos[k] for k in sorted(pos)])


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default="~/ncpr")
    ap.add_argument("--host", default="localhost")
    asyncio.run(main(ap.parse_args()))
