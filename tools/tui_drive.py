#!/usr/bin/env python3
"""Drive the full-screen app headless in a pseudo-terminal and print screen snapshots (needs `pip install pyte`).

usage: tui_drive.py COLS ROWS SCRIPT -- cmd args...
SCRIPT: semicolon-separated steps:
  w:SECONDS   wait (the app keeps drawing)       k:KEY      press a key
  s:LABEL     print the screen as text            x:LABEL    report whether the program has exited
KEYS: up down left right enter esc space tab ctrl-c, or a literal character.

Example (simulation: open Balance, start, place the ball, watch 10 s, snapshot, Ctrl-C):
  cargo build && python3 tools/tui_drive.py 130 44 "k:enter;k:space;w:2.5;k:b;w:10;s:balancing;k:ctrl-c;w:3;x:exit" \
      -- ./target/debug/paddle-balance -c /tmp/none.toml tui --sim --log-dir /tmp/sim_logs
"""
import fcntl
import os
import pty
import select
import struct
import sys
import termios
import time

import pyte

KEYS = {"up": "\x1b[A", "down": "\x1b[B", "right": "\x1b[C", "left": "\x1b[D", "enter": "\r", "esc": "\x1b",
        "space": " ", "tab": "\t", "ctrl-c": "\x03"}


def main():
    cols, rows, script = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
    cmd = sys.argv[sys.argv.index("--") + 1:]
    screen = pyte.Screen(cols, rows)
    stream = pyte.ByteStream(screen)
    pid, fd = pty.fork()
    if pid == 0:
        os.environ["TERM"] = "xterm-256color"
        os.execvp(cmd[0], cmd)
    fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))

    def pump(t):
        end = time.time() + t
        while True:
            left = end - time.time()
            if left <= 0:
                return
            r, _, _ = select.select([fd], [], [], left)
            if r:
                try:
                    data = os.read(fd, 65536)
                except OSError:
                    return
                if not data:
                    return
                stream.feed(data)

    pump(0.8)
    for step in script.split(";"):
        kind, _, arg = step.partition(":")
        if kind == "w":
            pump(float(arg))
        elif kind == "k":
            os.write(fd, KEYS.get(arg, arg).encode())
            pump(0.15)
        elif kind == "x":
            r = os.waitpid(pid, os.WNOHANG)
            print(f"===== exit check: {'still running' if r == (0, 0) else 'exited, status %d' % r[1]} =====")
        elif kind == "s":
            print(f"===== {arg} =====")
            for line in screen.display:
                print(line.rstrip())
    try:
        os.kill(pid, 9)
    except OSError:
        pass


if __name__ == "__main__":
    main()
