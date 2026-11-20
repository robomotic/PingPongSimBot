#!/usr/bin/env python3
"""Drive the full-screen app headless in a pseudo-terminal and print screen snapshots (needs `pip install pyte`).

usage: tui_drive.py COLS ROWS SCRIPT -- cmd args...
SCRIPT: semicolon-separated steps:
  w:SECONDS   wait (the app keeps drawing)       k:KEY      press a key
  s:LABEL     print the screen as text            x:LABEL    report whether the program has exited
  p:FILE.png  save the screen as a PNG screenshot (needs Pillow; colours, bold, reverse)
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

PALETTE = {  # dark theme, ANSI names as pyte reports them
    "black": (40, 42, 54), "red": (255, 85, 85), "green": (80, 250, 123), "brown": (241, 250, 140),
    "blue": (98, 114, 164), "magenta": (255, 121, 198), "cyan": (139, 233, 253), "white": (200, 200, 200),
    "brightblack": (110, 115, 140), "brightred": (255, 140, 120), "brightgreen": (105, 255, 148),
    "brightyellow": (255, 255, 165), "brightblue": (130, 150, 230), "brightmagenta": (255, 146, 223),
    "brightcyan": (164, 255, 255), "brightwhite": (255, 255, 255),
}
BG, FG = (30, 31, 41), (230, 230, 230)


def color(name, default):
    if name == "default":
        return default
    if name in PALETTE:
        return PALETTE[name]
    if len(name) == 6:
        try:
            return tuple(int(name[i:i + 2], 16) for i in (0, 2, 4))
        except ValueError:
            pass
    return default


def screenshot(screen, path, size=15):
    from PIL import Image, ImageDraw, ImageFont
    regular = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", size)
    bold = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", size)
    cw = round(regular.getlength("M"))
    ch = round(size * 1.25)
    pad = 12
    img = Image.new("RGB", (screen.columns * cw + 2 * pad, screen.lines * ch + 2 * pad), BG)
    d = ImageDraw.Draw(img)
    for y in range(screen.lines):
        row = screen.buffer[y]
        for x in range(screen.columns):
            c = row[x]
            fg, bg = color(c.fg, FG), color(c.bg, BG)
            if c.reverse:
                fg, bg = bg, fg
            x0, y0 = pad + x * cw, pad + y * ch
            if bg != BG:
                d.rectangle([x0, y0, x0 + cw - 1, y0 + ch - 1], fill=bg)
            o = ord(c.data[0]) if c.data else 32
            if 0x2801 <= o <= 0x28FF:  # braille: draw the dots (2 x 4 grid, Unicode bit order)
                r = max(1, cw // 6)
                for bit, (dx, dy) in enumerate([(0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2), (0, 3), (1, 3)]):
                    if (o - 0x2800) >> bit & 1:
                        cx = x0 + (dx * 2 + 1) * cw / 4
                        cy = y0 + (dy * 2 + 1) * ch / 8
                        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fg)
            elif 0x2581 <= o <= 0x2588:  # lower blocks: crisp bars
                h = round(ch * (o - 0x2580) / 8)
                d.rectangle([x0, y0 + ch - h, x0 + cw - 1, y0 + ch - 1], fill=fg)
            elif c.data and c.data != " ":
                d.text((x0, y0 + (ch - size) // 2 - 1), c.data, font=bold if c.bold else regular, fill=fg)
    img.save(path)


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
        elif kind == "p":
            screenshot(screen, arg)
            print(f"===== saved {arg} =====")
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
