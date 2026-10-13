#!/usr/bin/env python3
"""Inject real X11 keyboard events into the GLFW arm window (no Python packages).

Run under Xvfb with Mesa software rendering; creates bounded logs and a JSON
report in --output, and always closes the viewer and the X11 connection.
"""
import argparse
import ctypes as C
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('build/rm26_arm'))
    parser.add_argument('--output', type=Path, default=Path('artifacts/rm26_arm/keyboard'))
    args = parser.parse_args()
    if not os.environ.get('DISPLAY'):
        parser.error('Run inside xvfb-run or provide an X11 DISPLAY')
    args.output.mkdir(parents=True, exist_ok=True)
    x11, xtst = C.CDLL('libX11.so.6'), C.CDLL('libXtst.so.6')
    ptr, window_type = C.c_void_p, C.c_ulong
    x11.XOpenDisplay.argtypes, x11.XOpenDisplay.restype = [C.c_char_p], ptr
    x11.XDefaultRootWindow.argtypes, x11.XDefaultRootWindow.restype = [ptr], window_type
    x11.XQueryTree.argtypes = [ptr, window_type, C.POINTER(window_type), C.POINTER(window_type), C.POINTER(C.POINTER(window_type)), C.POINTER(C.c_uint)]
    x11.XFetchName.argtypes = [ptr, window_type, C.POINTER(C.c_char_p)]
    x11.XFree.argtypes = [ptr]
    x11.XStringToKeysym.argtypes, x11.XStringToKeysym.restype = [C.c_char_p], C.c_ulong
    x11.XKeysymToKeycode.argtypes, x11.XKeysymToKeycode.restype = [ptr, C.c_ulong], C.c_uint
    x11.XSetInputFocus.argtypes = [ptr, window_type, C.c_int, C.c_ulong]
    x11.XFlush.argtypes = [ptr]
    x11.XCloseDisplay.argtypes = [ptr]
    xtst.XTestFakeKeyEvent.argtypes = [ptr, C.c_uint, C.c_int, C.c_ulong]
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError('Cannot open X11 display')
    held = set()
    process = None
    log_path = args.output / 'viewer.log'
    expression = re.compile(r'ARM_VIEWER event=(\w+)(?: key=(\d+))? mode=(\w+) time=([\d.]+) tool=([^ ]+) joints=([^\s]+)')

    def records():
        text = log_path.read_text() if log_path.exists() else ''
        return [dict(event=m[0], key=int(m[1]) if m[1] else None, mode=m[2], time=float(m[3]),
                     tool=[float(x) for x in m[4].split(',')], joints=[float(x) for x in m[5].split(',')])
                for m in expression.findall(text)]

    def wait_event(event, after=0, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            events = records()[after:]
            found = [x for x in events if x['event'] == event]
            if found:
                return found[-1]
            if process.poll() is not None:
                raise RuntimeError(f'Viewer exited {process.returncode}: {log_path.read_text()}')
            time.sleep(.025)
        raise RuntimeError(f'Timed out waiting for {event}; see {log_path}')

    def key(name, pressed):
        code = x11.XKeysymToKeycode(display, x11.XStringToKeysym(name.encode()))
        if not code:
            raise RuntimeError(f'Unknown X11 key {name}')
        if not xtst.XTestFakeKeyEvent(display, code, int(pressed), 0):
            raise RuntimeError('XTest key injection failed')
        x11.XFlush(display)
        if pressed:
            held.add(name)
        else:
            held.discard(name)

    def tap(name, event):
        start = len(records())
        key(name, True)
        time.sleep(.06)
        key(name, False)
        result = wait_event(event, start)
        time.sleep(.12)
        return result

    def locate(parent):
        name = C.c_char_p()
        if x11.XFetchName(display, parent, C.byref(name)) and name.value:
            title = name.value.decode(errors='replace')
            x11.XFree(C.cast(name, ptr))
            if title.startswith(('RM26 Arm |', 'Arm ')):
                return parent
        root, owner, children, count = window_type(), window_type(), C.POINTER(window_type)(), C.c_uint()
        if x11.XQueryTree(display, parent, C.byref(root), C.byref(owner), C.byref(children), C.byref(count)):
            child_ids = [children[i] for i in range(count.value)]
            if children:
                x11.XFree(children)
            for child in child_ids:
                found = locate(child)
                if found:
                    return found
        return None

    report = {'renderer': None, 'directions': [], 'joint_modes': [], 'checks': []}
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([str(args.binary.resolve()), '--screenshot', str((args.output/'keyboard.png').resolve())], stdout=log, stderr=subprocess.STDOUT)
            initial = wait_event('ready')
            window = locate(x11.XDefaultRootWindow(display))
            if not window:
                raise RuntimeError('GLFW window not found')
            x11.XSetInputFocus(display, window, 1, 0)
            x11.XFlush(display)
            time.sleep(.2)
            for name, axis, sign in [('w',0,1), ('s',0,-1), ('a',1,1), ('d',1,-1), ('q',2,1), ('e',2,-1),
                                      ('Up',0,1), ('Down',0,-1), ('Left',1,1), ('Right',1,-1)]:
                before = tap('r', 'reset')
                offset = len(records())
                key(name, True)
                wait_event('motion_start', offset)
                time.sleep(.55)
                key(name, False)
                after = wait_event('motion_stop', offset)
                delta = [b-a for a,b in zip(before['tool'], after['tool'])]
                assert delta[axis]*sign > .005, (name, delta)
                assert max(abs(x) for i,x in enumerate(delta) if i != axis) < .01, (name, delta)
                time.sleep(.15)
                held_state = tap('1', 'select')
                assert max(abs(a-b) for a,b in zip(after['joints'], held_state['joints'])) < 2e-6
                report['directions'].append({'key':name, 'delta_m':delta, 'release_holds':True})
            before = tap('r', 'reset')
            offset = len(records())
            key('w', True)
            wait_event('motion_start', offset)
            paused = tap('space', 'pause')
            key('w', False)
            time.sleep(.2)
            resumed = tap('space', 'resume')
            assert paused['time'] == resumed['time']
            assert paused['joints'] == resumed['joints']
            time.sleep(.2)
            held_state = tap('1', 'select')
            assert held_state['joints'] == resumed['joints']
            report['checks'].append('pause freezes simulation and resume has no latent motion')
            for expected in ('POSITION', 'VELOCITY'):
                changed = tap('m', 'mode')
                assert changed['mode'] == expected
                tap('r', 'reset')
                before = tap('7', 'select')
                offset = len(records())
                key('equal', True)
                wait_event('motion_start', offset)
                time.sleep(.65)
                key('equal', False)
                after = wait_event('motion_stop', offset)
                delta = after['joints'][6] - before['joints'][6]
                assert delta > .02, (expected, delta)
                assert max(abs(a-b) for a,b in zip(before['joints'][:6],after['joints'][:6])) < 2e-6
                report['joint_modes'].append({'mode':expected, 'joint_index':7, 'delta_rad':delta})
            reset = tap('r', 'reset')
            assert reset['joints'] == initial['joints']
            report['checks'].append('reset restores home')
            tap('Escape', 'exit')
            process.wait(timeout=20)
            assert process.returncode == 0
        text = log_path.read_text()
        report['renderer'] = re.search(r'OpenGL renderer: (.+)', text)[1]
        assert 'llvmpipe' in report['renderer'].lower(), report['renderer']
        assert all(math.isfinite(x) for event in records() for x in event['joints'] + event['tool'])
        report['result'] = 'PASS'
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
    finally:
        for name in list(held):
            key(name, False)
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        x11.XCloseDisplay(display)


if __name__ == '__main__':
    main()
