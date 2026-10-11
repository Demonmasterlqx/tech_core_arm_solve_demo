#!/usr/bin/env python3
"""Bounded X11/XTest acceptance test of the real planning demo window.

Run in xvfb-run. Uses only Python's standard library, libX11 and libXtst.
Viewer processes, held keys and X11 connections are released in finally blocks.
Output directories must be new; event logs rotate at 1 MiB in the viewer.
"""
import argparse
import ctypes as C
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time


class Keyboard:
    def __init__(self):
        self.x11, self.xtst = C.CDLL('libX11.so.6'), C.CDLL('libXtst.so.6')
        ptr, win = C.c_void_p, C.c_ulong
        self.x11.XOpenDisplay.argtypes, self.x11.XOpenDisplay.restype = [C.c_char_p], ptr
        self.x11.XDefaultRootWindow.argtypes, self.x11.XDefaultRootWindow.restype = [ptr], win
        self.x11.XQueryTree.argtypes = [ptr, win, C.POINTER(win), C.POINTER(win), C.POINTER(C.POINTER(win)), C.POINTER(C.c_uint)]
        self.x11.XFetchName.argtypes = [ptr, win, C.POINTER(C.c_char_p)]
        self.x11.XFree.argtypes = [ptr]
        self.x11.XStringToKeysym.argtypes, self.x11.XStringToKeysym.restype = [C.c_char_p], C.c_ulong
        self.x11.XKeysymToKeycode.argtypes, self.x11.XKeysymToKeycode.restype = [ptr, C.c_ulong], C.c_uint
        self.x11.XSetInputFocus.argtypes = [ptr, win, C.c_int, C.c_ulong]
        self.x11.XFlush.argtypes = [ptr]
        self.x11.XCloseDisplay.argtypes = [ptr]
        self.xtst.XTestFakeKeyEvent.argtypes = [ptr, C.c_uint, C.c_int, C.c_ulong]
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('Cannot open X11 DISPLAY')
        self.root = self.x11.XDefaultRootWindow(self.display)
        self.held = set()

    def key(self, name, down):
        code = self.x11.XKeysymToKeycode(self.display, self.x11.XStringToKeysym(name.encode()))
        if not code or not self.xtst.XTestFakeKeyEvent(self.display, code, int(down), 0):
            raise RuntimeError(f'XTest failed for {name}')
        self.x11.XFlush(self.display)
        (self.held.add if down else self.held.discard)(name)

    def focus(self, window):
        self.x11.XSetInputFocus(self.display, window, 1, 0)
        self.x11.XFlush(self.display)
        time.sleep(.15)

    def locate(self, parent=None):
        parent = self.root if parent is None else parent
        name = C.c_char_p()
        if self.x11.XFetchName(self.display, parent, C.byref(name)) and name.value:
            title = name.value.decode(errors='replace')
            self.x11.XFree(C.cast(name, C.c_void_p))
            if title.startswith('RM26 Planning Demo |'):
                return parent
        win = C.c_ulong
        root, owner, children, count = win(), win(), C.POINTER(win)(), C.c_uint()
        if self.x11.XQueryTree(self.display, parent, C.byref(root), C.byref(owner), C.byref(children), C.byref(count)):
            ids = [children[i] for i in range(count.value)]
            if children:
                self.x11.XFree(children)
            for child in ids:
                found = self.locate(child)
                if found:
                    return found
        return None

    def release(self):
        for name in list(self.held):
            self.key(name, False)

    def close(self):
        self.release()
        self.x11.XCloseDisplay(self.display)


class Run:
    def __init__(self, binary, output, keyboard, case='dev_nominal'):
        self.output, self.keyboard = output, keyboard
        output.mkdir()
        self.events = output / 'events.jsonl'
        self.png = output / 'current.png'
        self.stdout = (output / 'stdout.log').open('w')
        self.process = subprocess.Popen([
            str(binary), '--case', case, '--duration', '180', '--log', str(self.events),
            '--screenshot', str(self.png)], stdout=self.stdout, stderr=subprocess.STDOUT)
        try:
            self.initial = self.wait('ready')
            self.window = keyboard.locate()
            if not self.window:
                raise RuntimeError('Planning window not found')
            keyboard.focus(self.window)
        except BaseException:
            self.close()
            raise

    def records(self):
        if not self.events.exists():
            return []
        # Ignore a concurrently written last line until the next poll.
        lines = self.events.read_text().splitlines(keepends=True)
        return [json.loads(line) for line in lines if line.endswith('\n')]

    def wait(self, event=None, after=0, state=None, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            matches = [r for r in self.records()[after:]
                       if (event is None or r['event'] == event) and (state is None or r['state'] == state)]
            if matches:
                return matches[-1]
            if self.process.poll() is not None:
                raise RuntimeError(f'Viewer exited {self.process.returncode}: {self.output / "stdout.log"}')
            time.sleep(.02)
        raise RuntimeError(f'Timeout {event}/{state}: {self.output / "stdout.log"}')

    def tap(self, key, event, state=None):
        offset = len(self.records())
        self.keyboard.key(key, True)
        time.sleep(.07)
        self.keyboard.key(key, False)
        result = self.wait(event, offset, state)
        time.sleep(.08)
        return result

    def jog(self, key, shift=False, duration=.35):
        offset = len(self.records())
        if shift:
            self.keyboard.key('Shift_L', True)
        self.keyboard.key(key, True)
        self.wait('motion_start', offset)
        time.sleep(duration)
        self.keyboard.key(key, False)
        if shift:
            self.keyboard.key('Shift_L', False)
        return self.wait('motion_stop', offset)

    def capture(self, name):
        result = self.tap('F12', 'screenshot')
        shutil.copyfile(self.png, self.output / name)
        return result

    def close(self):
        self.keyboard.release()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.stdout.close()

    def exit(self):
        self.tap('Escape', 'exit')
        self.process.wait(timeout=10)
        assert self.process.returncode == 0


def unchanged(a, b):
    assert max(abs(x-y) for x, y in zip(a['q'], b['q'])) < 1e-10, (a, b)
    assert max(abs(x-y) for x, y in zip(a['tcp'], b['tcp'])) < 1e-10, (a, b)
    assert a['mechanism'] == b['mechanism'] and a['time'] == b['time'], (a, b)


def zeroed(state):
    assert state['q'] == [0.0] * 7, state
    assert state['mechanism'] == [0.0] * 4 and state['time'] == 0, state
    assert state['state'] == 'EDITING' and 'knots' not in state, state


def bidirectional_q(state):
    assert state['success'] and 0 < state['q_positive_degrees'] <= 90, state
    assert -90 <= state['q_final_degrees'] < 0, state
    assert state['q_positive_stop'] and state['q_negative_stop'], state
    full = (math.isclose(state['q_positive_degrees'], 90) and
            math.isclose(state['q_final_degrees'], -90))
    assert state['full_q_sweep'] == full, state


def gpu_snapshot(output, label):
    if shutil.which('nvidia-smi'):
        with (output / f'gpu-{label}.txt').open('w') as log:
            subprocess.run(['nvidia-smi', '--query-gpu=index,uuid,memory.used', '--format=csv'], stdout=log, check=True)
            subprocess.run(['nvidia-smi'], stdout=log, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('build/rm26_demo'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not os.environ.get('DISPLAY'):
        parser.error('Run inside xvfb-run or an isolated X11 DISPLAY')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(CUDA_VISIBLE_DEVICES='', LIBGL_ALWAYS_SOFTWARE='1',
                      GALLIUM_DRIVER='llvmpipe', __GLX_VENDOR_LIBRARY_NAME='mesa', LP_NUM_THREADS='4')
    keyboard, run = Keyboard(), None
    report = {'checks': [], 'arm_directions': [], 'core_controls': [], 'failures': []}
    try:
        gpu_snapshot(args.output, 'before')
        run = Run(args.binary.resolve(), args.output / 'nominal', keyboard)
        report['viewer_pid'] = run.process.pid
        gpu_snapshot(args.output, 'startup')
        zeroed(run.initial)
        report['startup'] = run.capture('startup-zero.png')
        unchanged(run.initial, report['startup'])
        report['checks'].append('Startup uses upstream URDF zero with all seven joint angles exactly zero')
        offset = len(run.records())
        run.tap('Return', 'enter', 'PLANNING')
        zero_preview = run.wait('state', after=offset)
        assert zero_preview['state'] == 'PREVIEW_READY' and zero_preview['requested_points'] > 0, zero_preview
        assert zero_preview['preinsert_ik_attempts'] >= 2 and zero_preview['approach_seconds'] > 0, zero_preview
        bidirectional_q(zero_preview)
        unchanged(run.initial, run.capture('zero-start-preview.png'))
        report['zero_start_preview'] = zero_preview
        report['zero_start_previews'] = [zero_preview]
        for key in ['1', '2']:
            run.tap(key, 'backend')
            offset = len(run.records())
            run.tap('Return', 'enter', 'PLANNING')
            ready = run.wait('state', after=offset)
            assert ready['state'] == 'PREVIEW_READY' and ready['preinsert_ik_attempts'] >= 2, ready
            bidirectional_q(ready)
            unchanged(run.initial, run.capture(f'zero-start-backend-{key}-preview.png'))
            report['zero_start_previews'].append(ready)
        offset = len(run.records())
        run.tap('Return', 'enter', 'EXECUTING')
        zero_complete = run.wait(state='COMPLETE', after=offset, timeout=45)
        assert zero_complete['time'] > 0 and any(abs(q) > .1 for q in zero_complete['q']), zero_complete
        assert math.isclose(zero_complete['mechanism'][3], math.radians(ready['q_final_degrees']), abs_tol=1e-9), zero_complete
        report['zero_start_complete'] = zero_complete
        run.capture('zero-start-complete.png')
        zeroed(run.tap('BackSpace', 'home'))
        report['checks'].append('Both backends preview a collision-checked approach from URDF zero; second Enter executes the complete assembly')
        preset = run.tap('r', 'reset')
        assert any(abs(q) > .1 for q in preset['q']) and 'knots' not in preset
        report['assembly_preset'] = preset
        for key, axis, sign in [('w', 0, 1), ('s', 0, -1), ('a', 1, 1), ('d', 1, -1), ('q', 2, 1), ('e', 2, -1)]:
            before = run.tap('r', 'reset')
            after = run.jog(key)
            delta = [y-x for x, y in zip(before['tcp'], after['tcp'])]
            assert delta[axis] * sign > .005, (key, delta)
            assert max(abs(v) for i, v in enumerate(delta) if i != axis) < .005, (key, delta)
            held = run.tap('2', 'backend')
            unchanged(after, held)
            report['arm_directions'].append({'key': key, 'delta_m': delta, 'release_holds': True})

        before = run.tap('r', 'reset')
        for key, axis, sign, shift in [('i',0,1,False), ('k',0,-1,False), ('j',1,1,False),
                ('l',1,-1,False), ('u',2,1,False), ('o',2,-1,False),
                ('i',0,1,True), ('k',0,-1,True), ('l',1,-1,True), ('j',1,1,True),
                ('u',2,1,True), ('o',2,-1,True)]:
            after = run.jog(key, shift=shift, duration=.25)
            field = 'core_rpy' if shift else 'core_world'
            delta = [y-x for x, y in zip(before[field], after[field])]
            assert delta[axis] * sign > (8 if shift else .025), (key, shift, delta)
            report['core_controls'].append({'key': ('Shift+' if shift else '')+key, 'delta': delta})
            before = after
        home = run.tap('BackSpace', 'home')
        assert home['core_world'] == before['core_world'] and home['core_rpy'] == before['core_rpy']
        zeroed(home)
        report['joint_zero'] = run.capture('joint-zero.png')
        unchanged(home, report['joint_zero'])
        report['checks'].append('Backspace sets all seven joints exactly to zero, clears progress and preserves core XYZ/RPY')

        run.tap('r', 'reset')
        low = run.jog('o')
        assert low['core_world'][2] < run.initial['core_world'][2] - .04, low
        tilted = run.jog('j', shift=True)
        assert tilted['core_rpy'][1] > 10, tilted
        report['free_core'] = run.capture('free-core.png')
        report['checks'].append('Core moves below the former Z minimum and above the former pitch maximum')

        run.tap('r', 'reset')
        outside = run.jog('k', duration=1.0)
        assert outside['core_world'][0] < .30, outside  # Default station front plane.
        report['free_core_previews'] = []
        for key in ['1', '2']:
            run.tap(key, 'backend')
            offset = len(run.records())
            run.tap('Return', 'enter', 'PLANNING')
            preview = run.wait('state', after=offset)
            assert preview['state'] in ('PREVIEW_READY', 'PREVIEW_FAILED'), preview
            assert preview['requested_points'] > 0 and preview['reason'] != 'invalid_station_front_plane', preview
            displayed = run.capture(f'free-core-backend-{key}-preview.png')
            unchanged(outside, displayed)
            assert displayed['core_world'] == outside['core_world'] and displayed['core_rpy'] == outside['core_rpy']
            assert displayed['ghost_geoms'] > 0
            if preview['state'] == 'PREVIEW_FAILED':
                blocked = run.tap('Return', 'enter', 'PREVIEW_FAILED')
                unchanged(outside, blocked)
            report['free_core_previews'].append(preview)
        report['checks'].append('Both backends preview beyond the former front plane with the real arm/core held; failed plans cannot execute')

        initial = run.tap('r', 'reset')
        unchanged(preset, initial)
        assert initial['core_world'] == run.initial['core_world'] and initial['core_rpy'] == run.initial['core_rpy']
        report['checks'].append('R loads the separate nonzero assembly preset and its core pose')
        run.tap('1', 'backend')
        offset = len(run.records())
        keyboard.key('Return', True)
        ready = run.wait(state='PREVIEW_READY', after=offset)
        time.sleep(.65)  # Key repeat after completion must not execute.
        keyboard.key('Return', False)
        held = run.capture('baseline-preview.png')
        unchanged(initial, held)
        assert ready['backend'] == 'baseline' and held['state'] == 'PREVIEW_READY'
        bidirectional_q(ready)
        report['baseline_preview'] = ready
        assert ready['requested_points'] == zero_preview['requested_points'], (ready, zero_preview)
        report['checks'].append('Assembly request starts at the pre-insertion pose independently of current arm joints')
        report['checks'].append('Baseline first Enter and held Enter keep arm/core frozen at preview')

        changed = run.tap('2', 'backend')
        assert changed['state'] == 'EDITING' and 'knots' not in changed
        run.tap('Return', 'enter', 'PLANNING')
        ready = run.wait(state='PREVIEW_READY', after=len(run.records())-1)
        bidirectional_q(ready)
        time.sleep(.3)
        held = run.capture('rigid-cache-preview.png')
        unchanged(initial, held)
        assert held['ghost_geoms'] > 0
        assert max(abs(x-y) for x, y in zip(held['q'], held['ghost_q'])) > .001
        run.tap('space', 'preview_pause')
        for _ in range(10):
            run.tap('bracketright', 'preview_scrub')
        ghost = run.capture('rigid-cache-ghost-preview.png')
        unchanged(initial, ghost)
        assert ghost['ghost_paused'] and ghost['ghost_time'] > held['ghost_time']
        time.sleep(.3)
        frozen = run.capture('ghost-paused.png')
        assert ghost['ghost_q'] == frozen['ghost_q'] and ghost['ghost_time'] == frozen['ghost_time']
        run.tap('space', 'preview_pause')
        time.sleep(.3)
        animated = run.capture('ghost-resumed.png')
        unchanged(initial, animated)
        assert animated['ghost_time'] != frozen['ghost_time']
        report['ghost_preview'] = ghost
        report['checks'].append('Translucent arm/core ghost animates, pauses and scrubs while opaque arm/core remain fixed')
        report['successful_preview'] = held
        report['checks'].append('Backend switch invalidates preview; RigidCache previews without execution')
        run.tap('Return', 'enter', 'EXECUTING')
        time.sleep(.25)
        paused = run.tap('space', 'pause', 'PAUSED')
        assert max(abs(x-y) for x, y in zip(initial['q'], paused['q'])) > .005
        time.sleep(.2)
        unchanged(paused, run.capture('paused.png'))
        run.tap('space', 'pause', 'EXECUTING')
        offset = len(run.records())
        keyboard.focus(keyboard.root)
        lost = run.wait('focus_lost', offset, 'PAUSED')
        keyboard.focus(run.window)
        unchanged(lost, run.capture('focus-paused.png'))
        gpu_snapshot(args.output, 'execution')
        offset = len(run.records())
        run.tap('space', 'pause', 'EXECUTING')
        complete = run.wait(state='COMPLETE', after=offset, timeout=45)
        assert math.isclose(complete['mechanism'][3], math.radians(ready['q_final_degrees']), abs_tol=1e-9), complete
        report['complete'] = complete
        run.capture('complete.png')
        report['checks'].append('Second Enter executes, Space/focus loss hold, resume reaches complete assembly')
        report['checks'].append('Both backends preview both Q directions from zero and the preset; execution holds the validated negative endpoint')

        zeroed(run.tap('BackSpace', 'home'))
        run.tap('r', 'reset')
        offset = len(run.records())
        run.tap('Return', 'enter', 'PLANNING')
        home = run.tap('BackSpace', 'home')
        zeroed(home)
        run.wait('discarded', offset)
        unchanged(home, run.capture('stale-discarded.png'))
        report['checks'].append('Backspace during planning holds exact zero joints and discards a stale successful result')
        run.exit(); run.close(); run = None

        for case in ['confirm_top', 'reject_blocked']:
            run = Run(args.binary.resolve(), args.output / case, keyboard, case)
            zeroed(run.initial)
            case_initial = run.tap('r', 'reset')
            run.tap('Return', 'enter', 'PLANNING')
            failed = run.wait(state='PREVIEW_FAILED')
            assert failed['requested_points'] > 0 and not failed['success']
            displayed = run.capture('failed-preview.png')
            unchanged(case_initial, displayed)
            assert displayed['ghost_geoms'] > 0
            if displayed['knots']:
                run.tap('space', 'preview_pause')
                for _ in range(12):
                    run.tap('bracketright', 'preview_scrub')
                ghost = run.capture('failed-ghost-preview.png')
                unchanged(case_initial, ghost)
                assert max(abs(x-y) for x, y in zip(ghost['q'], ghost['ghost_q'])) > .01
            else:
                assert displayed['ghost_q'] == case_initial['q'] and displayed['ghost_time'] == 0
            blocked = run.tap('Return', 'enter', 'PREVIEW_FAILED')
            unchanged(displayed, blocked)
            if case == 'reject_blocked':
                assert blocked['knots'] == 0
                assert blocked['reason'].startswith('start_collision_'), blocked
            report['failures'].append(blocked)
            run.exit(); run.close(); run = None
        report['checks'].append('Partial and zero-knot failures are displayed and second Enter cannot execute them')
        renderer = next(line.strip().split(': ', 1)[1] for line in
                        (args.output/'nominal/stdout.log').read_text().splitlines() if line.startswith('OpenGL renderer:'))
        assert 'llvmpipe' in renderer.lower(), renderer
        report['renderer'], report['result'] = renderer, 'PASS'
    finally:
        if run is not None:
            run.close()
        keyboard.close()
        gpu_snapshot(args.output, 'shutdown')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
