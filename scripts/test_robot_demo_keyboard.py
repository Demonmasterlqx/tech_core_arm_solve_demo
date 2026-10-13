#!/usr/bin/env python3
"""Exercise selected robot model through the real X11 planning preview/execution UI."""
import argparse
import json
import os
from pathlib import Path
import time
from test_planning_demo_keyboard import Keyboard, Run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,default=Path('build/arm_demo'))
    parser.add_argument('--robot',default='mec_arm')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if not os.environ.get('DISPLAY'):
        parser.error('Run with xvfb-run or an X11 DISPLAY')
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ.update(CUDA_VISIBLE_DEVICES='',LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',LP_NUM_THREADS='4')
    keyboard,run = Keyboard(),None
    report={}
    try:
        run=Run(args.binary.resolve(),args.output/'viewer',keyboard,robot=args.robot)
        report['startup']=run.initial
        n=len(run.initial['q'])
        assert run.initial['robot']==args.robot and n>0
        reset=run.tap('r','reset')
        assert len(reset['q'])==n
        report['preset']=run.capture('preset.png')
        offset=len(run.records())
        run.tap('Return','enter','PLANNING')
        preview=run.wait('state',after=offset,timeout=20)
        assert preview['state']=='PREVIEW_READY' and preview['success'], preview
        assert preview['q']==reset['q'] and preview['time']==0, 'Preview moved actual robot'
        report['preview']=run.capture('preview.png')
        run.tap('Return','enter','EXECUTING')
        deadline=time.monotonic()+50
        completed=None
        while time.monotonic()<deadline:
            records=run.records()
            matches=[row for row in records if row['state']=='COMPLETE']
            if matches:
                completed=matches[-1];break
            assert run.process.poll() is None
            time.sleep(.05)
        assert completed is not None, 'Execution did not reach complete state'
        assert len(completed['q'])==n and completed['time']>0
        report['complete']=run.capture('complete.png')
        run.tap('BackSpace','home','EDITING')
        assert len(run.records()[-1]['q'])==n
        run.exit()
        report['passed']=True
    finally:
        if run:
            run.close()
        keyboard.close()
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'robot':args.robot,'passed':report['passed'],'dof':n}))


if __name__=='__main__':
    main()
