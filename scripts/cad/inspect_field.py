#!/usr/bin/env python3
"""Inspect official STEP assembly names and exact, untessellated bounds."""
import argparse, json, time
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepTools import BRepTools
from OCP.TDF import TDF_Tool
from OCP.TCollection import TCollection_AsciiString
from OCP.TopoDS import TopoDS_Iterator


def label_name(label):
    value=TDataStd_Name()
    return value.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),value) else ''


def entry(label):
    s=TCollection_AsciiString();TDF_Tool.Entry_s(label,s);return s.ToCString()


def bounds(shape):
    box=Bnd_Box();BRepBndLib.AddOptimal_s(shape,box,False,False)
    return list(box.Get()) if not box.IsVoid() else None


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source');ap.add_argument('output');args=ap.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    started=time.monotonic()
    reader=STEPCAFControl_Reader();reader.SetColorMode(True);reader.SetNameMode(True)
    reader.SetLayerMode(False);reader.SetPropsMode(False)
    print('READ',args.source,flush=True)
    status=reader.ReadFile(args.source)
    if status!=IFSelect_RetDone:raise RuntimeError(f'STEP read failed: {status}')
    print('READ_COMPLETE',time.monotonic()-started,flush=True)
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    if not reader.Transfer(doc):raise RuntimeError('STEP transfer failed')
    print('TRANSFER_COMPLETE',time.monotonic()-started,flush=True)
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots=TDF_LabelSequence();tool.GetFreeShapes(roots)
    tree=[]
    def visit(label,depth=0):
        shape=tool.GetShape_s(label)
        b=bounds(shape)
        row=dict(entry=entry(label),name=label_name(label),depth=depth,bounds_mm=b,
                 assembly=tool.IsAssembly_s(label),reference=tool.IsReference_s(label))
        if b:row['size_mm']=[b[i+3]-b[i] for i in range(3)]
        tree.append(row)
        if depth<=2: print(json.dumps(row,ensure_ascii=False),flush=True)
        if depth<=1:
            BRepTools.Write_s(shape,str(out/(row['entry'].replace(':','_')+'.brep')))
        children=TDF_LabelSequence();tool.GetComponents_s(label,children,False)
        if tool.IsReference_s(label):
            from OCP.TDF import TDF_Label
            referred=TDF_Label();tool.GetReferredShape_s(label,referred)
            children=TDF_LabelSequence();tool.GetComponents_s(referred,children,False)
        for i in range(1,children.Length()+1):visit(children.Value(i),depth+1)
    for i in range(1,roots.Length()+1):visit(roots.Value(i))
    (out/'assembly.json').write_text(json.dumps(tree,ensure_ascii=False,indent=2)+'\n')
    print('COMPLETE',len(tree),time.monotonic()-started,flush=True)

if __name__=='__main__':main()
