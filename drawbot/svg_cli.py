import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from .backend import load_svg, svg_to_strokes, SvgError
from .strokes import Placement, StrokeList, MARKERS
from .svg_pipeline import _travel


def main():
    parser = argparse.ArgumentParser(description="Convert SVG artwork to board-mm robot strokes.")
    parser.add_argument("input",type=Path)
    parser.add_argument("--width",type=float,default=100.)
    parser.add_argument("--x",type=float,default=10.)
    parser.add_argument("--y",type=float,default=10.)
    parser.add_argument("--board-width",type=float,default=300.)
    parser.add_argument("--board-height",type=float,default=300.)
    parser.add_argument("--margin",type=float,default=10.)
    parser.add_argument("--spacing",type=float,default=2.)
    parser.add_argument("--no-optimize",action="store_true")
    parser.add_argument("--output",type=Path,default=Path("output/svg"))
    args=parser.parse_args()
    try:
        if not all(math.isfinite(v) for v in (args.board_width,args.board_height,args.margin)) or min(args.board_width,args.board_height)<=0 or args.margin<0:
            raise SvgError("Board dimensions must be positive and margin non-negative.")
        started=time.perf_counter()
        drawing=load_svg(args.input.read_bytes())
        placement=Placement(args.x,args.y,args.width,args.width*drawing.aspect)
        if not (args.margin<=args.x and args.margin<=args.y and args.x+args.width<=args.board_width-args.margin and args.y+placement.height<=args.board_height-args.margin):
            raise SvgError("Placement exceeds the board edge margin. Reduce width or move the artwork.")
        raw=svg_to_strokes(drawing,placement,args.spacing,False)
        sample_time=time.perf_counter()-started
        started_order=time.perf_counter()
        strokes=svg_to_strokes(drawing,placement,args.spacing,not args.no_optimize)
        conversion_time=time.perf_counter()-started_order
        report={"input":str(args.input),"stroke_count":len(strokes),"point_count":sum(len(s.points) for s in strokes),"ink_length_mm":sum(s.length() for s in strokes),"travel_before_mm":_travel(raw),"travel_after_mm":_travel(strokes),"load_and_unoptimized_conversion_s":sample_time,"conversion_s":conversion_time,"warnings":drawing.warnings,"robot_simulation_verified":False,"real_robot_verified":False}
        output=StrokeList(args.board_width,args.board_height,placement,strokes)
        namespace="http://www.w3.org/2000/svg"
        preview=ET.Element(f"{{{namespace}}}svg",{"viewBox":f"0 0 {args.board_width} {args.board_height}","width":"900","role":"img","aria-label":"Board-mm stroke preview: top-left origin, y down"})
        ET.SubElement(preview,f"{{{namespace}}}rect",{"width":str(args.board_width),"height":str(args.board_height),"fill":"white","stroke":"#777"})
        for stroke in strokes:
            ET.SubElement(preview,f"{{{namespace}}}polyline",{"points":" ".join(f"{x},{y}" for x,y in stroke.points),"fill":"none","stroke":MARKERS[stroke.marker],"stroke-width":"0.35"})
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/"strokes.json").write_text(output.to_json(decimals=6),encoding="utf-8")
        (args.output/"report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
        (args.output/"preview.svg").write_text(ET.tostring(preview,encoding="unicode"),encoding="utf-8")
        print(json.dumps(report,indent=2))
        print(f"Outputs: {args.output.resolve()}")
    except (SvgError,OSError) as exc:
        parser.exit(1,f"Error: {exc}\n")


if __name__=="__main__":
    main()
