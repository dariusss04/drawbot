import math
import unittest
import xml.etree.ElementTree as ET
from drawbot.backend import FillSettings, load_svg, svg_to_strokes, SvgError, match_color
from drawbot.strokes import Placement
from drawbot.svg_pipeline import FILL_OUTLINE_WARNING, _travel


def svg(body):
    return ('<svg xmlns="http://www.w3.org/2000/svg">'+body+'</svg>').encode()


class PipelineTests(unittest.TestCase):
    def strokes(self, body, optimize=False, spacing=2):
        drawing = load_svg(svg(body))
        placement = Placement(10,20,100,100*drawing.aspect)
        return drawing, svg_to_strokes(drawing,placement,spacing,optimize)

    def test_circle_size_and_spacing(self):
        drawing, strokes = self.strokes('<circle cx="50" cy="50" r="50" stroke="black" fill="none"/>')
        points = strokes[0].points
        self.assertEqual(drawing.bbox,(0.,0.,100.,100.))
        self.assertEqual(points[0],points[-1])
        self.assertAlmostEqual(max(x for x,y in points)-min(x for x,y in points),100)
        self.assertLess(max(math.dist(a,b) for a,b in zip(points,points[1:])),6)
        self.assertTrue(all(type(v) is float for p in points for v in p))

    def test_nested_transform(self):
        d,_ = self.strokes('<g transform="translate(10 20)"><g transform="scale(2)"><rect width="10" height="20"/></g></g>')
        self.assertEqual(d.bbox,(10.,20.,20.,40.))

    def test_rotation(self):
        d,_=self.strokes('<g transform="rotate(90)"><rect x="10" y="20" width="10" height="20"/></g>')
        for a,b in zip(d.bbox,(-40,10,20,10)):
            self.assertAlmostEqual(a,b)

    def test_explicit_coincident_moveto(self):
        _,s=self.strokes('<path d="M0 0L10 0 M10 0L10 10"/>')
        self.assertEqual([x.source for x in s],["0.0","0.1"])

    def test_relative_moveto(self):
        _,s=self.strokes('<path d="M0 0L10 0 m0 10 l10 0"/>')
        self.assertEqual(len(s),2)
        self.assertEqual(s[1].points[0],(60.,70.))

    def test_all_path_commands(self):
        _,s=self.strokes('<path d="M0 0 L10 0 H20 V10 C20 20 30 20 30 10 S40 0 40 10 Q45 20 50 10 T60 10 A10 5 0 0 1 80 10 Z"/>')
        self.assertEqual(len(s),1)
        self.assertEqual(s[0].points[0],s[0].points[-1])

    def test_basic_shapes(self):
        body='<line x1="0" y1="0" x2="10" y2="10"/><polyline points="0,0 10,10 20,0"/><polygon points="0,0 10,10 20,0"/><ellipse cx="30" cy="20" rx="5" ry="10"/><rect x="40" y="10" width="10" height="20" rx="2"/>'
        _,s=self.strokes(body)
        self.assertEqual(len(s),5)

    def test_colors_and_inheritance(self):
        d,_=self.strokes('<g stroke="rgb(100%,0%,0%)" fill="none"><rect width="10" height="10"/><circle cx="20" cy="10" r="5" style="stroke:rebeccapurple"/></g>')
        self.assertEqual(d.colors,{"0":"#ff0000","1":"#663399"})

    def test_hidden_shapes_do_not_change_bounds(self):
        d,_=self.strokes('<rect width="10" height="10"/><g display="none"><rect x="100" width="30" height="30"/></g><rect x="200" width="20" height="20" opacity="0"/>')
        self.assertEqual(d.bbox,(0.,0.,10.,10.))

    def test_sanitized_markup_and_warning(self):
        d,_=self.strokes('<script>alert(1)</script><text>Hello</text><image href="https://example.com/a"/><rect width="10" height="10" onclick="bad()"/>')
        root=ET.fromstring(d.markup)
        self.assertTrue(all(e.tag.rsplit("}",1)[-1] in {"svg","path"} for e in root.iter()))
        self.assertNotIn("onclick",d.markup)
        self.assertTrue(any("Text" in w for w in d.warnings))

    def test_source_stability_and_optimization(self):
        drawing=load_svg(svg('<rect x="100" width="10" height="10"/><rect width="10" height="10"/><rect x="50" width="10" height="10"/>'))
        placement=Placement(10,20,100,100*drawing.aspect)
        original=svg_to_strokes(drawing,placement,optimize=False)
        ordered=svg_to_strokes(drawing,placement,optimize=True)
        self.assertEqual({s.source for s in original},{s.source for s in ordered})
        self.assertLessEqual(_travel(ordered),_travel(original)+1e-9)
        self.assertEqual(sum(map(lambda s:len(s.points),original)),sum(map(lambda s:len(s.points),ordered)))

    def test_preview_ids_match_sources(self):
        d,s=self.strokes('<path d="M0 0L10 0 M10 0L10 10"/>')
        ids={e.get('data-sid') for e in ET.fromstring(d.markup).iter() if e.get('data-sid') is not None}
        self.assertEqual(ids,{x.source.split('.')[0] for x in s})

    def test_invalid_inputs(self):
        for data in (b'bad',b'<html/>',svg('<text>Hello</text>'),svg('<rect width="10" height="10" fill="none"/>')):
            with self.subTest(data=data),self.assertRaises(SvgError):
                load_svg(data)

    def test_invalid_placement_and_spacing(self):
        d=load_svg(svg('<rect width="10" height="10"/>'))
        for p,spacing in [(Placement(0,0,10,20),2),(Placement(-1,0,10,10),2),(Placement(0,0,10,10),0),(Placement(0,0,10,10),float('nan'))]:
            with self.subTest(p=p,spacing=spacing),self.assertRaises(SvgError):
                svg_to_strokes(d,p,spacing)

    def test_color_contract_unchanged(self):
        cases={'#f2c500':'orange','#f07a1a':'orange','#e8418c':'orange','#9a9a9a':'black','#ffffff':'black','#000000':'black','#7dd321':'green','#21c4d6':'purple','#7b3fb5':'purple','#0d2a6b':'purple'}
        for color,expected in cases.items():
            self.assertEqual(match_color(color),expected)


class HatchTests(unittest.TestCase):
    def hatches(self, body, **settings):
        drawing = load_svg(svg(body))
        placement = Placement(0,0,100,100*drawing.aspect)
        outlines = svg_to_strokes(drawing,placement,2,False)
        return svg_to_strokes(drawing,placement,2,False,FillSettings(**settings))[len(outlines):]

    def covers(self, hatches, x, y):
        """True if a hatch line passes within half the 3.2 mm spacing of (x, y)."""
        from shapely.geometry import LineString, Point
        return any(LineString(s.points).distance(Point(x, y)) <= 1.6 + 1e-9 for s in hatches)

    def test_spacing_and_pen_inset_in_board_mm(self):
        h = self.hatches('<rect width="200" height="100" fill="red"/>')
        ys = sorted(s.points[0][1] for s in h)
        self.assertEqual(len(h), 16)
        self.assertTrue(all(abs(b-a-3.2) < 1e-9 for a,b in zip(ys,ys[1:])))
        self.assertAlmostEqual(min(s.points[0][0] for s in h), 0.4)
        self.assertEqual({s.marker for s in h}, {"orange"})

    def test_fill_rules_and_holes(self):
        evenodd = self.hatches('<path fill-rule="evenodd" d="M0 0H100V100H0Z M25 25H75V75H25Z"/>')
        nonzero_hole = self.hatches('<path d="M0 0H100V100H0Z M25 25V75H75V25Z"/>')
        nonzero_solid = self.hatches('<path d="M0 0H100V100H0Z M25 25H75V75H25Z"/>')
        self.assertFalse(self.covers(evenodd, 50, 50))
        self.assertFalse(self.covers(nonzero_hole, 50, 50))
        self.assertTrue(self.covers(nonzero_solid, 50, 50))

    def test_later_and_white_shapes_cover_earlier_fills(self):
        h = self.hatches('<rect width="100" height="100" fill="blue"/><rect x="40" y="40" width="20" height="20" fill="#fff"/>')
        self.assertFalse(self.covers(h, 50, 50))
        self.assertTrue(self.covers(h, 50, 20))

    def test_thin_shape_through_transform_gets_one_centre_line(self):
        h = self.hatches('<g transform="scale(2)"><rect width="50" height="1" fill="green"/></g>')
        self.assertEqual([s.points for s in h], [[(0.4, 1.0), (99.6, 1.0)]])

    def test_min_segment_contour_and_validation(self):
        self.assertEqual(self.hatches('<rect width="100" height="100"/><rect x="200" width="5" height="5"/>',
                                      min_segment_mm=200), [])
        contour = self.hatches('<rect width="100" height="100"/>', mode="contour")
        self.assertTrue(contour and all(s.points[0] == s.points[-1] for s in contour))
        with self.assertRaises(SvgError):
            self.hatches('<rect width="10" height="10"/>', hatch_spacing_mm=0)

    def test_fill_warning_only_when_an_area_is_filled(self):
        warned = lambda body: FILL_OUTLINE_WARNING in load_svg(svg(body)).warnings
        self.assertTrue(warned('<rect width="10" height="10" fill="red" stroke="blue"/>'))
        self.assertTrue(warned('<circle r="5"/>'))
        self.assertFalse(warned('<line x2="9" y2="9" stroke="red"/><path d="M0 0L5 5" stroke="red"/>'))
        self.assertFalse(warned('<rect width="10" height="10" fill="none" stroke="red"/><rect width="9" height="9" fill="#fff" stroke="red"/>'))

    def test_unfilled_and_line_shapes_get_no_hatching(self):
        self.assertEqual(self.hatches('<rect width="10" height="10" fill="none" stroke="red"/><line x2="9" y2="9" stroke="red"/>'), [])


if __name__ == '__main__':
    unittest.main()
