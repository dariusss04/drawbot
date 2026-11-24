from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from streamlit.testing.v1 import AppTest
from drawbot import ui_bridge as bridge
from drawbot.robot import api as robot_api
from drawbot.robot.connection import connect_existing
from drawbot.robot.circle_check import check_circle
from robodk.robolink import (ITEM_TYPE_FRAME, ITEM_TYPE_PROGRAM, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET, ITEM_TYPE_TOOL,
                             INS_TYPE_MOVE, PROGRAM_RUN_ON_SIMULATOR)
from robodk.robomath import invH

report = {'started_at_utc':datetime.now(timezone.utc).isoformat(), 'method':'Streamlit AppTest with real uploaded files, original SVG Pipeline, real program generator and local RoboDK', 'physical_robot_used':False, 'animation_played':False, 'cases':{}}
output = ROOT / 'logs/web_ui_e2e.json'


def save():
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def result_case(app, label, action='Simulate in RoboDK', expect_success=True):
    print('START', label, flush=True)
    button = next(b for b in app.button if b.label == action)
    assert not button.disabled, (label, 'button disabled')
    button.click().run(timeout=240)
    assert not app.exception, str(app.exception)
    result = app.session_state['job_result']
    request_path = Path(result['ui_request_path'])
    data = json.loads(request_path.with_name('strokes.json').read_text(encoding="utf-8"))
    metadata = json.loads(request_path.read_text(encoding="utf-8"))
    assert result['ok'] == expect_success, (label, result.get('errors'), result.get('report_path'))
    entry = {'success':result['ok'], 'expected_success':expect_success, 'request':str(request_path), 'source_name':metadata['source_name'], 'svg_sha256':metadata['svg_sha256'], 'stroke_list_sha256':metadata['stroke_list_sha256'], 'settings':metadata['settings'], 'placement':data['placement'], 'strokes':len(data['strokes']), 'points':sum(len(s['points']) for s in data['strokes']), 'markers':[s['marker'] for s in data['strokes']], 'result':result}
    report['cases'][label] = entry
    save()
    if expect_success:
        core = result['report']
        assert core['input_sha256']==metadata['stroke_list_sha256']
        assert core['valid_ratio']==1.0 and core['counts_match']
        assert not result['animation_played']
        assert '<in-memory' not in core['data_path'] and 'sample_strokes.json' not in core['data_path']
        rdk = connect_existing()
        try:
            program=rdk.Item(result['program_name'], ITEM_TYPE_PROGRAM)
            assert program.RunType()==PROGRAM_RUN_ON_SIMULATOR
            frame=rdk.Item(core['target_frame_name'])
            targets={}; pending=[frame]
            while pending:
                parent=pending.pop()
                for child in parent.Childs():
                    if child.Parent()!=parent: continue
                    if child.Type()==ITEM_TYPE_TARGET: targets[child.Name()]=child
                    else: pending.append(child)
            error=0; joint_error=0
            robot=rdk.Item('UR5e', ITEM_TYPE_ROBOT); board=rdk.Item('BoardFrame', ITEM_TYPE_FRAME); tool=rdk.Item('PenTCP', ITEM_TYPE_TOOL)
            for move in core['motion_manifest']:
                target=targets[move['target']]
                if move['target']=='Home_Park':
                    actual=(invH(board.PoseAbs())*robot.PoseAbs()*robot.SolveFK(target.Joints())*tool.PoseTool()).Pos()
                    joint_error=max(joint_error,max(abs(a-b) for a,b in zip(actual,move['xyz_mm'])))
                else:
                    actual=target.Pose().Pos()
                    error=max(error,max(abs(a-b) for a,b in zip(actual,move['xyz_mm'])))
            assert error < 1e-8, ('target error', error)
            assert joint_error < 1e-4, ('park target error', joint_error)
            for index, stroke in enumerate(data['strokes'],1):
                contact=[m['xyz_mm'][:2] for m in core['motion_manifest'] if m['stroke']==index and ('PenDown' in m['target'] or '_Draw_' in m['target'])]
                assert contact==stroke['points'], (label,index,'coordinate/order mismatch')
            expected_calls = [] if core['pen_change_mode']=='manual_preloaded' else [p['program'] for p in core['pen_requests']]
            instructions=[program.Instruction(i) for i in range(program.InstructionCount())]
            calls=[]; order=[]
            for i, ins in enumerate(instructions):
                if any(p in ins[0] for p in ('ChangePen_Black','ChangePen_Orange','ChangePen_Green','ChangePen_Purple')):
                    name=next(p for p in expected_calls if p in ins[0])
                    calls.append(name)
                    previous=next((x[0] for x in reversed(instructions[:i]) if x[1]==INS_TYPE_MOVE),None)
                    following=next((x[0] for x in instructions[i+1:] if x[1]==INS_TYPE_MOVE),None)
                    assert previous is None or 'Retract' in previous or 'Home' in previous,(label,'change before retract',previous)
                    assert following and 'Approach' in following,(label,'next motion not approach',following)
                    order.append({'call':name,'previous_motion':previous,'next_motion':following})
            assert calls==expected_calls,(label,calls,expected_calls)
            entry['actual_target_error_mm']=error
            entry['actual_pen_call_order']=order
            entry['simulator_run_type']=True
        finally:
            rdk.Disconnect()
        files=bridge.downloadable_files(result)
        entry['download_files_verified']=[str(p) for p,content in files]
        if action=='Generate robot program':
            assert {p.suffix for p,_ in files}=={'.urp','.script','.urscript'}
            assert len([b for b in app.get('download_button') if b.label.startswith('Download UI_')])==3
        save()
    print('DONE',label,'ok=',result['ok'],'strokes=',entry['strokes'],'points=',entry['points'],'ratio=',result.get('valid_ratio'),flush=True)
    return data,result


try:
    try:
        with urllib.request.urlopen('http://127.0.0.1:8511/_stcore/health',timeout=5) as response:
            report['streamlit_http_health']=response.read().decode()
    except OSError:
        report['streamlit_http_health']='no server running (not needed)'
    assert robot_api.status()['ok']
    app=AppTest.from_file(ROOT/'app/streamlit_app.py',default_timeout=240).run()
    assert not app.exception,str(app.exception)
    def mode(name):
        app.session_state['mode']=name
        app.run()
        assert not app.exception,str(app.exception)
    mode('Robot')
    assert (app.number_input(key='approach').value,app.number_input(key='v_draw').value,app.number_input(key='v_lift').value)==(80,150,150)
    assert app.slider(key='live_speed').value==20
    mode('Drawing')
    app.radio(key='marker_mode').set_value('Multiple markers').run()
    next(s for s in app.selectbox if s.label=='Export pen mode').select('Single marker — manually preloaded').run()
    app.session_state['job_program_name']='UI_E2E_Circle'
    circle,circle_result=result_case(app,'A_builtin_circle_simulate')
    geometry=check_circle(circle)
    assert geometry['passed'] and geometry['point_count']==65
    report['circle_geometry']=geometry
    result_case(app,'D_single_circle_manual_export','Generate robot program')

    next(s for s in app.selectbox if s.label=='or load a sample').select('Color test, 8 colors').run()
    app.session_state['job_program_name']='UI_E2E_Multicolor'
    multicolor,multi_result=result_case(app,'B_C_builtin_multicolor_simulate')
    assert len(multicolor['strokes'])==8
    assert {s['marker'] for s in multicolor['strokes']}=={'black','orange','green','purple'}
    next(s for s in app.selectbox if s.label=='Export pen mode').select('STUB test export — no physical pen changes').run()
    assert next(b for b in app.button if b.label=='Generate robot program').disabled
    app.checkbox(key='job_stub_ack').check().run()
    result_case(app,'D_multicolor_explicit_stub_export','Generate robot program')

    custom=b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 40 20"><path d="M0 0 L40 20" fill="none" stroke="orange"/></svg>'
    next(u for u in app.file_uploader if u.key=='svg_upload').set_value(('user-diagonal.svg',custom,'image/svg+xml')).run()
    app.number_input(key='in_w').set_value(40.0).run()
    app.number_input(key='in_x').set_value(20.0).run()
    app.number_input(key='in_y').set_value(30.0).run()
    app.session_state['job_program_name']='UI_E2E_Upload'
    upload,_=result_case(app,'E_user_upload')
    assert upload['strokes'][0]['points'][0]==[20,30]
    assert upload['strokes'][0]['points'][-1]==[60,50]

    app.number_input(key='in_w').set_value(60.0).run()
    app.number_input(key='in_x').set_value(40.0).run()
    app.number_input(key='in_y').set_value(50.0).run()
    assert 'job_result' not in app.session_state
    resized,_=result_case(app,'F_changed_width_and_xy')
    assert resized['strokes'][0]['points'][0]==[40,50]
    assert resized['strokes'][0]['points'][-1]==[100,80]
    assert resized['strokes']!=upload['strokes']

    app.toggle(key='keep_colors').set_value(False).run()
    app.button(key='all_purple').click().run()
    mode('Robot'); app.number_input(key='v_draw').set_value(25.0).run(); mode('Drawing')
    recolored,recolored_result=result_case(app,'G_changed_marker_and_speed')
    assert recolored['strokes'][0]['marker']=='purple'
    assert recolored['strokes'][0]['points']==resized['strokes'][0]['points']
    assert recolored_result['report']['motion_parameters']['draw_speed_mm_s']==25

    custom2=b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 30"><path d="M0 0 L20 30 M0 30 L20 0" fill="none" stroke="green"/></svg>'
    next(u for u in app.file_uploader if u.key=='svg_upload').set_value(('user-cross.svg',custom2,'image/svg+xml')).run()
    app.number_input(key='in_w').set_value(30.0).run()
    app.number_input(key='in_x').set_value(20.0).run()
    app.number_input(key='in_y').set_value(30.0).run()
    mode('Robot'); app.number_input(key='v_draw').set_value(150.0).run(); mode('Drawing')
    app.toggle(key='keep_colors').set_value(True).run()
    app.session_state['job_program_name']='UI_E2E_Upload2'
    second,_=result_case(app,'H_different_uploaded_svg')
    assert len(second['strokes'])==2
    assert second['strokes']!=upload['strokes']

    next(s for s in app.selectbox if s.label=='Export pen mode').select('External — validated real ChangePen programs').run()
    app.checkbox(key='job_external_ack').check().run()
    _,blocked=result_case(app,'I_external_rejects_existing_stubs','Generate robot program',False)
    assert any('STUB' in message for message in blocked['errors'])

    rdk=connect_existing()
    try:
        final=ROOT/'stations/Web_UI_Integration_validated.rdk'
        rdk.Save(str(final),rdk.ActiveStation())
        report['final_station']=str(final)
    finally:rdk.Disconnect()
    report['success']=True
except Exception as exc:
    import traceback
    report.update(success=False,error=str(exc),traceback=traceback.format_exc())
    print(report['traceback'],flush=True)
finally:
    report['finished_at_utc']=datetime.now(timezone.utc).isoformat()
    save()
sys.exit(0 if report['success'] else 1)
