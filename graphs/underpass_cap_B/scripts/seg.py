"""Minimal prepared-loop cap: hold X2's A strand, pass the remote tail under X2,
then carry it over the far X3 strand before releasing.  ``start_shape=pre_thread``
only: every result is a diagnostic segment and is never a from-straight success.
"""
from __future__ import annotations
import json, math
from pathlib import Path
import numpy as np
from knotlab import diagram, skills, task, thread_lab, tip_feedback


def _go(ctx, arm, xy, height, rot, steps=300, speed=0.15):
    r=ctx.tool("robot.go_to_pose", arm_id=int(arm), pose=skills._pose((float(xy[0]),float(xy[1]),skills.BENCH_Z+float(height)),rot), max_steps=int(steps))
    return {"outcome":skills._outcome(r), "remaining_distance":r.get("remaining_distance") if isinstance(r,dict) else None}


def _xy_heading(ctx,s,reference=None):
    p=ctx.tool("rope.point",s=float(s)); x=np.asarray(p["position"][:2],float)
    h=math.radians(float(p["heading_deg"])); u=np.array([-math.sin(h),math.cos(h)],float)
    if reference is not None and float(u@reference)<0:u=-u
    return x,u


def run(ctx):
    p=json.loads((Path(__file__).resolve().parents[1]/"params.json").read_text())
    target=np.asarray(ctx.tool("rope.layout")["target_nodes"],float)-task.BENCH_ORIGIN
    c1,c2,c3=[thread_lab.target_crossing(target,k) for k in (1,2,3)]
    arc=diagram.arc_length(target); L=float(arc[-1]); tip_s=L-0.04
    # reference tangent from target c2 A strand, used only to choose a stable sign.
    i=int(np.searchsorted(arc,c2["s_a"])); ref=target[min(i+2,len(target)-1),:2]-target[max(i-2,0),:2]; ref/=max(np.linalg.norm(ref),1e-9)
    # The remote B strand's tangent is the direction in which the free tail
    # enters X2.  Using the live A tangent made the pass tangent to the arch
    # and caused seed-dependent IK/unreachable failures.
    ib=int(np.searchsorted(arc,c2["s_b"])); b_ref=target[min(ib+2,len(target)-1),:2]-target[max(ib-2,0),:2]; b_ref/=max(np.linalg.norm(b_ref),1e-9)
    i3=int(np.searchsorted(arc,c3["s_b"])); b3_ref=target[min(i3+2,len(target)-1),:2]-target[max(i3-2,0),:2]; b3_ref/=max(np.linalg.norm(b3_ref),1e-9)
    anchor_s=float(c2["s_a"])
    report={"claim_scope":"prepared pre_thread loop diagnostic; no from-straight tying claim",
            "params":p,"target":{"c1":c1,"c2":c2,"c3":c3,"length":L},"stages":[]}
    anchor_cap=None; tip_cap=None; anchor=None; tip=None; rot=None; u=None; x=None
    def state(label,**extra):
        st=ctx.tool("rope.state"); nodes=np.asarray(st["nodes"],float)-task.BENCH_ORIGIN
        score=thread_lab.segment_score(nodes,target)
        row={"label":label,"crossings":len(st.get("crossings",[])),"code":st.get("code"),
             "progress":st.get("progress"),"score":score,"max_height":st.get("max_height"),
             "anchor_s":anchor_s,"tip_s":tip_s}
        if anchor_cap is not None:
            try: row["anchor_drift"]=tip_feedback.drift(ctx,anchor_cap,s=anchor_s)
            except Exception as e: row["anchor_drift_error"]=repr(e)
        if tip_cap is not None:
            try: row["tip_drift"]=tip_feedback.drift(ctx,tip_cap,s=tip_s)
            except Exception as e: row["tip_drift_error"]=repr(e)
        row.update(extra); report["stages"].append(row); return st,row
    state("before")
    # Anchor is on the A strand of X2, rather than the remote end of X1. This
    # is the actual helper arch that the under-pass must retain.
    anchor=skills.pinch(ctx,0,anchor_s,low=0.012,fraction=0.95,open_width=0.035,verify_lift=0.02)
    report["anchor_grasp"]={k:anchor.get(k) for k in ("held","rise","s","at")}
    state("after_anchor_grasp")
    if not anchor.get("held"):
        # One bounded inward offset catches a 10 mm edge miss without silently
        # calling the diagnostic a success.
        skills.retreat(ctx,0,anchor.get("rot"),up=.04)
        anchor=skills.pinch(ctx,0,anchor_s,low=.012,fraction=.95,open_width=.035,verify_lift=.02,offset_xy=(-.01,0.0))
        report["anchor_retry"]={k:anchor.get(k) for k in ("held","rise","s","at")}
    if not anchor.get("held"):
        state("anchor_failed"); return {"route":"done","report":report}
    axy=np.asarray(anchor["at"],float); ar=anchor["rot"]
    amove=_go(ctx,0,axy,.075,ar)
    report["anchor_lift"]={**amove,"held":True}
    anchor_cap=tip_feedback.capture(ctx,0,s=anchor_s)
    state("helper_hold",anchor_lift=amove)
    # Take the remote tail. This is deliberately separate from the anchor so
    # the cap exposes grip/slide failures before any under-pass movement.
    tip=skills.pinch(ctx,1,tip_s,low=.012,fraction=.95,open_width=.035,verify_lift=.02)
    report["tip_grasp"]={k:tip.get(k) for k in ("held","rise","s","at")}
    tip_cap=tip_feedback.capture(ctx,1,s=tip_s) if tip.get("held") else None
    state("tip_held")
    if not tip.get("held"):
        state("tip_failed"); return {"route":"done","report":report}
    # Keep helper arch up. Aim from the live c2 A point and tangent; the tail
    # enters from the negative tangent side, then passes to the positive side.
    x,_a_u=_xy_heading(ctx,anchor_s,ref)
    # Cross the helper A strand along the remote B strand direction, not along
    # A itself.  This is the pass direction recorded in the cap report.
    u=b_ref.copy()
    theta=math.radians(float(p.get("tilt",75)))
    approach={"x":float(math.sin(theta)*u[0]),"y":float(math.sin(theta)*u[1]),"z":float(-math.cos(theta))}
    # Rotate in a clear high pose and approach horizontally with a plain
    # tool-down frame.  Directly combining a 75-degree tilt with the long
    # diagonal move is an IK singularity for the softer E=1e5 cord.
    rot=skills.frame(ctx,1,math.degrees(math.atan2(u[1],u[0])))
    here=np.asarray(ctx.tool("rope.point",s=tip_s)["position"][:2])
    app=x-u*float(p.get("approach_back",.035)); pass_xy=x+u*float(p.get("pass_ahead",.035))
    report["under_geometry"]={"x":x.tolist(),"u":u.tolist(),"approach":app.tolist(),"pass":pass_xy.tolist()}
    report["tip_clear_before_approach"]=_go(ctx,1,here,.12,tip["rot"],steps=300)
    report["tip_approach_high"]=_go(ctx,1,app,.12,rot,steps=350)
    report["tip_approach"]=_go(ctx,1,app,float(p.get("tip_height",.028)),rot,steps=350)
    state("under_approach")
    report["under_pass"]=_go(ctx,1,pass_xy,float(p.get("tip_height",.028)),rot,steps=400)
    st,row=state("under_pass_closed",tip_past=float((np.asarray(ctx.tool("rope.point",s=tip_s)["position"][:2])-x)@u),under_outcome=report["under_pass"])
    before_n=report["stages"][-2]["crossings"]
    report["under_pass"]["crossing_increase"]=row["crossings"]-before_n
    # A) after the under-pass, clear the working gripper vertically and
    # sideways before any far-side action.  This tests whether X2 survives
    # removal of the tool's lateral load while the helper remains up.
    mode=str(p.get("mode","B"))
    tip_tcp,_=skills.ee(ctx,1)
    if tip_tcp is not None:
        retreat_xy=np.asarray(tip_tcp[:2],float)
        report["tip_vertical_retreat"]=_go(ctx,1,retreat_xy,.10,rot,steps=320)
        side_xy=retreat_xy+np.array([0.025,-0.025])
        report["tip_side_clear"]=_go(ctx,1,side_xy,.10,rot,steps=260)
    _, retreat_row=state("under_retreat")
    anchor_gate=float(retreat_row.get("anchor_drift",{}).get("xyz",1.0)) <= .025
    report["anchor_gate"]={"threshold_m":.025,"passed":anchor_gate,
                            "drift_m":retreat_row.get("anchor_drift",{}).get("xyz")}
    if mode == "A":
        report["x3_action"]={"skipped":True,"reason":"hold c2 after under-pass; no broad X3 motion"}
    elif not anchor_gate:
        # Never trade a drifting helper for a prettier far-side trace.
        report["x3_action"]={"skipped":True,"reason":"anchor_drift_gate"}
    else:
        # B) high-route only: re-read the live c3 point/tangent, move the
        # held tail laterally while high (0.10 m), then descend at the short
        # outside target.  There is no low pass_xy->c3 sweep.
        tip_cap=tip_feedback.capture(ctx,1,s=tip_s)
        x3,a3_u=_xy_heading(ctx,float(c3["s_a"]),ref)
        a3_u=np.asarray(a3_u,float); a3_u/=max(np.linalg.norm(a3_u),1e-9)
        geom=tip_feedback.point_state(ctx,s=float(c3["s_a"]),inset=.02)
        live_c3_tan=np.asarray(geom["tangent"][:2],float); live_c3_tan/=max(np.linalg.norm(live_c3_tan),1e-9)
        # The remote B tangent chooses the side outside the eye; the live A
        # tangent is retained in the report so this choice is auditable.
        outside=b3_ref.copy(); outside/=max(np.linalg.norm(outside),1e-9)
        target_xy=x3+min(float(p.get("x3_out",.02)),.02)*outside
        report["x3_geometry"]={"x3_live":x3.tolist(),"a3_tangent_live":live_c3_tan.tolist(),"outside_b_ref":outside.tolist(),"target":target_xy.tolist()}
        report["x3_high_move"]=tip_feedback.move_tip(ctx,arm_id=1,s=tip_s,target_xy=target_xy,capture=tip_cap,height=.10,speed=.10,inset=.02,verify=True)
        report["x3_low_drop"]=tip_feedback.move_tip(ctx,arm_id=1,s=tip_s,target_xy=target_xy,capture=tip_cap,height=.03,speed=.10,inset=.02,verify=True)
        report["x3_action"]={"lag_gate":float(report["x3_low_drop"].get("drift",{}).get("xy",1.0))<=.025}
        state("x3_high_drop",tip_past_x3=float((np.asarray(ctx.tool("rope.point",s=tip_s)["position"][:2])-x3)@outside))
    # Release tip in place, then move sideways/up before lowering helper. This
    # separates release slip from helper-anchor drift.
    tcp,_=skills.ee(ctx,1); rel={}
    if tcp is not None:
        rel["down"]=_go(ctx,1,tcp[:2],.017,rot,steps=200)
        ctx.tool("robot.wait_steps",steps=20)
        ctx.tool("robot.set_grip",width_m=.06,arm_id=1,settle_steps=18)
        ctx.tool("robot.set_grip",preset="open",arm_id=1,settle_steps=10)
        rel["clear_side"]=_go(ctx,1,np.asarray(tcp[:2])+np.array([.07,.015]),.025,rot,steps=250)
        rel["rise"]=_go(ctx,1,np.asarray(tcp[:2])+np.array([.07,.015]),.12,rot,steps=300)
    report["tip_release"]=rel
    state("after_tip_release")
    # Lower helper only after tip release; measure crossing loss and drift.
    hrel=skills.place(ctx,0,axy,ar,s=anchor_s,height=.075)
    report["helper_release"]=hrel
    state("after_helper_release")
    for arm in (0,1):
        pos,r=skills.ee(ctx,arm)
        if pos is not None:
            ctx.tool("robot.set_grip",preset="open",arm_id=arm,settle_steps=8)
            _go(ctx,arm,pos[:2],.20,r,steps=300)
    ctx.tool("robot.wait_steps",steps=int(p.get("release_wait_steps",360)))
    state("after_release_3s")
    return {"route":"done","report":report}
