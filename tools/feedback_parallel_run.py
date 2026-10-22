"""Benchmark RSS/GaP rollout performance on two Daytona shapes, then paired candidates.

This tool is deliberately independent from the learning/evaluation tools. It keeps one
frozen graph bundle per run, labels sandboxes with unique pools, and always deletes them.
It never prints credentials or touches sandboxes outside this project.
"""
from __future__ import annotations
import argparse, dataclasses, hashlib, json, shutil, time, uuid
from pathlib import Path
import sys
from concurrent.futures import ThreadPoolExecutor
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from fleet.pool import Pool
from fleet.common import describe
from harness import rehearse, variants
from knotlab import task


def digest(path: Path):
 h=hashlib.sha256()
 for p in sorted(path.rglob('*')):
  if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':
   h.update(p.relative_to(path).as_posix().encode()+b'\0'); h.update(p.read_bytes())
 return h.hexdigest()

def run_one(graph, out, instances, cpu, memory, workers, max_steps):
 name='feedback-'+uuid.uuid4().hex[:12]
 pool=Pool(name,cpu=cpu,memory=memory,disk=30)
 t=time.time(); rows=[]; error=None
 try:
  rows=rehearse.run_daytona_many([(graph,out)],instances,max_steps=max_steps,workers=workers,pool=pool)
 except Exception as exc:
  error=describe(exc)
 finally:
  left=pool.down()
 return {'pool':name,'cpu':cpu,'memory':memory,'workers':workers,'wall_s':round(time.time()-t,2),
         'jobs':rows,'error':error,'remaining_sandboxes':left}

def audit(out, graph, instances):
 rows=[]
 for v in instances:
  ep=out/f'ep{v.seed:06d}'; r={'seed':v.seed,'episode':str(ep),'runtime_success':False,'audit_errors':[]}
  try:
   x=json.loads((ep/'result.json').read_text()); r.update({k:x.get(k) for k in ('success','sim_time_s','wall_s','grader_version')}); r['runtime_success']=bool(x.get('success'))
   if x.get('grader_version')!=task.GRADER_VERSION:r['audit_errors'].append('grader_version')
   if x.get('variant')!=dataclasses.asdict(v):r['audit_errors'].append('variant')
   import numpy as np
   n=np.load(ep/'rope_final.npy',allow_pickle=False)
   r['geometry_shape']=list(n.shape); r['geometry_finite']=bool(np.isfinite(n).all())
   if n.ndim!=2 or n.shape[1]!=3 or len(n)<4 or not r['geometry_finite']:r['audit_errors'].append('invalid final geometry')
   r['error']=x.get('error','')
   if r['error']:r['audit_errors'].append('episode exception')
  except Exception as e:r['audit_errors'].append(f'{type(e).__name__}: {e}')
  rows.append(r)
 return rows

def main():
 ap=argparse.ArgumentParser(description=__doc__)
 ap.add_argument('--out',type=Path,required=True); ap.add_argument('--graph',action='append',default=[])
 ap.add_argument('--n',type=int,default=1); ap.add_argument('--offset',type=int,default=0)
 ap.add_argument('--max-steps',type=int,default=1200); ap.add_argument('--workers',type=int,default=2)
 ap.add_argument('--benchmark-only',action='store_true'); ap.add_argument('--skip-benchmark',action='store_true')
 ap.add_argument('--variant-json',default='',help='exact Variant JSON; otherwise nominal overhand')
 args=ap.parse_args()
 if not 1<=args.workers<=6 or args.n<1 or args.offset<0 or args.max_steps<1: ap.error('n/max-steps positive; workers 1..6')
 out=args.out.absolute(); out.mkdir(parents=True,exist_ok=True)
 if (out/'summary.json').exists():ap.error('completed output already exists; use a fresh --out')
 graph=ROOT/'graphs/grasp_probe' if not args.graph else ROOT/args.graph[0]
 if not graph.is_dir(): ap.error(f'missing graph {graph}')
 v=task.Variant.from_json(args.variant_json) if args.variant_json else variants.nominal('overhand')
 instances=[dataclasses.replace(v,seed=args.offset+i) for i in range(args.n)]
 sources=[ROOT/g for g in args.graph] if args.graph else [graph]
 if len(sources)>6:ap.error('at most six candidate graphs')
 if len({g.name for g in sources})!=len(sources):ap.error('candidate graph directory names must be unique')
 for g in sources:
  if not g.is_dir():ap.error(f'missing candidate graph {g}')
 frozen=[]
 for i,g in enumerate(sources):
  dst=out/'graphs'/g.name
  if dst.exists():
   if digest(dst)!=digest(g):ap.error('frozen graph differs; use a fresh --out')
  else:shutil.copytree(g,dst,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
  frozen.append(dst)
 graph=frozen[0]
 manifest={'graphs':{g.name:digest(g) for g in frozen},'instances':[dataclasses.asdict(x) for x in instances],
           'max_steps':args.max_steps,'workers':args.workers,'grader_version':task.GRADER_VERSION,
           'benchmark_shapes':[{'cpu':2,'memory':4,'disk':30},{'cpu':8,'memory':16,'disk':30}],
           'candidate_shape':{'cpu':8,'memory':16,'disk':30},'benchmark_parallelism':2,
           'scope':'RSS/GaP runtime benchmark; no tying claim'}
 if (out/'manifest.json').exists() and json.loads((out/'manifest.json').read_text())!=manifest:ap.error('manifest differs; use fresh --out')
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 allruns=[]
 if not args.skip_benchmark:
  def benchmark(shape):
   label,cpu,mem=shape
   run=run_one(graph,out/'benchmark'/label/'eval',instances,cpu,mem,1,args.max_steps); run['label']=label
   (out/'benchmark'/f'{label}.json').write_text(json.dumps(run,indent=2)+'\n')
   (out/'benchmark'/f'{label}_audit.json').write_text(json.dumps(audit(out/'benchmark'/label/'eval',graph,instances),indent=2)+'\n')
   return run
  with ThreadPoolExecutor(max_workers=2) as executor:
   allruns.extend(executor.map(benchmark,[('cpu2',2,4),('cpu8',8,16)]))
 if not args.benchmark_only:
  graphs=frozen if args.graph else []
  if len(graphs)>6: ap.error('at most six candidate graphs')
  if graphs:
   for i,g in enumerate(graphs):
    if not g.is_dir():ap.error(f'missing candidate graph {g}')
   specs=[(g,out/'candidates'/g.name) for g in graphs]
   name='feedback-'+uuid.uuid4().hex[:12]; pool=Pool(name,cpu=8,memory=16,disk=30); t=time.time(); err=None; jobs=[]
   try: jobs=rehearse.run_daytona_many(specs,instances,max_steps=args.max_steps,workers=min(args.workers,6),pool=pool)
   except Exception as exc: err=describe(exc)
   finally: left=pool.down()
   run={'pool':name,'cpu':8,'memory':16,'workers':min(args.workers,6),'wall_s':round(time.time()-t,2),'jobs':jobs,'error':err,'remaining_sandboxes':left}
   allruns.append({'label':'candidates',**run}); (out/'candidates/run.json').write_text(json.dumps(run,indent=2)+'\n')
   audits={g.name:audit(out/'candidates'/g.name,g,instances) for g in graphs}; (out/'candidates/audit.json').write_text(json.dumps(audits,indent=2)+'\n')
 summary={'runs':allruns,'benchmark':{r['label']:{'wall_s':r['wall_s'],'jobs':len(r['jobs']),'error':r['error']} for r in allruns if r['label'] in ('cpu2','cpu8')},'cleanup_verified':all(r['remaining_sandboxes']==0 for r in allruns)}
 bench=[r for r in allruns if r['label'] in ('cpu2','cpu8')]
 if len(bench)==2 and all(r['jobs'] and all(j['status']=='ok' for j in r['jobs']) for r in bench):
  secs={r['label']:sum(j['run_s'] for j in r['jobs'])/len(r['jobs']) for r in bench}
  summary['performance']={'mean_job_run_s':secs,'cpu8_speedup_vs_cpu2':secs['cpu2']/secs['cpu8'],
                          'note':'Short grasp-only task; includes startup and IK; not a full knot-tying throughput guarantee.'}
 (out/'cleanup.json').write_text(json.dumps([{'pool':r['pool'],'remaining_sandboxes':r['remaining_sandboxes']} for r in allruns],indent=2)+'\n')
 (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n'); print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
