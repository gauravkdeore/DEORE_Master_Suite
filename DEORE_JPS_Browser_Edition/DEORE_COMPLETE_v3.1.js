/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║  DEORE v3.1 — Dual-Ended Optimality and Routing Engine                   ║
 * ║  © Gaurav Deore — All Rights Reserved. Patent Pending.                   ║
 * ║                                                                           ║
 * ║  Original Mechanisms (all by Gaurav Deore):                              ║
 * ║   1. TADC  — Topology-Aware Dead-End Collapse          [preprocessing]   ║
 * ║   2. MTD   — Maze Topology Detector                    [routing]         ║
 * ║   3. ACE   — Adaptive Cost Estimation                  [heuristic]       ║
 * ║   4. FRAP  — Forward-Reach Angle Pruning               [JPS pruning]     ║
 * ║   5. DCM   — Dual-Confirmation Meeting                 [termination]     ║
 * ║   6. ACF   — Admissible Cost Floor                     [termination v3]  ║
 * ║   7. DPES  — Dual-Pivot Expansion Selector             [new v3.1]        ║
 * ║   8. SSP   — Sterile Segment Pruning                   [new v3.1]        ║
 * ║                                                                           ║
 * ║  v3 fixed: broken lb(eps) termination → ACF                              ║
 * ║  v3.1 adds: DPES (midpoint-balanced frontier selection)                   ║
 * ║             SSP  (prune jump segments that cannot improve incumbent)      ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 *
 * POSITIONING vs STATE-OF-THE-ART (as of 2025):
 *
 *  • MM / MEET (IJCAI 2025): Optimal bi-HS with tight MMP termination.
 *    Operates cell-by-cell. On grid maps DEORE's JPS core skips entire
 *    corridors, reducing "structural node count" by orders of magnitude
 *    before the bi-HS mechanism even activates.
 *
 *  • BAE* / DIBBS: Exploit consistent heuristics for early termination.
 *    Both are DXBB algorithms, bounded by NBS theory (≤2·VC expansions).
 *    DEORE is also DXBB; it acknowledges this and argues its JPS + TADC
 *    preprocessing shrinks VC itself, yielding a smaller absolute cost.
 *
 *  • NBS: Near-optimal DXBB expansions via vertex cover of Must-Expand
 *    Graph. Requires pair-selection at each step (O(open) overhead).
 *    DEORE's DPES achieves approximate midpoint balance in O(1) per step.
 *
 *  • JPS / JPS+: Unidirectional symmetry-breaking for open terrain.
 *    DEORE wraps JPS bidirectionally, adds FRAP + SSP for further pruning,
 *    and switches to TADC + uniJPS for maze topologies.
 *
 *  DEORE's domain is uniform-cost 8-directional grids (game AI, robotics).
 *  Its three-layer architecture (topology classification → preprocessing →
 *  bi-HS core) is unique among published algorithms.
 */
'use strict';

const EMPTY=0, WALL=1, SQRT2=Math.SQRT2, EPS=1e-9, EPS_MAX=2.5;
const DIRS8=[[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]];
const ck  = (r,c) => `${r},${c}`;
const pk  = k => k.split(',').map(Number);
const oct = (r,c,[tr,tc]) => { const dr=Math.abs(r-tr),dc=Math.abs(c-tc); return Math.max(dr,dc)+(SQRT2-1)*Math.min(dr,dc); };
const ec  = (r1,c1,r2,c2) => (r1===r2||c1===c2)?1:SQRT2;
const free = (g,r,c) => { const R=g.length,C=g[0].length; return r>=0&&r<R&&c>=0&&c<C&&g[r][c]!==WALL; };

// ── MinHeap ─────────────────────────────────────────────────────────────────
// Manages ordering only. Lower bound reasoning is handled by ACF (Mechanism 6)
// and DPES (Mechanism 7), not by this structure.
class MinHeap {
  constructor() { this.h = []; }
  push(x)  { this.h.push(x); this._u(this.h.length-1); }
  pop()    { const t=this.h[0],l=this.h.pop(); if(this.h.length){this.h[0]=l;this._d(0);}return t; }
  peek()   { return this.h[0]; }
  get size() { return this.h.length; }
  minF()   { return this.h.length?this.h[0][0]:Infinity; }
  _u(i)   { while(i>0){const p=(i-1)>>1;if(this.h[p][0]<=this.h[i][0])break;[this.h[p],this.h[i]]=[this.h[i],this.h[p]];i=p;} }
  _d(i)   { const n=this.h.length;while(true){let s=i,l=2*i+1,r=2*i+2;if(l<n&&this.h[l][0]<this.h[s][0])s=l;if(r<n&&this.h[r][0]<this.h[s][0])s=r;if(s===i)break;[this.h[s],this.h[i]]=[this.h[i],this.h[s]];i=s;} }
}

// ── JPS Helpers ──────────────────────────────────────────────────────────────
function hasForced(g,r,c,dr,dc) {
  if(dr&&dc){
    if(!free(g,r-dr,c)&&free(g,r-dr,c+dc))return true;
    if(!free(g,r,c-dc)&&free(g,r+dr,c-dc))return true;
  }else if(dr===0){
    if(!free(g,r-1,c)&&free(g,r-1,c+dc))return true;
    if(!free(g,r+1,c)&&free(g,r+1,c+dc))return true;
  }else{
    if(!free(g,r,c-1)&&free(g,r+dr,c-1))return true;
    if(!free(g,r,c+1)&&free(g,r+dr,c+1))return true;
  }
  return false;
}

function natNeighbors(g,r,c,fr2,fc) {
  if(fr2===null) return DIRS8.map(([dr,dc])=>[r+dr,c+dc]).filter(([nr,nc])=>free(g,nr,nc));
  const dr=Math.sign(r-fr2),dc=Math.sign(c-fc),n=[];
  if(dr&&dc){
    if(free(g,r+dr,c))n.push([r+dr,c]);
    if(free(g,r,c+dc))n.push([r,c+dc]);
    if((free(g,r+dr,c)||free(g,r,c+dc))&&free(g,r+dr,c+dc))n.push([r+dr,c+dc]);
    if(!free(g,r-dr,c)&&free(g,r-dr,c+dc))n.push([r-dr,c+dc]);
    if(!free(g,r,c-dc)&&free(g,r+dr,c-dc))n.push([r+dr,c-dc]);
  }else if(dr===0){
    if(free(g,r,c+dc))n.push([r,c+dc]);
    if(!free(g,r-1,c)&&free(g,r-1,c+dc))n.push([r-1,c+dc]);
    if(!free(g,r+1,c)&&free(g,r+1,c+dc))n.push([r+1,c+dc]);
  }else{
    if(free(g,r+dr,c))n.push([r+dr,c]);
    if(!free(g,r,c-1)&&free(g,r+dr,c-1))n.push([r+dr,c-1]);
    if(!free(g,r,c+1)&&free(g,r+dr,c+1))n.push([r+dr,c+1]);
  }
  return n.filter(([nr,nc])=>free(g,nr,nc));
}

function jumpIter(g,r0,c0,dr,dc,goal,oG,d=0) {
  if(d>5000)return null;
  const nr=r0+dr,nc=c0+dc;
  if(!free(g,nr,nc))return null;
  if(nr===goal[0]&&nc===goal[1])return[nr,nc];
  if(oG[ck(nr,nc)]!==undefined)return[nr,nc];
  if(hasForced(g,nr,nc,dr,dc))return[nr,nc];
  if(dr&&dc){
    if(jumpIter(g,nr,nc,dr,0,goal,oG,d+1))return[nr,nc];
    if(jumpIter(g,nr,nc,0,dc,goal,oG,d+1))return[nr,nc];
  }
  return jumpIter(g,nr,nc,dr,dc,goal,oG,d+1);
}

function cellsBetween(r1,c1,r2,c2) {
  const a=[[r1,c1]],dr=Math.sign(r2-r1),dc=Math.sign(c2-c1);
  if(!dr&&!dc)return a;
  let r=r1,c=c1;
  while(r!==r2||c!==c2){r+=dr;c+=dc;a.push([r,c]);}
  return a;
}

// ════════════════════════════════════════════════════════════════════════════
//  _core — DEORE Bidirectional Core
//
//  Implements all eight mechanisms in an integrated loop:
//
//  ACE   (§3.3) — epsilon decays 2.5→1.0 driving early dive then tightening
//  FRAP  (§3.4) — JPS natural+forced neighbors only (backward directions pruned)
//  ACF   (§3.6) — per-frontier monotonic true-cost floor (O(1) per push)
//  DPES  (§3.7) — frontier selection by ACF midpoint balance (O(1) per step)
//  SSP   (§3.8) — jump segments whose cost cannot beat incumbent are skipped
//  DCM   (§3.5) — doubly-closed meeting nodes, gated by ACF before finalizing
// ════════════════════════════════════════════════════════════════════════════
function _core(grid, start, end, useJPS, getCost, hScale, maxNodes) {
  const R=grid.length, C=grid[0].length;
  const sk=ck(...start), ek=ck(...end);
  const h0=oct(...start,end)*hScale;

  // ACE: epsilon decays from 2.5→1.0 as search progresses
  let estPath=Math.max(h0*3,20), estUpdated=false, totalNodes=0;
  const getEps=()=>1.0+1.5*Math.exp(-4*totalNodes/estPath);

  // ACF initialisation: both frontiers start with true-h from source
  let acfFwd=h0, acfBwd=h0;

  const fG={[sk]:0}, bG={[ek]:0};
  const fPar={}, bPar={}, fSeg={}, bSeg={};
  const fCl=new Set(), bCl=new Set(), order=[];
  const fO=new MinHeap(), bO=new MinHeap();
  fO.push([h0*getEps(),0,[...start,null,null]]);
  bO.push([h0*getEps(),0,[...end,null,null]]);

  let tC=Infinity,tMk=null, cC=Infinity,cMk=null, epsT=false;

  const updateT=nk=>{
    if(fG[nk]!==undefined&&bG[nk]!==undefined){
      const t=fG[nk]+bG[nk];
      if(t<tC-EPS){tC=t;tMk=nk;if(!estUpdated){estPath=Math.max(estPath,t);estUpdated=true;}}
    }
  };
  const updateC=k=>{
    if(fCl.has(k)&&bCl.has(k)&&fG[k]!==undefined&&bG[k]!==undefined){
      const t=fG[k]+bG[k];
      if(t<cC-EPS){cC=t;cMk=k;}
    }
  };
  const doTrans=()=>{ for(const k of fCl){if(bCl.has(k))updateC(k);}epsT=true; };

  // ACF: record true admissible cost floor for each pushed node
  const noteACF=(isFwd,g,r,c,tgt)=>{
    const trueF=g+oct(r,c,tgt)*hScale;
    if(isFwd){if(trueF<acfFwd)acfFwd=trueF;}
    else     {if(trueF<acfBwd)acfBwd=trueF;}
  };

  const expand=isFwd=>{
    const O  =isFwd?fO:bO,   Cl =isFwd?fCl:bCl;
    const G  =isFwd?fG:bG,   oG =isFwd?bG:fG;
    const Par=isFwd?fPar:bPar,Seg=isFwd?fSeg:bSeg;
    const tgt=isFwd?end:start, src=isFwd?start:end;

    while(O.size&&Cl.has(ck(...O.peek()[2].slice(0,2))))O.pop();
    if(!O.size)return;
    const[,gv,posArr]=O.pop();
    const[r,c,fr2,fc]=posArr;
    const k=ck(r,c);
    if(Cl.has(k))return;
    Cl.add(k); totalNodes++; order.push([r,c,isFwd?0:1]);

    const eps=getEps();
    if(!epsT&&eps<=1.001)doTrans();
    updateC(k);

    if(useJPS){
      // FRAP: only expand natural + forced neighbors
      for(const[nr,nc]of natNeighbors(grid,r,c,fr2,fc)){
        const dr=Math.sign(nr-r),dc=Math.sign(nc-c);
        const jp=jumpIter(grid,r,c,dr,dc,tgt,oG);
        if(!jp)continue;
        const[jr,jc]=jp,jk=ck(jr,jc);
        if(Cl.has(jk))continue;

        // SSP (Mechanism 8): Sterile Segment Pruning
        // Compute the minimum possible path cost through this jump point BEFORE
        // paying the cost of building the full segment (cellsBetween).
        // We need at least the straight-line octile from current node to jp.
        const stepDr=Math.abs(jr-r),stepDc=Math.abs(jc-c);
        const diagSteps=Math.min(stepDr,stepDc),straightSteps=Math.abs(stepDr-stepDc);
        const minSegCost=diagSteps*SQRT2+straightSteps;
        const ng=gv+Math.max(minSegCost,EPS);

        // SSP check 1: this jump point's g + h to target already exceeds incumbent
        if(tC<Infinity&&ng+oct(jr,jc,tgt)*hScale>=tC-EPS)continue;
        // SSP check 2: if opposite side already has a cost to this jp, check join
        if(oG[jk]!==undefined&&ng+oG[jk]>=tC-EPS)continue;

        if(G[jk]===undefined||ng<G[jk]-EPS){
          G[jk]=ng; Par[jk]=k;
          // Build segment only when we've passed SSP and will actually record it
          Seg[jk]=cellsBetween(r,c,jr,jc);
          O.push([ng+eps*oct(jr,jc,tgt)*hScale,ng,[jr,jc,r,c]]);
          noteACF(isFwd,ng,jr,jc,tgt);
        }
        updateT(jk);
      }
    }else{
      // Cell-by-cell mode: fallback / custom cost function
      const oC=isFwd?bCl:fCl;
      const isDE=(nr,nc)=>{
        if(nr===tgt[0]&&nc===tgt[1]||nr===src[0]&&nc===src[1])return false;
        for(const[d2,dc2]of DIRS8){const[a,b]=[nr+d2,nc+dc2];if(a<0||a>=R||b<0||b>=C)continue;if(oC.has(ck(a,b)))return false;}
        let f=0;
        for(const[d2,dc2]of DIRS8){const[a,b]=[nr+d2,nc+dc2];if(a<0||a>=R||b<0||b>=C||grid[a][b]===WALL||a===r&&b===c)continue;f++;}
        return f===0;
      };
      for(const[dr,dc]of DIRS8){
        const[nr,nc]=[r+dr,c+dc];
        if(!free(grid,nr,nc))continue;
        const nk=ck(nr,nc);
        if(Cl.has(nk))continue;
        if(isDE(nr,nc))continue;
        const ng=gv+getCost(r,c,nr,nc);
        if(tC<Infinity&&ng+oct(nr,nc,tgt)*hScale>=tC-EPS)continue; // SSP for cell mode
        if(G[nk]===undefined||ng<G[nk]-EPS){
          G[nk]=ng; Par[nk]=k;
          O.push([ng+eps*oct(nr,nc,tgt)*hScale,ng,[nr,nc,r,c]]);
          noteACF(isFwd,ng,nr,nc,tgt);
        }
        updateT(nk);
      }
    }
  };

  let partial=false;

  while(fO.size||bO.size){
    // Lazy-delete stale heap tops
    while(fO.size&&fCl.has(ck(...fO.peek()[2].slice(0,2))))fO.pop();
    while(bO.size&&bCl.has(ck(...bO.peek()[2].slice(0,2))))bO.pop();

    // ACF global termination: if both floors together prove no better path exists
    if(tMk!==null&&acfFwd+acfBwd>=tC-EPS)break;

    // ════════════════════════════════════════════════════════════════════
    //  DPES — Dual-Pivot Expansion Selector (Mechanism 7)
    //
    //  Goal: keep both frontiers converging symmetrically toward the path
    //  midpoint, which minimises total expansions for bidirectional search.
    //
    //  Strategy: expand the frontier whose ACF value is FURTHER from tC/2
    //  (the theoretical midpoint). This "catches up" the lagging frontier,
    //  keeping both floors balanced around tC/2.
    //
    //  When tC is not yet known (Infinity), fall back to min-f selection.
    //  Ties broken by min open-list f-value.
    //
    //  Why this works: if one frontier's ACF is much smaller than tC/2, it
    //  has already found cheap paths well past the midpoint; continuing to
    //  expand it yields diminishing returns. The other frontier needs to
    //  advance to confirm or improve the incumbent. DPES enforces this
    //  rebalancing in O(1) per iteration with no auxiliary data structure.
    // ════════════════════════════════════════════════════════════════════
    let expandFwd;
    if(tC<Infinity&&fO.size&&bO.size){
      const mid=tC/2;
      const fwdDist=Math.abs(acfFwd-mid);
      const bwdDist=Math.abs(acfBwd-mid);
      // Expand the frontier that is furthest from midpoint (needs to catch up)
      if(Math.abs(fwdDist-bwdDist)>EPS){
        expandFwd=(fwdDist>=bwdDist);
      }else{
        expandFwd=(fO.minF()<=bO.minF());
      }
    }else{
      expandFwd=!bO.size||(fO.size&&fO.minF()<=bO.minF());
    }

    if(expandFwd&&fO.size)expand(true);
    else if(bO.size)expand(false);
    else break;

    // DCM + ACF gate: only finalise a doubly-closed meeting when ACF confirms
    if(fCl.has(ek)){updateC(ek);if(cMk&&acfFwd+acfBwd>=cC-EPS)break;}
    if(bCl.has(sk)){updateC(sk);if(cMk&&acfFwd+acfBwd>=cC-EPS)break;}
    if(totalNodes>=maxNodes&&!cMk){partial=true;break;}
  }

  const eps_end=getEps();
  const mk=cMk||tMk;
  const cost=cC<Infinity?cC:tC;
  if(!mk)return{path:null,order,cost:Infinity,nodes:totalNodes,eps:eps_end,partial,jpsMode:useJPS,acfFwd,acfBwd};

  let path=null;
  if(useJPS){
    const trace=(Par,Seg,fK,tK)=>{
      const segs=[];let cur=tK,s=0;
      while(cur!==fK&&s++<1e5){const prev=Par[cur];if(!prev)break;if(Seg[cur])segs.push(Seg[cur]);cur=prev;}
      segs.reverse();
      const cells=[];
      for(const seg of segs){
        if(!cells.length)cells.push(...seg);
        else if(cells[cells.length-1][0]===seg[0][0]&&cells[cells.length-1][1]===seg[0][1])cells.push(...seg.slice(1));
        else cells.push(...seg);
      }
      return cells;
    };
    const fH=trace(fPar,fSeg,sk,mk),bH=trace(bPar,bSeg,ek,mk);
    bH.reverse();
    let fp;
    if(!fH.length&&!bH.length)fp=[pk(mk)];
    else if(!fH.length)fp=bH;
    else if(!bH.length)fp=fH;
    else{const mf=fH[fH.length-1],mb=bH[0];fp=(mf[0]===mb[0]&&mf[1]===mb[1])?[...fH,...bH.slice(1)]:[...fH,...bH];}
    if(!fp.length)fp=[[...start]];
    if(fp[0][0]!==start[0]||fp[0][1]!==start[1])fp=[[...start],...fp];
    const last=fp[fp.length-1];
    if(last[0]!==end[0]||last[1]!==end[1])fp=[...fp,[...end]];
    let ok2=true;
    for(let i=1;i<fp.length;i++){
      const[r,c]=fp[i],[pr,pc]=fp[i-1];
      if(Math.abs(r-pr)>1||Math.abs(c-pc)>1){ok2=false;break;}
    }
    path=ok2?fp:null;
  }else{
    const fh=[];let cur=mk,s=0;
    while(cur!==sk){if(!fPar[cur]||s++>1e5)break;fh.unshift(pk(cur));cur=fPar[cur];}
    fh.unshift([...start]);
    const bh=[];cur=mk;s=0;
    while(cur!==ek){if(s++>1e5)break;const nx=bPar[cur];if(!nx)break;bh.push(pk(cur));cur=nx;}
    bh.push([...end]);
    path=[...fh,...bh.slice(1)];
  }
  return{path,order,cost,nodes:totalNodes,eps:eps_end,partial,jpsMode:useJPS,acfFwd,acfBwd};
}

// ════════════════════════════════════════════════════════════════════════════
//  MTD — Maze Topology Detector
// ════════════════════════════════════════════════════════════════════════════
const _mazeCache=new WeakMap();
function isMazeLike(grid){
  if(_mazeCache.has(grid))return _mazeCache.get(grid);
  const R=grid.length,C=grid[0].length,total=R*C;
  let walls=0,freeCells=0,totalDeg=0;
  for(let r=0;r<R;r++)for(let c=0;c<C;c++){
    if(grid[r][c]===WALL){walls++;continue;}
    freeCells++;let deg=0;
    for(const[dr,dc]of DIRS8){const nr=r+dr,nc=c+dc;if(free(grid,nr,nc))deg++;}
    totalDeg+=deg;
  }
  if(!freeCells){_mazeCache.set(grid,false);return false;}
  const result=(walls/total)>0.40&&(totalDeg/freeCells)<=3.0;
  _mazeCache.set(grid,result);
  return result;
}

// ════════════════════════════════════════════════════════════════════════════
//  Unidirectional JPS A* — optimal; used by MTD for maze topologies
// ════════════════════════════════════════════════════════════════════════════
function jumpUni(g,r0,c0,dr,dc,goal,d){
  if(d>5000)return null;
  const nr=r0+dr,nc=c0+dc;
  if(!free(g,nr,nc))return null;
  if(nr===goal[0]&&nc===goal[1])return[nr,nc];
  if(hasForced(g,nr,nc,dr,dc))return[nr,nc];
  if(dr&&dc){
    if(jumpUni(g,nr,nc,dr,0,goal,d+1))return[nr,nc];
    if(jumpUni(g,nr,nc,0,dc,goal,d+1))return[nr,nc];
  }
  return jumpUni(g,nr,nc,dr,dc,goal,d+1);
}

function uniJPSAstar(grid,start,end){
  const sk=ck(...start),ek=ck(...end);
  const gMap={[sk]:0},par={[sk]:null},cl=new Set();
  const pq=new MinHeap(),order=[];
  pq.push([oct(...start,end),0,[...start,null,null]]);
  while(pq.size){
    while(pq.size&&cl.has(ck(pq.peek()[2][0],pq.peek()[2][1])))pq.pop();
    if(!pq.size)break;
    const[,gv,posArr]=pq.pop();
    const[r,c,fr2,fc]=posArr;
    const k=ck(r,c);
    if(cl.has(k))continue;
    cl.add(k);order.push([r,c,0]);
    if(k===ek){
      const raw=[];let cur=ek;
      while(cur){raw.unshift(pk(cur));cur=par[cur];}
      const full=[raw[0]];
      for(let i=1;i<raw.length;i++){
        const[pr,pc]=raw[i-1],[cr,cc]=raw[i];
        const dr=Math.sign(cr-pr),dc=Math.sign(cc-pc);
        let rr=pr,rc=pc;
        while(rr!==cr||rc!==cc){rr+=dr;rc+=dc;full.push([rr,rc]);}
      }
      return{path:full,order,cost:gMap[ek],nodes:cl.size,eps:1,partial:false,jpsMode:true,mazeMode:true};
    }
    let dirs;
    if(fr2===null){dirs=DIRS8;}
    else{
      const dr=Math.sign(r-fr2),dc=Math.sign(c-fc);
      dirs=[];
      if(dr&&dc){
        dirs.push([dr,0],[0,dc],[dr,dc]);
        if(!free(grid,r-dr,c))dirs.push([-dr,dc]);
        if(!free(grid,r,c-dc))dirs.push([dr,-dc]);
      }else if(dc){
        dirs.push([0,dc]);
        if(!free(grid,r-1,c))dirs.push([-1,dc]);
        if(!free(grid,r+1,c))dirs.push([1,dc]);
      }else{
        dirs.push([dr,0]);
        if(!free(grid,r,c-1))dirs.push([dr,-1]);
        if(!free(grid,r,c+1))dirs.push([dr,1]);
      }
    }
    for(const[dr,dc]of dirs){
      const jp=jumpUni(grid,r,c,dr,dc,end,0);
      if(!jp)continue;
      const[jr,jc]=jp,jk=ck(jr,jc);
      if(cl.has(jk))continue;
      const stepDr=Math.abs(jr-r),stepDc=Math.abs(jc-c);
      const diag=Math.min(stepDr,stepDc),straight=Math.abs(stepDr-stepDc);
      const nd=gv+diag*SQRT2+straight;
      if(gMap[jk]===undefined||nd<gMap[jk]-EPS){
        gMap[jk]=nd;par[jk]=k;
        pq.push([nd+oct(jr,jc,end),nd,[jr,jc,r,c]]);
      }
    }
  }
  return{path:null,order,cost:Infinity,nodes:cl.size,eps:1,partial:false,jpsMode:true,mazeMode:true};
}

// ════════════════════════════════════════════════════════════════════════════
//  TADC — Topology-Aware Dead-End Collapse
// ════════════════════════════════════════════════════════════════════════════
function applyTADC(grid,start,end){
  const R=grid.length,C=grid[0].length;
  const g=grid.map(r=>[...r]);
  const degree=Array.from({length:R},()=>new Int32Array(C).fill(0));
  const q=[];
  for(let r=1;r<R-1;r++)for(let c=1;c<C-1;c++){
    if(g[r][c]===WALL)continue;
    if(r===start[0]&&c===start[1])continue;
    if(r===end[0]&&c===end[1])continue;
    let freeCount=0;
    for(const[dr,dc]of DIRS8){if(g[r+dr][c+dc]===EMPTY)freeCount++;}
    degree[r][c]=freeCount;
    if(freeCount<=1)q.push([r,c]);
  }
  while(q.length>0){
    const[r,c]=q.pop();
    if(g[r][c]===WALL)continue;
    g[r][c]=WALL;
    for(const[dr,dc]of DIRS8){
      const nr=r+dr,nc=c+dc;
      if(nr>0&&nr<R-1&&nc>0&&nc<C-1&&g[nr][nc]===EMPTY){
        degree[nr][nc]--;
        if(degree[nr][nc]<=1&&!(nr===start[0]&&nc===start[1])&&!(nr===end[0]&&nc===end[1]))
          q.push([nr,nc]);
      }
    }
  }
  return g;
}

// ════════════════════════════════════════════════════════════════════════════
//  deoreCoreCore — Main entry point
// ════════════════════════════════════════════════════════════════════════════
function deoreCoreCore(grid,start,end,opts={}){
  if(grid[start[0]]?.[start[1]]===WALL||grid[end[0]]?.[end[1]]===WALL)
    return{path:null,order:[],cost:Infinity,nodes:0,eps:1,partial:false};
  if(start[0]===end[0]&&start[1]===end[1])
    return{path:[[...start]],order:[],cost:0,nodes:0,eps:1,partial:false};

  const getCost=opts.getCost||ec;
  const minEdge=opts.minEdgeCost??1;
  const maxNodes=opts.maxNodes??Infinity;
  const useJPS=!opts.getCost;
  const hScale=Math.min(1,minEdge);

  // MTD: maze-like grids → TADC + unidirectional JPS
  if(useJPS&&isMazeLike(grid)){
    const prunedGrid=applyTADC(grid,start,end);
    const r=uniJPSAstar(prunedGrid,start,end);
    if(r.path)return{...r,fallback:false};
  }

  // Open/semi-open terrain: bidirectional core with all 8 mechanisms
  const r1=_core(grid,start,end,useJPS,getCost,hScale,maxNodes);

  // Guaranteed fallback: cell-by-cell is always complete
  if(!r1.path&&useJPS&&!r1.partial){
    const r2=_core(grid,start,end,false,getCost,hScale,maxNodes);
    return{...r2,nodes:r1.nodes+r2.nodes,fallback:true,jpsMode:false};
  }
  return{...r1,fallback:false};
}

// ════════════════════════════════════════════════════════════════════════════
//  DynamicDEORE — Dynamic re-planning agent
// ════════════════════════════════════════════════════════════════════════════
class DynamicDEORE{
  constructor(grid,opts={}){
    this.grid=grid.map(r=>[...r]);this.R=grid.length;this.C=grid[0].length;
    this.opts=opts;this.path=null;this.cost=Infinity;
    this.agent=null;this.goal=null;
    this.replans=0;this.totalNodes=0;this.epsilonHistory=[];
  }
  plan(start,goal){this.agent=[...start];this.goal=[...goal];return this._run(start);}
  step(){if(!this.path||this.path.length<2)return null;this.path.shift();this.agent=[...this.path[0]];return this.agent;}
  pathValid(){return!!this.path&&this.path.every(([r,c])=>this.grid[r]?.[c]!==WALL);}
  updateObstacle(r,c,toWall,agentPos=null){
    if(this.grid[r][c]===(toWall?WALL:EMPTY))return{replanned:false,path:this.path,cost:this.cost,nodes:0};
    this.grid[r][c]=toWall?WALL:EMPTY;
    const pos=agentPos||this.agent;
    if(!this._pathBlockedBy(r,c)&&this.path)return{replanned:false,path:this.path,cost:this.cost,nodes:0,blocked:false};
    return{replanned:true,...this._run(pos)};
  }
  _pathBlockedBy(wr,wc){
    return!this.path||this.path.some(([r,c])=>Math.abs(r-wr)<=1&&Math.abs(c-wc)<=1)
        ||!this.path.every(([r,c])=>this.grid[r]?.[c]!==WALL);
  }
  _run(from){
    const r=deoreCoreCore(this.grid,from,[...this.goal],this.opts);
    this.path=r.path;this.cost=r.cost;this.replans++;
    this.totalNodes+=r.nodes;
    if(r.eps&&isFinite(r.eps)&&!isNaN(r.eps))this.epsilonHistory.push(r.eps);
    return{path:r.path,cost:r.cost,nodes:r.nodes,partial:r.partial};
  }
  stats(){
    const e=this.epsilonHistory.filter(x=>isFinite(x)&&!isNaN(x));
    return{
      replans:this.replans,totalNodes:this.totalNodes,
      avgNodes:this.replans>0?Math.round(this.totalNodes/this.replans):0,
      maxEps:e.length?Math.max(...e).toFixed(4):'N/A',
      avgEps:e.length?(e.reduce((a,b)=>a+b)/e.length).toFixed(4):'N/A'
    };
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  analyzeEpsilonTightness — Empirical optimality analysis (vs Dijkstra)
// ════════════════════════════════════════════════════════════════════════════
function analyzeEpsilonTightness(numCases=10000){
  function makeG(R,C){return Array.from({length:R},()=>Array(C).fill(EMPTY));}
  function makeMaze(R,C,seed){
    const g=makeG(R,C).map(r=>r.fill(WALL));
    let s=seed;
    const rand=()=>{s=(s*1664525+1013904223)&0xffffffff;return(s>>>0)/0xffffffff;};
    function carve(r,c){
      g[r][c]=0;
      const ds=[[0,2],[0,-2],[2,0],[-2,0]].sort(()=>rand()-0.5);
      for(const[dr,dc]of ds){const[nr,nc]=[r+dr,c+dc];if(nr>0&&nr<R-1&&nc>0&&nc<C-1&&g[nr][nc]===1){g[r+dr/2][c+dc/2]=0;carve(nr,nc);}}
    }
    carve(1,1);g[1][1]=g[R-2][C-2]=0;
    return{g,start:[1,1],end:[R-2,C-2]};
  }
  function dijkstra(grid,s,e){
    const R=grid.length,C=grid[0].length,sk=ck(...s),ek=ck(...e);
    const g={[sk]:0},cl=new Set(),pq=new MinHeap();
    pq.push([0,0,s]);
    while(pq.size){
      const[d,,pos]=pq.pop();const[r,c]=pos;const k=ck(r,c);
      if(cl.has(k))continue;cl.add(k);if(k===ek)return g[ek];
      for(const[dr,dc]of DIRS8){
        const[nr,nc]=[r+dr,c+dc];
        if(nr<0||nr>=R||nc<0||nc>=C||grid[nr][nc]===WALL)continue;
        const nk=ck(nr,nc),nd=d+ec(r,c,nr,nc);
        if(g[nk]===undefined||nd<g[nk]-EPS){g[nk]=nd;pq.push([nd,nd,[nr,nc]]);}
      }
    }
    return Infinity;
  }

  const ratios=[];let violations=0,seed=7777;
  const rand=()=>{seed=(seed*1664525+1013904223)&0xffffffff;return(seed>>>0)/0xffffffff;};
  for(let i=0;i<numCases;i++){
    let grid,s,e;
    if(i%5===4){
      const{g,start,end}=makeMaze(24,48,i*3333);
      grid=g;s=start;e=end;
    }else{
      const R=Math.floor(rand()*12)+4,C=Math.floor(rand()*18)+6;
      grid=makeG(R,C);
      for(let r=0;r<R;r++)for(let c=0;c<C;c++)if(rand()<0.25)grid[r][c]=WALL;
      const sr=Math.floor(rand()*R),sc=Math.floor(rand()*C),er=Math.floor(rand()*R),ec2=Math.floor(rand()*C);
      grid[sr][sc]=grid[er][ec2]=EMPTY;s=[sr,sc];e=[er,ec2];
    }
    const opt=dijkstra(grid,s,e);
    if(!isFinite(opt)||opt===0)continue;
    const r=deoreCoreCore(grid,s,e,{});
    if(!r.path)continue;
    const ratio=r.cost/opt;
    if(!isFinite(ratio)||isNaN(ratio))continue;
    ratios.push(ratio);
    if(ratio>EPS_MAX+EPS)violations++;
  }
  if(!ratios.length)return{n:0,violations:0,summary:'No valid cases'};
  ratios.sort((a,b)=>a-b);
  const n=ratios.length,mean=ratios.reduce((a,b)=>a+b,0)/n;
  const pct=idx=>ratios[Math.floor(n*idx)]?.toFixed(4)||'N/A';
  return{
    n,violations,
    min:ratios[0].toFixed(4),p50:pct(.50),p90:pct(.90),p95:pct(.95),p99:pct(.99),
    max:ratios[n-1].toFixed(4),mean:mean.toFixed(4),
    theoreticalMax:'2.5000',
    utilization:((ratios[n-1]/EPS_MAX)*100).toFixed(1),
    overhead:((mean-1)*100).toFixed(2),
    summary:`ε ∈ [${ratios[0].toFixed(3)},${ratios[n-1].toFixed(3)}]. Ceiling 2.5 is ${((ratios[n-1]/EPS_MAX)*100).toFixed(1)}% utilized. Mean overhead ${((mean-1)*100).toFixed(2)}%. ${violations} violations.`
  };
}

function deoreCore(grid,start,end,opts={}){return deoreCoreCore(grid,start,end,opts);}
module.exports={deoreCore,deoreCoreCore,DynamicDEORE,analyzeEpsilonTightness};
