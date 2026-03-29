/**
 * DEORE_FullComparison.js — Comprehensive 8-Algorithm Benchmark
 *
 * Algorithms: Dijkstra, A*, Greedy Best-First, BFS, Bidirectional A*,
 *             JPS (Jump Point Search), DEORE, DEORE
 *
 * Outputs JSON results for the HTML visualizer.
 *
 * Usage: node DEORE_FullComparison.js [--limit=N]
 */
'use strict';
const fs = require('fs');
const path = require('path');

const { deoreCore, deoreCoreCore } = require(path.resolve(__dirname, 'DEORE_COMPLETE_v3.1.js'));

// ═══════════════════════════════════════════════════════════════
//  Core Utilities
// ═══════════════════════════════════════════════════════════════
const EMPTY = 0, WALL = 1, SQRT2 = Math.SQRT2, EPS = 1e-6;
const DIRS8 = [[-1,0],[1,0],[0,-1],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]];
const DIRS4 = [[-1,0],[1,0],[0,-1],[0,1]];
const ck = (r,c) => `${r},${c}`;
const pk = k => k.split(',').map(Number);
const ec = (r1,c1,r2,c2)=>(r1===r2||c1===c2)?1:SQRT2;
const oct = (r,c,tr,tc)=>{const dr=Math.abs(r-tr),dc=Math.abs(c-tc);return Math.max(dr,dc)+(SQRT2-1)*Math.min(dr,dc);};
const free = (g,r,c) => r>=0&&r<g.length&&c>=0&&c<g[0].length&&g[r][c]!==WALL;

class MinHeap {
  constructor(){this.h=[];}push(x){this.h.push(x);this._u(this.h.length-1);}pop(){const t=this.h[0],l=this.h.pop();if(this.h.length){this.h[0]=l;this._d(0);}return t;}get size(){return this.h.length;}
  _u(i){while(i>0){const p=(i-1)>>1;if(this.h[p][0]<=this.h[i][0])break;[this.h[p],this.h[i]]=[this.h[i],this.h[p]];i=p;}}
  _d(i){const n=this.h.length;while(true){let s=i,l=2*i+1,r=2*i+2;if(l<n&&this.h[l][0]<this.h[s][0])s=l;if(r<n&&this.h[r][0]<this.h[s][0])s=r;if(s===i)break;[this.h[s],this.h[i]]=[this.h[i],this.h[s]];i=s;}}
}

function neighbors8(grid,r,c){
  const n=[];
  for(const[dr,dc]of DIRS8){
    const nr=r+dr,nc=c+dc;
    if(!free(grid,nr,nc))continue;
    if(dr!==0&&dc!==0){if(!free(grid,r+dr,c)||!free(grid,r,c+dc))continue;}
    n.push([nr,nc,ec(r,c,nr,nc)]);
  }
  return n;
}

function reconstructPath(par,endKey){
  const p=[];let k=endKey;
  while(k){p.unshift(pk(k));k=par[k];}
  return p;
}

function computePathCost(path){
  if(!path||path.length<2)return 0;
  let c=0;for(let i=1;i<path.length;i++)c+=ec(path[i-1][0],path[i-1][1],path[i][0],path[i][1]);
  return c;
}

// ═══════════════════════════════════════════════════════════════
//  1. Dijkstra
// ═══════════════════════════════════════════════════════════════
function dijkstra(grid,s,e){
  const sk=ck(...s),ek=ck(...e),g={[sk]:0},par={[sk]:null},cl=new Set(),pq=new MinHeap(),expanded=[];
  pq.push([0,s]);
  while(pq.size){
    const[d,pos]=pq.pop();const[r,c]=pos;const k=ck(r,c);
    if(cl.has(k))continue;cl.add(k);expanded.push([r,c]);
    if(k===ek)return{path:reconstructPath(par,ek),nodes:cl.size,expanded,cost:g[ek]};
    for(const[nr,nc,w]of neighbors8(grid,r,c)){
      const nk=ck(nr,nc);if(cl.has(nk))continue;
      const nd=d+w;
      if(g[nk]===undefined||nd<g[nk]-EPS){g[nk]=nd;par[nk]=k;pq.push([nd,[nr,nc]]);}
    }
  }
  return{path:null,nodes:cl.size,expanded,cost:Infinity};
}

// ═══════════════════════════════════════════════════════════════
//  2. A* (octile heuristic)
// ═══════════════════════════════════════════════════════════════
function astar(grid,s,e){
  const sk=ck(...s),ek=ck(...e),g={[sk]:0},par={[sk]:null},cl=new Set(),pq=new MinHeap(),expanded=[];
  pq.push([oct(...s,...e),s]);
  while(pq.size){
    const[,pos]=pq.pop();const[r,c]=pos;const k=ck(r,c);
    if(cl.has(k))continue;cl.add(k);expanded.push([r,c]);
    if(k===ek)return{path:reconstructPath(par,ek),nodes:cl.size,expanded,cost:g[ek]};
    for(const[nr,nc,w]of neighbors8(grid,r,c)){
      const nk=ck(nr,nc);if(cl.has(nk))continue;
      const nd=g[k]+w;
      if(g[nk]===undefined||nd<g[nk]-EPS){g[nk]=nd;par[nk]=k;pq.push([nd+oct(nr,nc,...e),[nr,nc]]);}
    }
  }
  return{path:null,nodes:cl.size,expanded,cost:Infinity};
}

// ═══════════════════════════════════════════════════════════════
//  3. Greedy Best-First Search
// ═══════════════════════════════════════════════════════════════
function greedyBFS(grid,s,e){
  const sk=ck(...s),ek=ck(...e),par={[sk]:null},cl=new Set(),pq=new MinHeap(),expanded=[];
  pq.push([oct(...s,...e),s]);
  while(pq.size){
    const[,pos]=pq.pop();const[r,c]=pos;const k=ck(r,c);
    if(cl.has(k))continue;cl.add(k);expanded.push([r,c]);
    if(k===ek){const p=reconstructPath(par,ek);return{path:p,nodes:cl.size,expanded,cost:computePathCost(p)};}
    for(const[nr,nc]of neighbors8(grid,r,c)){
      const nk=ck(nr,nc);if(cl.has(nk)||par[nk]!==undefined)continue;
      par[nk]=k;pq.push([oct(nr,nc,...e),[nr,nc]]);
    }
  }
  return{path:null,nodes:cl.size,expanded,cost:Infinity};
}

// ═══════════════════════════════════════════════════════════════
//  4. BFS (unweighted, 8-directional)
// ═══════════════════════════════════════════════════════════════
function bfs(grid,s,e){
  const sk=ck(...s),ek=ck(...e),par={[sk]:null},vis=new Set([sk]),q=[s],expanded=[];
  while(q.length){
    const[r,c]=q.shift();const k=ck(r,c);expanded.push([r,c]);
    if(k===ek){const p=reconstructPath(par,ek);return{path:p,nodes:vis.size,expanded,cost:computePathCost(p)};}
    for(const[nr,nc]of neighbors8(grid,r,c)){
      const nk=ck(nr,nc);if(vis.has(nk))continue;
      vis.add(nk);par[nk]=k;q.push([nr,nc]);
    }
  }
  return{path:null,nodes:vis.size,expanded,cost:Infinity};
}

// ═══════════════════════════════════════════════════════════════
//  5. Bidirectional A*
// ═══════════════════════════════════════════════════════════════
function biAstar(grid,s,e){
  const sk=ck(...s),ek=ck(...e);
  const fG={[sk]:0},bG={[ek]:0},fP={[sk]:null},bP={[ek]:null};
  const fCl=new Set(),bCl=new Set();
  const fQ=new MinHeap(),bQ=new MinHeap();
  const expanded=[];
  fQ.push([oct(...s,...e),s]);bQ.push([oct(...e,...s),e]);
  let best=Infinity,bestKey=null;
  while(fQ.size||bQ.size){
    // Forward step
    if(fQ.size){
      const[,pos]=fQ.pop();const[r,c]=pos;const k=ck(r,c);
      if(!fCl.has(k)){
        fCl.add(k);expanded.push([r,c]);
        if(bCl.has(k)){const t=fG[k]+bG[k];if(t<best){best=t;bestKey=k;}}
        for(const[nr,nc,w]of neighbors8(grid,r,c)){
          const nk=ck(nr,nc);if(fCl.has(nk))continue;
          const nd=fG[k]+w;
          if(fG[nk]===undefined||nd<fG[nk]-EPS){fG[nk]=nd;fP[nk]=k;fQ.push([nd+oct(nr,nc,...e),[nr,nc]]);}
        }
      }
    }
    // Backward step
    if(bQ.size){
      const[,pos]=bQ.pop();const[r,c]=pos;const k=ck(r,c);
      if(!bCl.has(k)){
        bCl.add(k);expanded.push([r,c]);
        if(fCl.has(k)){const t=fG[k]+bG[k];if(t<best){best=t;bestKey=k;}}
        for(const[nr,nc,w]of neighbors8(grid,r,c)){
          const nk=ck(nr,nc);if(bCl.has(nk))continue;
          const nd=bG[k]+w;
          if(bG[nk]===undefined||nd<bG[nk]-EPS){bG[nk]=nd;bP[nk]=k;bQ.push([nd+oct(nr,nc,...s),[nr,nc]]);}
        }
      }
    }
    // Termination
    if(bestKey&&fQ.size&&bQ.size){
      const lb=fQ.h[0][0]+bQ.h[0][0]-oct(...s,...e);
      if(lb>=best-EPS)break;
    }
    if(fCl.size+bCl.size>grid.length*grid[0].length*2)break;
  }
  if(!bestKey||!isFinite(best))return{path:null,nodes:fCl.size+bCl.size,expanded,cost:Infinity};
  const fwd=[];let k=bestKey;while(k){fwd.unshift(pk(k));k=fP[k];}
  k=bP[bestKey];while(k){fwd.push(pk(k));k=bP[k];}
  return{path:fwd,nodes:fCl.size+bCl.size,expanded,cost:computePathCost(fwd)};
}

// ═══════════════════════════════════════════════════════════════
//  6. JPS (Jump Point Search)
// ═══════════════════════════════════════════════════════════════
function jps(grid,s,e){
  const R=grid.length,C=grid[0].length;
  function hasForced(r,c,dr,dc){
    if(dr&&dc){
      if(!free(grid,r-dr,c)&&free(grid,r-dr,c+dc))return true;
      if(!free(grid,r,c-dc)&&free(grid,r+dr,c-dc))return true;
    }else if(dr===0){
      if(!free(grid,r-1,c)&&free(grid,r-1,c+dc))return true;
      if(!free(grid,r+1,c)&&free(grid,r+1,c+dc))return true;
    }else{
      if(!free(grid,r,c-1)&&free(grid,r+dr,c-1))return true;
      if(!free(grid,r,c+1)&&free(grid,r+dr,c+1))return true;
    }
    return false;
  }
  function jump(r,c,dr,dc,depth){
    if(depth>5000)return null;
    const nr=r+dr,nc=c+dc;
    if(!free(grid,nr,nc))return null;
    if(nr===e[0]&&nc===e[1])return[nr,nc];
    if(hasForced(nr,nc,dr,dc))return[nr,nc];
    if(dr&&dc){
      if(jump(nr,nc,dr,0,depth+1))return[nr,nc];
      if(jump(nr,nc,0,dc,depth+1))return[nr,nc];
    }
    return jump(nr,nc,dr,dc,depth+1);
  }
  
  const sk=ck(...s),ek=ck(...e),g={[sk]:0},par={[sk]:null},cl=new Set(),pq=new MinHeap(),expanded=[];
  pq.push([oct(...s,...e),s,null]);
  while(pq.size){
    const[,pos,from]=pq.pop();const[r,c]=pos;const k=ck(r,c);
    if(cl.has(k))continue;cl.add(k);expanded.push([r,c]);
    if(k===ek){
      // Reconstruct with intermediate cells
      const rawPath=reconstructPath(par,ek);
      const fullPath=[rawPath[0]];
      for(let i=1;i<rawPath.length;i++){
        const[pr,pc]=rawPath[i-1],[cr,cc]=rawPath[i];
        const dr=Math.sign(cr-pr),dc=Math.sign(cc-pc);
        let rr=pr,rc=pc;
        while(rr!==cr||rc!==cc){rr+=dr;rc+=dc;fullPath.push([rr,rc]);}
      }
      return{path:fullPath,nodes:cl.size,expanded,cost:g[ek]};
    }
    // Identify successors
    let dirs;
    if(!from){dirs=DIRS8;}
    else{
      const dr=Math.sign(r-from[0]),dc=Math.sign(c-from[1]);
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
      const jp=jump(r,c,dr,dc,0);
      if(!jp)continue;
      const[jr,jc]=jp;const jk=ck(jr,jc);if(cl.has(jk))continue;
      const dist=Math.max(Math.abs(jr-r),Math.abs(jc-c))*((dr&&dc)?SQRT2:(dr?1:1));
      // Proper distance calc
      const stepDr=Math.abs(jr-r),stepDc=Math.abs(jc-c);
      const diagSteps=Math.min(stepDr,stepDc),straightSteps=Math.abs(stepDr-stepDc);
      const nd=g[k]+diagSteps*SQRT2+straightSteps;
      if(g[jk]===undefined||nd<g[jk]-EPS){
        g[jk]=nd;par[jk]=k;pq.push([nd+oct(jr,jc,...e),[jr,jc],[r,c]]);
      }
    }
  }
  return{path:null,nodes:cl.size,expanded,cost:Infinity};
}

// ═══════════════════════════════════════════════════════════════
//  7 & 8. DEORE and DEORE wrappers
// ═══════════════════════════════════════════════════════════════
function runDEORE(grid,s,e){
  const t0=performance.now();
  const res=deore(grid,s,e);
  const ms=performance.now()-t0;
  return{path:res.path,nodes:res.nodes,expanded:[],cost:res.path?computePathCost(res.path):Infinity,ms};
}
function runDEOREJ(grid,s,e){
  const t0=performance.now();
  const res=deoreCoreCore(grid,s,e,{});
  const ms=performance.now()-t0;
  return{path:res.path,nodes:res.nodes,expanded:[],cost:res.path?computePathCost(res.path):Infinity,ms};
}

// ═══════════════════════════════════════════════════════════════
//  Map generators
// ═══════════════════════════════════════════════════════════════
function makeGrid(R,C,fill=EMPTY){return Array.from({length:R},()=>Array(C).fill(fill));}
function addWalls(g,pct){for(let r=1;r<g.length-1;r++)for(let c=1;c<g[0].length-1;c++)if(Math.random()<pct)g[r][c]=WALL;}
function makeOpen(R,C){return makeGrid(R,C);}
function makeRandom(R,C,pct){const g=makeGrid(R,C);addWalls(g,pct);return g;}
function makeMaze(R,C){
  const g=makeGrid(R,C,WALL);
  let seed=42;
  const rand=()=>{seed=(seed*1664525+1013904223)&0xffffffff;return(seed>>>0)/0xffffffff;};
  function carve(r,c){
    g[r][c]=EMPTY;
    const dirs=[[0,2],[0,-2],[2,0],[-2,0]].sort(()=>rand()-0.5);
    for(const[dr,dc]of dirs){
      const nr=r+dr,nc=c+dc;
      if(nr>0&&nr<R-1&&nc>0&&nc<C-1&&g[nr][nc]===WALL){
        g[r+dr/2][c+dc/2]=EMPTY;carve(nr,nc);
      }
    }
  }
  carve(1,1);g[1][1]=EMPTY;g[R-2][C-2]=EMPTY;return g;
}
function makeRooms(R,C){
  const g=makeGrid(R,C);
  const rs=8;
  for(let br=0;br<R;br+=rs)for(let bc=0;bc<C;bc+=rs){
    for(let r=br;r<Math.min(br+rs,R);r++){if(r===br||r===br+rs-1)for(let c=bc;c<Math.min(bc+rs,C);c++)g[r][c]=WALL;}
    for(let c=bc;c<Math.min(bc+rs,C);c++){if(c===bc||c===bc+rs-1)for(let r=br;r<Math.min(br+rs,R);r++)g[r][c]=WALL;}
    const dr=br+Math.floor(rs/2);const dc=bc+Math.floor(rs/2);
    if(dr<R)g[dr][bc]=EMPTY;if(dc<C)g[br][dc]=EMPTY;
    if(dr<R&&bc+rs-1<C)g[dr][bc+rs-1]=EMPTY;if(dc<C&&br+rs-1<R)g[br+rs-1][dc]=EMPTY;
  }
  g[1][1]=EMPTY;g[R-2][C-2]=EMPTY;return g;
}

// ═══════════════════════════════════════════════════════════════
//  Map file parser (MovingAI format)
// ═══════════════════════════════════════════════════════════════
function parseMap(mapFile){
  const lines=fs.readFileSync(mapFile,'utf8').split(/\r?\n/);
  let h=0,w=0,ms=0;
  for(let i=0;i<lines.length;i++){
    const l=lines[i].trim();
    if(l.startsWith('height'))h=parseInt(l.split(/\s+/)[1]);
    else if(l.startsWith('width'))w=parseInt(l.split(/\s+/)[1]);
    else if(l==='map'){ms=i+1;break;}
  }
  const grid=[];
  for(let r=0;r<h;r++){
    const row=[],line=lines[ms+r]||'';
    for(let c=0;c<w;c++){const ch=line[c]||'@';row.push((ch==='.'||ch==='G')?EMPTY:WALL);}
    grid.push(row);
  }
  return grid;
}

function parseScen(scenFile){
  const lines=fs.readFileSync(scenFile,'utf8').split(/\r?\n/),scens=[];
  for(const l of lines){
    if(l.startsWith('version')||l.trim()==='')continue;
    const p=l.split('\t');if(p.length<9)continue;
    scens.push({bucket:parseInt(p[0]),start:[parseInt(p[5]),parseInt(p[4])],goal:[parseInt(p[7]),parseInt(p[6])],optCost:parseFloat(p[8])});
  }
  return scens;
}

// ═══════════════════════════════════════════════════════════════
//  Run all algorithms on a single scenario
// ═══════════════════════════════════════════════════════════════
const ALGOS = [
  {name:'Dijkstra',     fn:dijkstra, color:'#6366f1', optimal:true},
  {name:'A*',           fn:astar,    color:'#22c55e', optimal:true},
  {name:'Greedy BFS',   fn:greedyBFS,color:'#f59e0b', optimal:false},
  {name:'BFS',          fn:bfs,      color:'#8b5cf6', optimal:false},
  {name:'Bidir A*',     fn:biAstar,  color:'#06b6d4', optimal:true},
  {name:'JPS',          fn:jps,      color:'#ec4899', optimal:true},
  {name:'DEORE',         fn:(g,s,e)=>{const r=deoreCore(g,s,e,{});return{path:r.path,nodes:r.nodes,expanded:[],cost:r.path?computePathCost(r.path):Infinity};}, color:'#ef4444', optimal:false},
  {name:'DEORE',       fn:(g,s,e)=>{const r=deoreCoreCore(g,s,e,{});return{path:r.path,nodes:r.nodes,expanded:[],cost:r.path?computePathCost(r.path):Infinity};}, color:'#f97316', optimal:false},
];

function runAll(grid,s,e){
  const results=[];
  for(const algo of ALGOS){
    const t0=performance.now();
    const res=algo.fn(grid,s,e);
    const ms=performance.now()-t0;
    results.push({
      name:algo.name,color:algo.color,optimal:algo.optimal,
      nodes:res.nodes,cost:res.cost,ms,
      pathLen:res.path?res.path.length:0,
      path:res.path,expanded:res.expanded||[]
    });
  }
  return results;
}

// ═══════════════════════════════════════════════════════════════
//  Main: Run all tests and generate JSON
// ═══════════════════════════════════════════════════════════════
const args=process.argv.slice(2);
const limitArg=args.find(a=>a.startsWith('--limit='));
const LIMIT=limitArg?parseInt(limitArg.split('=')[1]):20;

const allResults={generated:[],movingai:[]};

// Generated maps
const GEN_MAPS=[
  {name:'Open Field',gen:()=>makeOpen(40,60),s:[1,1],e:[38,58]},
  {name:'Random 25%',gen:()=>makeRandom(40,60,0.25),s:[1,1],e:[38,58]},
  {name:'Random 40%',gen:()=>makeRandom(40,60,0.4),s:[1,1],e:[38,58]},
  {name:'Rooms',gen:()=>makeRooms(40,60),s:[1,1],e:[38,58]},
  {name:'Maze',gen:()=>makeMaze(41,61),s:[1,1],e:[39,59]},
];

console.log('═══════════════════════════════════════');
console.log('  DEORE FULL COMPARISON — 8 Algorithms');
console.log('═══════════════════════════════════════\n');

for(const m of GEN_MAPS){
  const grid=m.gen();
  // Ensure start and end are passable
  grid[m.s[0]][m.s[1]]=EMPTY;grid[m.e[0]][m.e[1]]=EMPTY;
  console.log(`▶ ${m.name} (${grid.length}×${grid[0].length})`);
  const results=runAll(grid,m.s,m.e);
  const dijCost=results[0].cost;
  
  console.log('  ┌────────────────┬────────┬────────────┬────────┬─────────┐');
  console.log('  │ Algorithm      │ Nodes  │ Cost       │ Ratio  │ Time ms │');
  console.log('  ├────────────────┼────────┼────────────┼────────┼─────────┤');
  for(const r of results){
    const ratio=isFinite(dijCost)&&dijCost>0&&isFinite(r.cost)?(r.cost/dijCost).toFixed(4):'N/A';
    console.log(`  │ ${r.name.padEnd(14)} │ ${String(r.nodes).padStart(6)} │ ${(isFinite(r.cost)?r.cost.toFixed(2):'∞').padStart(10)} │ ${ratio.padStart(6)} │ ${r.ms.toFixed(1).padStart(7)} │`);
  }
  console.log('  └────────────────┴────────┴────────────┴────────┴─────────┘\n');
  
  // Save grid and results for visualization (convert grid to compact string)
  const gridStr=grid.map(row=>row.map(c=>c===WALL?'#':'.').join('')).join('\n');
  allResults.generated.push({
    name:m.name,width:grid[0].length,height:grid.length,
    gridStr,start:m.s,end:m.e,
    algorithms:results.map(r=>({
      name:r.name,color:r.color,nodes:r.nodes,
      cost:isFinite(r.cost)?+r.cost.toFixed(4):null,
      ms:+r.ms.toFixed(2),pathLen:r.pathLen,
      ratio:isFinite(dijCost)&&dijCost>0&&isFinite(r.cost)?+(r.cost/dijCost).toFixed(4):null,
      path:r.path?r.path:null,
      expandedCount:r.expanded?r.expanded.length:0
    }))
  });
}

// MovingAI maps
const MAI_MAPS=[
  {map:'maps/dao/arena.map',scen:'maps/dao/arena.map.scen'},
];

// Check if sc1 exists
const sc1Map=path.resolve(__dirname,'maps/sc1/AcrosstheCape.map');
if(fs.existsSync(sc1Map)){
  MAI_MAPS.push({map:'maps/sc1/AcrosstheCape.map',scen:'maps/sc1/AcrosstheCape.map.scen'});
}

for(const m of MAI_MAPS){
  const mf=path.resolve(__dirname,m.map),sf=path.resolve(__dirname,m.scen);
  if(!fs.existsSync(mf)||!fs.existsSync(sf))continue;
  const grid=parseMap(mf);const scens=parseScen(sf);
  const scenLimit=Math.min(scens.length,LIMIT);
  console.log(`▶ MovingAI: ${path.basename(m.map)} (${grid.length}×${grid[0].length}, ${scenLimit} scenarios)`);
  
  // Aggregate stats
  const stats={};
  for(const a of ALGOS)stats[a.name]={nodes:0,cost:0,ms:0,count:0,misses:0,violations:0,optMatches:0};
  
  let sampleGrid=null,sampleResults=null,sampleStart=null,sampleEnd=null;
  
  for(let i=0;i<scenLimit;i++){
    const sc=scens[i];
    if(grid[sc.start[0]]?.[sc.start[1]]===WALL||grid[sc.goal[0]]?.[sc.goal[1]]===WALL)continue;
    if(sc.start[0]===sc.goal[0]&&sc.start[1]===sc.goal[1])continue;
    
    const results=runAll(grid,sc.start,sc.goal);
    
    // Save first scenario as sample for visualization
    if(!sampleGrid){
      sampleGrid=grid;sampleResults=results;sampleStart=sc.start;sampleEnd=sc.goal;
    }
    
    for(const r of results){
      const st=stats[r.name];
      st.count++;
      st.nodes+=r.nodes;
      st.ms+=r.ms;
      if(!r.path&&sc.optCost>0)st.misses++;
      else if(r.path){
        st.cost+=r.cost;
        if(sc.optCost>0&&r.cost/sc.optCost>2.501)st.violations++;
        if(sc.optCost>0&&Math.abs(r.cost-sc.optCost)<0.01)st.optMatches++;
      }
    }
    if((i+1)%10===0)process.stdout.write(`\r  Progress: ${i+1}/${scenLimit}...`);
  }
  console.log(`\r  Progress: ${scenLimit}/${scenLimit}... done!`);
  
  console.log('  ┌────────────────┬──────────┬──────────┬────────┬────────┬────────┐');
  console.log('  │ Algorithm      │ AvgNodes │  AvgCost │ AvgMs  │ Misses │ Viols  │');
  console.log('  ├────────────────┼──────────┼──────────┼────────┼────────┼────────┤');
  for(const a of ALGOS){
    const st=stats[a.name];if(!st.count)continue;
    const an=Math.round(st.nodes/st.count);
    const ac=(st.cost/st.count).toFixed(2);
    const am=(st.ms/st.count).toFixed(2);
    console.log(`  │ ${a.name.padEnd(14)} │ ${String(an).padStart(8)} │ ${ac.padStart(8)} │ ${am.padStart(6)} │ ${String(st.misses).padStart(6)} │ ${String(st.violations).padStart(6)} │`);
  }
  console.log('  └────────────────┴──────────┴──────────┴────────┴────────┴────────┘\n');
  
  // Save for visualization
  if(sampleGrid){
    const gridStr=sampleGrid.map(row=>row.map(c=>c===WALL?'#':'.').join('')).join('\n');
    allResults.movingai.push({
      name:path.basename(m.map),width:sampleGrid[0].length,height:sampleGrid.length,
      gridStr,start:sampleStart,end:sampleEnd,
      stats:Object.entries(stats).map(([name,st])=>({
        name,count:st.count,avgNodes:st.count?Math.round(st.nodes/st.count):0,
        avgCost:st.count?+(st.cost/st.count).toFixed(2):0,
        avgMs:st.count?+(st.ms/st.count).toFixed(2):0,
        misses:st.misses,violations:st.violations,optMatches:st.optMatches,
        color:ALGOS.find(a=>a.name===name)?.color||'#888'
      })),
      algorithms:sampleResults.map(r=>({
        name:r.name,color:r.color,nodes:r.nodes,
        cost:isFinite(r.cost)?+r.cost.toFixed(4):null,
        ms:+r.ms.toFixed(2),pathLen:r.pathLen,
        path:r.path,expandedCount:r.expanded?r.expanded.length:0
      }))
    });
  }
}

// Write JSON for HTML visualizer
const outFile=path.resolve(__dirname,'benchmark_results.json');
fs.writeFileSync(outFile,JSON.stringify(allResults,null,0));
console.log(`\n✅ Results saved to ${outFile}`);
console.log('Open DEORE_Visualizer.html to see visual results.\n');

