// Browser state regressions. All API responses are local fixtures; no robot I/O.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
function fixture(){
 const nodes=new Map(),lines=[],writes=[],listeners={},requests=[],timers=[];
 const parent={insertBefore(child){if(child.innerHTML?.includes('point-navigation-engine'))nodes.set('#point-navigation-engine',{value:'platform',addEventListener(event,fn){this[event]=fn;}});}};
 const element=()=>({parentNode:parent,hidden:false,textContent:'',setAttribute(k,v){this[k]=v;},remove(){const i=lines.indexOf(this);if(i>=0)lines.splice(i,1);},querySelector(){return null;},append(child){lines.push(child);}});
 const button=element();button.dataset={ui:'go-goal'};let disabled=true;Object.defineProperty(button,'disabled',{get:()=>disabled,set:v=>{disabled=v;writes.push(v);}});
 nodes.set('[data-ui="go-goal"]',button);nodes.set('#main .page-heading',element());nodes.set('#map-navigation-flow',element());nodes.set('#nav-map',element());
 nodes.set('#goal-yaw-mode',{value:'auto'});nodes.set('#goal-yaw',{value:'0'});nodes.set('#goal-yaw-field',element());
 const map={id:'m1',robot_ip:'fixture',robot_map_name:'room',verified_for_robot:true,origin:[0,0],resolution:.1,height:100};
 let now=1000;
 const c=vm.createContext({console,page:'navigation',navIntent:'goal',points:[{x:2,y:3}],mapId:'m1',S:{mode:'live',ip:'fixture',nav:{status:'idle'}},mapLibrary:{active:'room'},
  performance:{now:()=>now},ui:{},selectedMap:()=>map,mapFlowStatus:()=>'',drawMap:()=>{},renderPoints:()=>c.drawMap(),
  $:s=>nodes.get(s)||lines.find(n=>'#'+n.id===s),text:(id,value)=>{if(!nodes.has('#'+id))nodes.set('#'+id,element());nodes.get('#'+id).textContent=value;},
  document:{hidden:false,createElement:element,createElementNS:element,addEventListener:(event,fn)=>{(listeners[event]||=[]).push(fn);}},
  AbortController,clearTimeout:timer=>{if(timer)timer.cancelled=true;},render:()=>{},api:async(path,data)=>{requests.push({path,data});return plan(data);},fetchState:async()=>{},toast(){},setInterval(){},setTimeout:(fn,ms)=>{const timer={fn,ms,cancelled:false};timers.push(timer);return timer;}});
 const workspace=fs.readFileSync('platform_web/workspace.js','utf8');
 const start=workspace.includes('function navigationMapReady()')?'function navigationMapReady()':'function paintMapFlow()';
 vm.runInContext(workspace.slice(workspace.indexOf(start),workspace.indexOf('const previousPaintMapLibrary=')),c);
 const source=fs.readFileSync('platform_web/map_learning.js','utf8');vm.runInContext(source.slice(source.indexOf('let goalPlan=')),c);
 vm.runInContext("ui['clear-points']=()=>{points=[];renderPoints();};",c);
 return {c,map,nodes,lines,button,writes,listeners,requests,timers,advance:ms=>{now+=ms;},value:name=>vm.runInContext(name,c)};
}
function plan(data={}){return {plan_id:'test-plan',map_id:data.map_id||'m1',path:[{x:1,y:1},{x:data.x||2,y:data.y||3}],length_m:3,points:[{x:2,y:3}],goal:{x:2,y:3,yaw:0}};}

test('browser shortcuts and composition do not start global motion',()=>{
 const source=fs.readFileSync('platform_web/app.js','utf8'),listeners={},moves=[];
 const c=vm.createContext({document:{addEventListener:(name,handler)=>listeners[name]=handler},
  $:()=>({open:false}),keys:new Set(),page:'tasks',session:null,
  startMotion:axes=>moves.push(Array.from(axes)),stopMotion:()=>moves.push([0,0,0])});
 vm.runInContext(source.slice(source.indexOf('function typingTarget('),source.indexOf('function blurStop(')),c);
 const event={key:'a',code:'KeyA',target:{closest:()=>null},preventDefault(){}};
 for(const modifier of ['metaKey','ctrlKey','altKey','shiftKey','isComposing'])listeners.keydown({...event,[modifier]:true});
 assert.deepEqual(moves,[]);
 listeners.keydown(event);listeners.keyup(event);
 assert.deepEqual(moves,[[0,1,0],[0,0,0]]);
});

test('single-goal preview and execution use the onboard planner',async()=>{
 const f=fixture();await f.c.refreshGoalPlan();await f.c.ui['go-goal']();
 assert.equal(f.requests[0].data.engine,'gos');
 assert.equal(f.requests.at(-1).path,'/api/nav/goal');assert.equal(f.requests.at(-1).data.engine,'gos');
 assert.equal(f.nodes.has('#point-navigation-engine'),false);
});

test('active platform navigation displays its updated route rather than the old preview',async()=>{
 const f=fixture();await f.c.refreshGoalPlan();f.c.S.nav={status:'running',engine:'platform',plan:{path:[{x:1,y:1},{x:4,y:6}]}};
 f.c.drawMap();assert.match(f.lines[0].points,/40,40/);
});

test('map refresh cannot enable the goal button before a valid plan exists',()=>{
 const f=fixture();for(let i=0;i<6;i++){f.c.paintGoalPlan();f.c.paintMapFlow();}
 assert.equal(f.button.disabled,true);assert.ok(!f.writes.includes(false));
});
test('map readiness and task state are included in the same button decision',async()=>{
 const f=fixture();await f.c.refreshGoalPlan();f.c.paintMapFlow();assert.equal(f.button.disabled,false);
 f.c.S.nav.status='running';f.c.paintGoalPlan();f.c.paintMapFlow();assert.equal(f.button.disabled,true);
 f.c.S.nav.status='idle';f.map.verified_for_robot=false;f.c.paintMapFlow();f.c.paintGoalPlan();assert.equal(f.button.disabled,true);
 f.map.verified_for_robot=true;f.advance(56000);f.c.paintMapFlow();assert.equal(f.button.disabled,true);
});
test('repeated status drawing keeps one preview; clearing removes it immediately',async()=>{
 const f=fixture();await f.c.refreshGoalPlan();for(let i=0;i<30;i++)f.c.drawMap();assert.equal(f.lines.length,1);
 f.c.ui['clear-points']();assert.equal(f.lines.length,0);assert.equal(f.button.disabled,true);assert.equal(f.value('goalPlan'),null);
 assert.match(f.nodes.get('#goal-plan-status').textContent,/选定目标|自动规划/);
});
test('clear then reselect the same point rejects a late plan from before the clear',async()=>{
 const f=fixture();let finish;f.c.api=()=>new Promise(r=>finish=r);const request=f.c.refreshGoalPlan();
 f.c.ui['clear-points']();f.c.points=[{x:2,y:3}];f.c.renderPoints();finish(plan());await request;
 assert.equal(f.value('goalPlan'),null);assert.equal(f.lines.length,0);assert.equal(f.button.disabled,true);
 f.c.api=async()=>plan();await f.c.refreshGoalPlan();assert.equal(f.button.disabled,false);assert.equal(f.lines.length,1);
});
test('late planning failure cannot contaminate a cleared or reselected goal',async()=>{
 const f=fixture();let reject;f.c.api=()=>new Promise((r,j)=>reject=j);const request=f.c.refreshGoalPlan();
 f.c.ui['clear-points']();f.c.points=[{x:2,y:3}];f.c.renderPoints();reject(Error('obsolete error'));await request;
 assert.equal(f.value('goalPlanError'),'');assert.equal(f.value('goalPlanBusy'),false);
});
test('clearing a draft never resurrects the server route or cancels a running task',async()=>{
 const f=fixture();await f.c.refreshGoalPlan();f.c.S.nav={status:'running',route:{temporary:true,map_id:'m1'},plan:plan()};
 f.c.ui['clear-points']();for(let i=0;i<5;i++)f.c.drawMap();
 assert.equal(f.lines.length,0);assert.equal(f.c.S.nav.status,'running');assert.ok(f.requests.every(r=>r.path==='/api/nav/plan'));
});
test('changing map, yaw, or navigation mode removes stale preview',async()=>{
 for(const change of ['map','yaw','mode']){
  const f=fixture();await f.c.refreshGoalPlan();
  if(change==='map'){f.c.mapId='m2';f.c.points=[];f.c.renderPoints();}
  if(change==='yaw'){f.nodes.get('#goal-yaw-mode').value='manual';for(const fn of f.listeners.change)fn({target:{id:'goal-yaw-mode'}});}
  if(change==='mode'){f.c.navIntent='route';f.c.points=[];f.c.renderPoints();}
  f.c.drawMap();assert.equal(f.lines.length,0,change);assert.equal(f.value('goalPlan'),null,change);
 }
});
test('goal submission stays disabled during the request and ignores a duplicate click',async()=>{
 const f=fixture();await f.c.refreshGoalPlan();let finish,count=0;
 f.c.api=()=>{count++;return new Promise(r=>finish=r);};
 const submit=f.c.ui['go-goal']();f.c.paintMapFlow();f.c.paintGoalPlan();assert.equal(f.button.disabled,true);
 await f.c.ui['go-goal']();assert.equal(count,1);finish({message:'fixture accepted'});await submit;
});

test('replan without a target cannot cancel an existing task',async()=>{
 const f=fixture();f.c.points=[];f.c.S.nav.status='running';f.c.fetchState=async()=>{f.c.S.nav.status='cancelled';};
 await assert.rejects(f.c.ui['goal-replan'](),/目标/);assert.equal(f.requests.length,0);
});
test('clearing a hanging preview releases its request and an older completion cannot unlock the next preview',async()=>{
 const f=fixture();let finishOld,finishNew,oldSignal;
 f.c.api=(path,data,quiet,options)=>{oldSignal=options?.signal;return new Promise(r=>finishOld=r);};
 const old=f.c.refreshGoalPlan();f.c.ui['clear-points']();assert.equal(oldSignal?.aborted,true);assert.equal(f.value('goalPlanBusy'),false);
 f.c.points=[{x:7,y:3}];f.c.renderPoints();f.c.api=()=>new Promise(r=>finishNew=r);const current=f.c.refreshGoalPlan();
 finishOld(plan());await old;assert.equal(f.value('goalPlanBusy'),true);assert.equal(f.value('goalPlan'),null);
 finishNew(plan({x:7,y:3}));await current;assert.equal(f.value('goalPlanBusy'),false);assert.equal(f.button.disabled,false);
});
test('planning timeout reports a retryable reason and releases the busy state',async()=>{
 const f=fixture();f.c.api=(path,data,quiet,options)=>new Promise((resolve,reject)=>options?.signal.addEventListener('abort',()=>reject(Object.assign(Error('abort'),{name:'AbortError'}))));
 const request=f.c.refreshGoalPlan();const timeout=f.timers.find(t=>t.ms===15000);assert.ok(timeout,'bounded planning request');timeout.fn();await request;
 assert.equal(f.value('goalPlanBusy'),false);assert.match(f.value('goalPlanError'),/超时.*重新规划/);assert.equal(f.button.disabled,true);
});
test('replan does not switch to a different target while waiting for cancellation',async()=>{
 const f=fixture();let finish;f.c.S.nav.status='running';f.c.fetchState=()=>new Promise(r=>finish=()=>{f.c.S.nav.status='cancelled';r();});
 const request=f.c.ui['goal-replan']();await new Promise(setImmediate);
 f.c.points=[{x:8,y:2}];f.c.renderPoints();finish();await request;
 assert.deepEqual(f.requests.map(r=>r.path),['/api/nav/cancel']);
});
