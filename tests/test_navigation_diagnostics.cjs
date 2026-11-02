// Real browser API/planning/preparation functions; all responses are synthetic.
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),vm=require('node:vm');
const tick=()=>new Promise(setImmediate);
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};
const diagnostic=()=>({classification:'map_surface_disagreement',reason_code:'unobserved_body_overlap',reason:'旧图占据落在机身盲区',recommended_action:'relocalize_and_replan',overlap_diagnostics:{overlap_cells:6,unobserved_body_cells:4,insufficient_vote_cells:6,live_blocked_cells:0,surface_state:'waiting_evidence',cells:[{cell:[2,3],observed_frames:0}],standing_is_not_map_clearing_evidence:true}});
function fixture(){
 const nodes=new Map(),all=[],events={},calls=[],toasts=[];let root;
 class Element{
  constructor(){this.children=[];this.parentNode=null;this.dataset={};this.disabled=false;this.hidden=false;this.value='';this.textContent='';this.attrs={};all.push(this);}
  set id(value){this._id=value;nodes.set('#'+value,this);}get id(){return this._id;}
  append(...children){for(const child of children){child.parentNode=this;this.children.push(child);}}
  prepend(child){child.parentNode=this;this.children.unshift(child);}
  insertBefore(child){this.append(child);}replaceChildren(){for(const child of this.children)child.parentNode=null;this.children=[];}
  setAttribute(key,value){this.attrs[key]=value;}addEventListener(){}querySelector(){return null;}
  remove(){this.parentNode=null;if(this.id)nodes.delete('#'+this.id);}
  matches(selector){return selector.split(',').some(s=>s.trim()==='#'+this.id||s.trim()===`[data-ui="${this.dataset.ui}"]`);}
  closest(selector){return this.matches(selector)?this:null;}
 }
 root=new Element();const connected=n=>n===root||n.parentNode&&connected(n.parentNode);
 const lookup=selector=>selector.startsWith('#')?nodes.get(selector):all.find(n=>connected(n)&&n.matches(selector));
 const add=(id,value='')=>{const n=new Element();n.id=id;n.value=value;root.append(n);return n;};
 for(const id of ['goal-plan-status','nav-feedback','nav-map'])add(id);
 add('goal-yaw-mode','auto');add('goal-yaw','0');add('point-navigation-engine','platform');
 for(const action of ['go-goal','goal-replan','pause-nav','cancel-nav']){const n=new Element();n.dataset.ui=action;root.append(n);}
 const map={id:'map',robot_map_name:'room',verified_for_robot:true,robot_ip:'fixture',origin:[0,0],height:100,resolution:.1,grid_id:0};
 const c=vm.createContext({console,performance,setTimeout,clearTimeout,setInterval(){},AbortController,crypto:require('node:crypto').webcrypto,
  S:{mode:'live',ip:'fixture',server_instance:'server',connection_generation:1,online:true,pose:{Location:0},nav:{status:'idle'},autonomy:{active:false},mapping:false,maps:[map],routes:[],navigation_preparation:{client_intent_protocol:1,verify_current_localization:true}},
  page:'navigation',navIntent:'goal',mapId:'map',routeId:null,points:[{x:2,y:3}],mapLibrary:{active:'room',maps:[{name:'room'}]},location:{hash:'#navigation'},ui:{},
  $:lookup,selectedMap:()=>c.S.maps.find(m=>m.id===c.mapId),navigationMapReady:()=>c.mapId==='map'&&c.mapLibrary.active==='room',routeOptions:()=>({}),
  text:(id,value)=>{const n=nodes.get('#'+id);if(n)n.textContent=value;},toast:(...args)=>toasts.push(args),renderPoints(){},render(){},update(){},drawMap(){},fetchState:async()=>{},runRoute(){},
  window:{addEventListener:(event,fn)=>(events[event]||=[]).push(fn)},
  document:{hidden:false,createElement:()=>new Element(),createElementNS:()=>new Element(),querySelectorAll:selector=>all.filter(n=>connected(n)&&n.matches(selector)),addEventListener:(event,fn)=>(events[event]||=[]).push(fn)}
 });
 c.ui['cancel-nav']=async()=>{};c.ui['pause-nav']=async()=>{};
 let response=path=>{
  if(path==='/api/platform')return structuredClone(c.S);
  if(path==='/api/nav/plan')return {ok:true,plan_id:'fresh-plan',map_id:'map',path:[{x:0,y:0},{x:2,y:3}],points:[{x:2,y:3}],length_m:4,goal:{x:2,y:3,yaw:0}};
  if(path==='/api/maps/localize-auto')return {ok:true,job_id:'static-recovery'};
  if(path.startsWith('/api/operations'))return {ok:true,jobs:{status:'completed',result:{localized:true}}};
  if(path==='/api/maps/library')return {ok:true,active:'actual-room',maps:[{name:'actual-room'}]};
  if(path==='/api/maps/library/sync')return {ok:true,map:{...map,id:'synced-map',robot_map_name:'actual-room'}};
  throw Error('Unexpected test endpoint: '+path);
 };
 c.fetch=async(path,options)=>{const data=options.body===undefined?undefined:JSON.parse(options.body);calls.push({path,data,options});const result=await response(path,data);return {ok:result.ok!==false,json:async()=>result};};
 const app=fs.readFileSync('platform_web/app.js','utf8');vm.runInContext(app.slice(app.indexOf('async function api('),app.indexOf('function modal(')),c);
 const planning=fs.readFileSync('platform_web/map_learning.js','utf8');vm.runInContext(planning.slice(planning.indexOf('let goalPlan=')),c);
 vm.runInContext(fs.readFileSync('platform_web/navigation_preparation.js','utf8'),c);
 return {c,calls,nodes,toasts,get:name=>vm.runInContext(name,c),lookup,setResponse:fn=>{const previous=response;response=(...args)=>fn(...args,previous);},setDiagnostics:d=>{c.diagnosticFixture=d;vm.runInContext('goalPlanDiagnostics=diagnosticFixture;goalPlanError=diagnosticFixture.reason;goalPlanKey=goalKey();',c);c.paintGoalPlan();}};
}
const starts=f=>f.calls.filter(r=>['/api/nav/start','/api/nav/goal'].includes(r.path));

test('API preserves structured navigation metadata and planner retains it on current failure',async()=>{
 const f=fixture(),d=diagnostic();f.setResponse((path,data,previous)=>path==='/api/nav/plan'?{ok:false,err:'当前位置与工作图重叠',navigation:d}:previous(path,data));
 await assert.rejects(f.c.api('/api/nav/plan',{},true),error=>error.navigation===d);
 await f.c.refreshGoalPlan();assert.equal(f.get('goalPlanDiagnostics'),d);
 assert.match(f.nodes.get('#goal-plan-diagnostics').children.map(n=>n.textContent).join(' '),/地图重叠 6 格.*机身盲区尚无观测 4 格/);
 const button=f.lookup('[data-ui="goal-localize-preview"]');assert.ok(button);assert.equal(button.disabled,false);
 f.c.paintGoalPlan();assert.equal(f.lookup('[data-ui="goal-localize-preview"]'),button,'stable diagnostics preserve focusable button');
 assert.equal(starts(f).length,0);
});
test('Location 0 conflict recovery explicitly forces structural localization and only updates preview',async()=>{
 const f=fixture();f.setDiagnostics(diagnostic());await f.c.ui['goal-localize-preview']();
 assert.deepEqual(f.calls.map(r=>r.path),['/api/platform','/api/maps/localize-auto','/api/operations?id=static-recovery','/api/platform','/api/nav/plan']);
 assert.deepEqual(f.calls[1].data,{verify_current:true});assert.equal(f.get('goalPlan').plan_id,'fresh-plan');assert.equal(f.get('goalPlanDiagnostics'),null);
 assert.equal(starts(f).length,0);assert.match(f.nodes.get('#goal-plan-status').textContent,/尚未开始行走/);
});
test('a normal-location backend without force-verification capability cannot offer false success',async()=>{
 const f=fixture();delete f.c.S.navigation_preparation.verify_current_localization;f.setDiagnostics(diagnostic());
 assert.equal(f.lookup('[data-ui="goal-localize-preview"]').disabled,true);
 await assert.rejects(f.c.ui['goal-localize-preview'](),/静态结构识别/);assert.equal(f.calls.length,0);
});
test('cancel or map/target/connection change discards late static localization completion',async()=>{
 const changes=[async f=>f.c.ui['cancel-nav'](),async f=>{f.c.mapId='other';f.c.update();},async f=>{f.c.points[0].x=8;f.c.update();},async f=>{f.c.S.connection_generation++;f.c.update();}];
 for(const change of changes){
  const f=fixture(),gate=deferred();f.setDiagnostics(diagnostic());f.setResponse((path,data,previous)=>path.startsWith('/api/operations')?gate.promise:previous(path,data));
  const work=f.c.ui['goal-localize-preview']();await tick();await change(f);
  gate.resolve({ok:true,jobs:{status:'completed',result:{localized:true}}});await work;
  assert.equal(f.calls.some(r=>r.path==='/api/nav/plan'),false);assert.equal(f.get('goalPlan'),null);assert.equal(starts(f).length,0);
 }
});
test('a late plan or its diagnostics cannot replace a changed selection',async()=>{
 for(const failure of [false,true]){
  const f=fixture(),gate=deferred();f.setDiagnostics(diagnostic());f.setResponse((path,data,previous)=>path==='/api/nav/plan'?gate.promise:previous(path,data));
  const work=f.c.ui['goal-localize-preview']();await tick();f.c.mapId='other';f.c.update();
  gate.resolve(failure?{ok:false,err:'old failure',navigation:diagnostic()}:{ok:true,plan_id:'late'});await work;
  assert.equal(f.get('goalPlan'),null);assert.equal(f.get('goalPlanDiagnostics'),null);assert.equal(starts(f).length,0);
 }
});
test('expired or insufficient observations offer one read-only recheck without localization',async()=>{
 for(const code of ['live_evidence_expired','insufficient_ground_votes']){
  const f=fixture(),d={...diagnostic(),reason_code:code,recommended_action:'revalidate_observation'};f.setDiagnostics(d);
  assert.ok(f.lookup('[data-ui="goal-observation-recheck"]'));await f.c.ui['goal-observation-recheck']();
  assert.deepEqual(f.calls.map(r=>r.path),['/api/platform','/api/nav/plan']);assert.equal(starts(f).length,0);
 }
});
test('map identity mismatch offers active-map sync instead of localization or map activation',async()=>{
 const f=fixture();f.setDiagnostics({...diagnostic(),classification:'map_identity_mismatch',reason_code:'active_map_mismatch',recommended_action:'sync_active_map'});
 assert.ok(f.lookup('[data-ui="goal-sync-active-map"]'));assert.equal(f.lookup('[data-ui="goal-localize-preview"]'),undefined);
 f.c.S.pose.Location=1;f.c.paintGoalPlan();assert.equal(f.lookup('[data-ui="go-goal"]').disabled,true);
 await assert.rejects(f.c.ui['go-goal'](),/同步/);
 await assert.rejects(f.c.ui['goal-localize-preview'](),/同步/);assert.equal(f.calls.length,0);
 await f.c.ui['goal-sync-active-map']();assert.deepEqual(f.calls.map(r=>r.path),['/api/platform','/api/maps/library','/api/maps/library/sync']);
 assert.deepEqual(f.calls.at(-1).data,{name:'actual-room',grid_id:0});assert.equal(f.c.mapId,'synced-map');assert.equal(f.c.points.length,0);
 assert.equal(starts(f).length,0);assert.equal(f.calls.some(r=>r.path.includes('/apply')),false);
});
test('a remaining overlap after static recovery retains the newer diagnostics without starting',async()=>{
 const f=fixture(),next={...diagnostic(),reason_code:'insufficient_ground_votes',recommended_action:'revalidate_observation'};f.setDiagnostics(diagnostic());
 f.setResponse((path,data,previous)=>path==='/api/nav/plan'?{ok:false,err:'还需独立地面观测',navigation:next}:previous(path,data));
 await assert.rejects(f.c.ui['goal-localize-preview'](),/独立地面观测/);assert.equal(f.get('goalPlanDiagnostics'),next);assert.equal(starts(f).length,0);
 assert.ok(f.lookup('[data-ui="goal-observation-recheck"]'));
});
