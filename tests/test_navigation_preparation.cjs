// Synthetic UI/API contracts only: no platform server, network or robot.
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),vm=require('node:vm');
const tick=()=>new Promise(setImmediate);
const deferred=()=>{let resolve;const promise=new Promise(r=>resolve=r);return {promise,resolve};};
function fixture(){
 const nodes=new Map(),events={},calls=[];
 const element=(id,value='')=>({id,value,disabled:false,textContent:'',dataset:{},hidden:false,setAttribute(){},prepend(el){nodes.set('#'+el.id,el);}});
 for(const id of ['goal-plan-status','nav-feedback','goal-yaw-mode','goal-yaw'])nodes.set('#'+id,element(id));
 nodes.get('#goal-yaw-mode').value='auto';
 for(const name of ['go-goal','goal-replan','start-nav','pause-nav','cancel-nav'])nodes.set(`[data-ui="${name}"]`,element(name));
 const routeButton=element('run');nodes.set('[data-route-run]',routeButton);
 const ctx=vm.createContext({console,performance,setTimeout,clearTimeout,AbortController,crypto:require('node:crypto').webcrypto,
  S:{mode:'live',ip:'fixture',server_instance:'server',connection_generation:1,navigation_preparation:{client_intent_protocol:1,verify_current_localization:true},online:true,pose:{Location:1},mapping:false,autonomy:{active:false},nav:{status:'idle'},maps:[{id:'map',robot_map_name:'room',robot_ip:'fixture',grid_id:0,verified_for_robot:true}],routes:[{id:'route',map_id:'map',points:[{x:2,y:1}],options:{nav_mode:1,speed:1}}]},
  page:'navigation',location:{hash:'#navigation'},navIntent:'goal',mapId:'map',routeId:null,points:[{x:2,y:1}],pointNavigationEngine:'platform',
  goalSubmitting:false,goalReplanning:false,goalPlanBusy:false,goalPlanError:'',goalPlanDiagnostics:null,goalPlan:{plan_id:'OLD'},goalPlanKey:'',mapLibrary:{active:'room',maps:[{name:'room',grids:[0]}]},
  ui:{},$:s=>nodes.get(s),text:(id,value)=>{if(nodes.has('#'+id))nodes.get('#'+id).textContent=value;},
  goalKey:()=>[ctx.mapId,ctx.points[0]?.x,ctx.points[0]?.y,ctx.pointNavigationEngine].join('|'),navigationMapReady:()=>true,
  routeOptions:()=>({nav_mode:1,speed:1}),invalidateGoalPlan:()=>{ctx.goalPlan=null;ctx.goalPlanKey='';},
  paintGoalPlan:()=>{nodes.get('[data-ui="go-goal"]').disabled=!ctx.goalPlan||!ctx.navigationMapReady();},drawMap(){},update(){},render(){},toast(){},fetchState:async()=>{},runRoute(){},
  window:{addEventListener:(event,fn)=>{(events[event]||=[]).push(fn);}},
  document:{hidden:false,createElement:()=>element(''),querySelectorAll:()=>[routeButton,nodes.get('[data-ui="start-nav"]'),nodes.get('[data-ui="goal-replan"]')],addEventListener:(event,fn)=>{(events[event]||=[]).push(fn);}}
 });
 for(const name of ['pause-nav','cancel-nav','nav-takeover','clear-points','nav-goal-mode','nav-route-mode','goal-replan','hub-connect','hub-demo'])ctx.ui[name]=async()=>{calls.push({path:'handler:'+name});};
 let responder=path=>{
  if(path==='/api/platform')return structuredClone(ctx.S);
  if(path==='/api/maps/localize-auto')return {job_id:'recover'};
  if(path.startsWith('/api/operations')){ctx.S.pose.Location=0;return {jobs:{status:'completed',result:{localized:true}}};}
  if(path==='/api/nav/plan')return {plan_id:'NEW',path:[],goal:{yaw:0},points:[],length_m:2};
  if(path==='/api/route/save')return {route:{id:'saved-route'}};
  if(path==='/api/maps/library')return structuredClone(ctx.mapLibrary);
  if(path==='/api/maps/library/sync')return {map:{...ctx.S.maps[0],verified_for_robot:true}};
  return {ok:true};
 };
 ctx.api=async(path,data,quiet,options)=>{calls.push({path,data,options});return responder(path,data,options);};
 ctx.fetch=async(path,options)=>{const data=JSON.parse(options.body);calls.push({path,data,options});const result=await responder(path,data,options);return {ok:true,json:async()=>result};};
 vm.runInContext(fs.readFileSync('platform_web/navigation_preparation.js','utf8'),ctx);
 return {ctx,nodes,calls,events,setResponder:fn=>{const previous=responder;responder=(...args)=>fn(...args,previous);},emit:(name,target={})=>{for(const fn of events[name]||[])fn({target,code:target.code});}};
}
const paths=f=>f.calls.map(r=>r.path);
const started=f=>f.calls.some(r=>['/api/nav/goal','/api/nav/start'].includes(r.path));

test('painting, polling and selection do not initialize localization or start navigation',()=>{
 const f=fixture();f.ctx.paintGoalPlan();f.ctx.update();f.ctx.render();
 assert.deepEqual(f.calls,[]);assert.equal(f.nodes.get('[data-ui="go-goal"]').disabled,false);
 assert.equal(f.nodes.get('[data-ui="go-goal"]').textContent,'执行导航');
 f.ctx.navigationMapReady=()=>false;f.ctx.paintGoalPlan();assert.equal(f.nodes.get('[data-ui="go-goal"]').disabled,true);
});
test('explicit goal start recovers then uses a newly planned id, never the old preview',async()=>{
 const f=fixture();await f.ctx.ui['go-goal']();
 assert.deepEqual(paths(f),['/api/platform','/api/maps/localize-auto','/api/operations?id=recover','/api/platform','/api/nav/plan','/api/nav/goal']);
 assert.equal(f.calls.at(-1).data.plan_id,'NEW');assert.equal(f.calls.at(-1).data.engine,'platform');
 assert.equal(f.ctx.window.M20NavigationPreparation.state,null);assert.equal(f.ctx.goalSubmitting,false);
});
test('ready or demo navigation still replans but does not start a localization operation',async()=>{
 for(const mode of ['live','demo']){const f=fixture();f.ctx.S.mode=mode;f.ctx.S.pose.Location=0;await f.ctx.ui['go-goal']();
  assert.deepEqual(paths(f),['/api/platform','/api/nav/plan','/api/nav/goal']);}
});
test('saved patrol start from tasks recovers before submitting the unchanged route options',async()=>{
 const f=fixture();f.ctx.page='tasks';f.ctx.location.hash='#tasks';await f.ctx.runRoute('route');
 assert.deepEqual(paths(f),['/api/platform','/api/maps/localize-auto','/api/operations?id=recover','/api/platform','/api/nav/plan','/api/nav/start']);
 assert.equal(f.calls.at(-1).data.id,'route');assert.equal(f.calls.at(-1).data.nav_mode,1);assert.equal(f.calls.at(-1).data.speed,1);
});
test('edited route start binds the original user intent before any asynchronous save',async()=>{
 const f=fixture();f.ctx.navIntent='route';await f.ctx.ui['start-nav']();
 assert.deepEqual(paths(f),['/api/platform','/api/maps/localize-auto','/api/operations?id=recover','/api/platform','/api/nav/plan','/api/route/save','/api/nav/start']);
 assert.equal(f.calls.at(-1).data.id,'saved-route');assert.equal(f.ctx.routeId,'saved-route');
});
test('late edited-route save after cancellation cannot create a new start intent',async()=>{
 const f=fixture(),gate=deferred();f.ctx.navIntent='route';f.ctx.S.pose.Location=0;
 f.setResponder((path,data,options,previous)=>path==='/api/route/save'?gate.promise:previous(path));
 const work=f.ctx.ui['start-nav']();await tick();await f.ctx.ui['cancel-nav']();
 gate.resolve({route:{id:'late-save'}});await work;assert.equal(started(f),false);assert.equal(f.ctx.routeId,null);
});
test('failed or unconfirmed localization never plans or starts',async()=>{
 for(const job of [{status:'failed',error:'环境仍不能唯一匹配'},{status:'completed',result:{localized:false}}]){
  const f=fixture();f.setResponder((path,data,options,previous)=>path.startsWith('/api/operations')?{jobs:job}:previous(path));
  await assert.rejects(f.ctx.ui['go-goal'](),/匹配|尚未确认/);assert.equal(started(f),false);
  assert.equal(paths(f).includes('/api/nav/plan'),false);assert.equal(f.ctx.goalSubmitting,false);
 }
});
test('successful operation with still-lost platform feedback cannot start',async()=>{
 const f=fixture();f.setResponder((path,data,options,previous)=>path.startsWith('/api/operations')?{jobs:{status:'completed',result:{localized:true}}}:previous(path));
 await assert.rejects(f.ctx.ui['go-goal'](),/反馈仍未就绪/);assert.equal(started(f),false);
});
test('pause and manual takeover invalidate pending intent before their existing handlers',async()=>{
 for(const name of ['pause-nav','cancel-nav','nav-takeover']){
  const f=fixture(),gate=deferred();f.setResponder((path,data,options,previous)=>path.startsWith('/api/operations')?gate.promise:previous(path));
  const work=f.ctx.ui['go-goal']();await tick();assert.equal(f.nodes.get('[data-ui="pause-nav"]').disabled,false);
  await f.ctx.ui[name]();assert.equal(f.ctx.window.M20NavigationPreparation.state,null);
  gate.resolve({jobs:{status:'completed',result:{localized:true}}});await work;assert.equal(started(f),false);
  assert.ok(paths(f).includes('handler:'+name));
 }
});
test('late localization success after target, map, connection, page or visibility change cannot start',async()=>{
 const changes=[f=>{f.ctx.points[0].x=3;},f=>{f.ctx.mapId='other';},f=>{f.ctx.S.ip='other';},
  f=>{f.ctx.S.connection_generation++;},f=>{f.ctx.S.server_instance='new-server';},f=>{f.ctx.page='motion';f.ctx.location.hash='#motion';},
  f=>{f.ctx.document.hidden=true;}];
 for(const change of changes){
  const f=fixture(),gate=deferred();f.setResponder((path,data,options,previous)=>path.startsWith('/api/operations')?gate.promise:previous(path));
  const work=f.ctx.ui['go-goal']();await tick();change(f);f.ctx.update();
  gate.resolve({jobs:{status:'completed',result:{localized:true}}});await work;
  assert.equal(started(f),false);assert.equal(f.ctx.window.M20NavigationPreparation.state,null);
 }
});
test('late route recovery cannot execute an edited route',async()=>{
 const f=fixture(),gate=deferred();f.setResponder((path,data,options,previous)=>path.startsWith('/api/operations')?gate.promise:previous(path));
 const work=f.ctx.runRoute('route');await tick();f.ctx.S.routes[0].points[0].x=99;f.ctx.update();
 gate.resolve({jobs:{status:'completed',result:{localized:true}}});await work;assert.equal(started(f),false);
});
test('cancel during replanning discards the late plan without a goal request',async()=>{
 const f=fixture(),gate=deferred();f.ctx.S.pose.Location=0;
 f.setResponder((path,data,options,previous)=>path==='/api/nav/plan'?gate.promise:previous(path));
 const work=f.ctx.ui['go-goal']();await tick();await f.ctx.ui['cancel-nav']();
 gate.resolve({plan_id:'LATE'});await work;assert.equal(started(f),false);assert.equal(f.ctx.goalPlan,null);
});
test('failed fresh plan never falls back to the previous plan id',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;f.setResponder((path,data,options,previous)=>{if(path==='/api/nav/plan')throw Error('目标不可达');return previous(path);});
 await assert.rejects(f.ctx.ui['go-goal'](),/不可达/);assert.equal(started(f),false);assert.equal(f.ctx.goalPlan,null);
});
test('explicit input event cancels even when a form is later reverted',async()=>{
 const f=fixture(),gate=deferred();f.setResponder((path,data,options,previous)=>path.startsWith('/api/operations')?gate.promise:previous(path));
 const work=f.ctx.ui['go-goal']();await tick();f.emit('input',{matches:()=>true});
 gate.resolve({jobs:{status:'completed',result:{localized:true}}});await work;assert.equal(started(f),false);
});
test('a stale previous request cannot release a newer pending goal intent',async()=>{
 const f=fixture(),first=deferred(),second=deferred();let count=0;
 f.setResponder((path,data,options,previous)=>path.startsWith('/api/operations')?(++count===1?first.promise:second.promise):previous(path));
 const old=f.ctx.ui['go-goal']();await tick();await f.ctx.ui['cancel-nav']();const newer=f.ctx.ui['go-goal']();await tick();
 first.resolve({jobs:{status:'completed',result:{localized:true}}});await old;
 assert.ok(f.ctx.window.M20NavigationPreparation.state);assert.equal(f.ctx.goalSubmitting,true);
 await f.ctx.ui['cancel-nav']();second.resolve({jobs:{status:'completed',result:{localized:true}}});await newer;assert.equal(started(f),false);
});
test('duplicate start clicks share a single preparation and fresh status failure prevents progress',async()=>{
 const f=fixture(),gate=deferred();f.setResponder((path,data,options,previous)=>path==='/api/platform'?gate.promise:previous(path));
 const work=f.ctx.ui['go-goal']();await tick();await f.ctx.ui['go-goal']();await f.ctx.runRoute('route');assert.equal(f.calls.length,1);
 gate.resolve({...structuredClone(f.ctx.S),online:false});await work;assert.equal(started(f),false);
});
test('a new UI with an old backend cannot issue automatic preparation requests',async()=>{
 const f=fixture();delete f.ctx.S.navigation_preparation;f.ctx.paintGoalPlan();
 assert.equal(f.nodes.get('[data-ui="go-goal"]').disabled,true);
 assert.match(f.nodes.get('[data-ui="go-goal"]').textContent,/新版/);
 await assert.rejects(f.ctx.ui['go-goal'](),/新版/);await assert.rejects(f.ctx.runRoute('route'),/新版/);assert.equal(f.calls.length,0);
});
test('replacing a target after a final goal POST revokes exactly its ID before the late reply',async()=>{
 const f=fixture(),gate=deferred();f.ctx.S.pose.Location=0;
 f.setResponder((path,data,options,previous)=>path==='/api/nav/goal'?gate.promise:previous(path));
 const work=f.ctx.ui['go-goal']();await tick();const start=f.calls.find(r=>r.path==='/api/nav/goal');assert.ok(start);
 f.ctx.points[0].x=4;f.ctx.update();await tick();
 const cancel=f.calls.find(r=>r.path==='/api/nav/intent/cancel');assert.ok(cancel);
 assert.equal(cancel.data.client_intent,start.data.client_intent);assert.equal(cancel.data.server_instance,start.data.server_instance);
 assert.equal(cancel.data.connection_generation,start.data.connection_generation);assert.equal(cancel.options.keepalive,true);
 assert.equal(paths(f).includes('/api/nav/cancel'),false);gate.resolve({ok:true});await work;
});
test('submitted navigation continues in the background while its acknowledgement is pending',async()=>{
 for(const action of ['hidden','page','pagehide']){
  const f=fixture(),gate=deferred();f.ctx.S.pose.Location=0;
  f.setResponder((path,data,options,previous)=>path==='/api/nav/goal'?gate.promise:previous(path));
  const work=f.ctx.ui['go-goal']();await tick();assert.ok(f.calls.some(r=>r.path==='/api/nav/goal'));
  if(action==='hidden'){f.ctx.document.hidden=true;f.emit('visibilitychange');}
  if(action==='page'){f.ctx.page='tasks';f.ctx.location.hash='#tasks';f.ctx.render();f.emit('hashchange');}
  if(action==='pagehide')f.emit('pagehide');
  gate.resolve({ok:true});await work;assert.equal(f.calls.some(r=>r.path==='/api/nav/intent/cancel'),false);
  assert.equal(f.ctx.window.M20NavigationPreparation.state,null);
 }
});
test('browser request loss after a submitted task goes to background does not revoke that task',async()=>{
 const f=fixture(),gate=deferred();f.ctx.S.pose.Location=0;
 f.setResponder(async(path,data,options,previous)=>{if(path==='/api/nav/goal'){await gate.promise;throw Error('page unloaded');}return previous(path);});
 const work=f.ctx.ui['go-goal']();await tick();f.emit('pagehide');gate.resolve();await work;
 assert.equal(f.calls.some(r=>r.path==='/api/nav/intent/cancel'),false);
});
test('edited and saved route final POST cancellation is scoped and cannot stop another intent',async()=>{
 for(const draft of [false,true]){
  const f=fixture(),gate=deferred();f.ctx.S.pose.Location=0;f.ctx.navIntent='route';
  f.setResponder((path,data,options,previous)=>path==='/api/nav/start'?gate.promise:previous(path));
  const work=draft?f.ctx.ui['start-nav']():f.ctx.runRoute('route');await tick();
  const start=f.calls.find(r=>r.path==='/api/nav/start');assert.ok(start);
  f.emit('input',{matches:()=>true});await tick();
  assert.equal(f.calls.find(r=>r.path==='/api/nav/intent/cancel').data.client_intent,start.data.client_intent);
  gate.resolve({ok:true});await work;
 }
});
test('lost final POST response still causes a scoped server revoke',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;f.setResponder((path,data,options,previous)=>{if(path==='/api/nav/goal')throw Error('response lost');return previous(path);});
 await assert.rejects(f.ctx.ui['go-goal'](),/response lost/);await tick();
 const start=f.calls.find(r=>r.path==='/api/nav/goal'),cancel=f.calls.find(r=>r.path==='/api/nav/intent/cancel');
 assert.equal(cancel.data.client_intent,start.data.client_intent);
});
test('a failed preview does not require a manual unlock before Go',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;f.ctx.goalPlan=null;f.ctx.goalPlanError='old preview unavailable';
 f.ctx.paintGoalPlan();assert.equal(f.nodes.get('[data-ui="go-goal"]').disabled,false);
 await f.ctx.ui['go-goal']();assert.equal(started(f),true);
});
test('Go automatically restores normal-status localization after a recoverable planning conflict',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;let plans=0;
 f.setResponder((path,data,options,previous)=>{
  if(path==='/api/nav/plan'&&plans++===0)throw Object.assign(Error('internal map overlap'),{navigation:{classification:'map_surface_disagreement',recommended_action:'relocalize_and_replan'}});
  return previous(path);
 });
 await f.ctx.ui['go-goal']();assert.equal(plans,2);assert.equal(started(f),true);
 assert.deepEqual(JSON.parse(JSON.stringify(f.calls.find(r=>r.path==='/api/maps/localize-auto').data)),{verify_current:true});
 assert.equal(f.calls.filter(r=>r.path==='/api/maps/localize-auto').length,1);
});
test('Go performs one observation retry and does not turn an unchanged blocked route into movement',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;
 f.setResponder((path,data,options,previous)=>{
  if(path==='/api/nav/plan')throw Object.assign(Error('insufficient votes'),{navigation:{recommended_action:'revalidate_observation'}});
  return previous(path);
 });
 await assert.rejects(f.ctx.ui['go-goal'](),/等待通路恢复/);
 assert.equal(f.calls.filter(r=>r.path==='/api/nav/plan').length,2);assert.equal(started(f),false);
 assert.equal(f.calls.some(r=>r.path==='/api/maps/localize-auto'),false);
});
test('same active-map sync preserves the chosen world target and updates its cache id automatically',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;f.ctx.S.maps[0].verified_for_robot=false;
 f.ctx.navigationMapReady=()=>f.ctx.S.maps.find(m=>m.id===f.ctx.mapId)?.verified_for_robot;
 f.ctx.navigationMapCanPrepare=()=>true;
 f.setResponder((path,data,options,previous)=>path==='/api/maps/library/sync'?{map:{...f.ctx.S.maps[0],id:'synced',verified_for_robot:true}}:previous(path));
 f.ctx.paintGoalPlan();assert.equal(f.nodes.get('[data-ui="go-goal"]').disabled,false);
 await f.ctx.ui['go-goal']();const plan=f.calls.find(r=>r.path==='/api/nav/plan');
 assert.equal(plan.data.map_id,'synced');assert.equal(plan.data.x,2);assert.equal(plan.data.y,1);
 assert.equal(f.calls.at(-1).data.map_id,'synced');assert.equal(started(f),true);
});
test('a truly different active map is not given the old target coordinates',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;f.ctx.S.maps[0].verified_for_robot=false;f.ctx.navigationMapReady=()=>false;f.ctx.navigationMapCanPrepare=()=>true;
 f.setResponder((path,data,options,previous)=>path==='/api/maps/library'?{active:'other',maps:[{name:'other',grids:[0]}]}:previous(path));
 await assert.rejects(f.ctx.ui['go-goal'](),/当前地图/);
 assert.equal(started(f),false);assert.equal(f.calls.some(r=>r.path==='/api/maps/library/sync'),false);
 assert.equal(f.calls.some(r=>r.path==='/api/maps/localize-auto'),false);assert.equal(f.ctx.points[0].x,2);
});
test('automatic localization recovery cannot late-start after cancellation',async()=>{
 const f=fixture(),gate={};f.ctx.S.pose.Location=0;let plans=0;
 f.setResponder((path,data,options,previous)=>{
  if(path==='/api/nav/plan'&&plans++===0)throw Object.assign(Error('overlap'),{navigation:{recommended_action:'relocalize_and_replan'}});
  if(path.startsWith('/api/operations'))return new Promise(resolve=>gate.resolve=resolve);
  return previous(path);
 });
 const work=f.ctx.ui['go-goal']();await tick();await f.ctx.ui['cancel-nav']();gate.resolve({jobs:{status:'completed',result:{localized:true}}});await work;
 assert.equal(plans,1);assert.equal(started(f),false);
});
test('ordinary patrol defaults to the platform executor and sends its reusable first plan',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;f.ctx.S.routes[0].points=[{x:2,y:1,yaw:1,type:0},{x:3,y:2,yaw:2,type:1}];
 await f.ctx.runRoute('route');const plan=f.calls.find(r=>r.path==='/api/nav/plan'),start=f.calls.find(r=>r.path==='/api/nav/start');
 assert.equal(plan.data.engine,'platform');assert.equal(plan.data.yaw,null);
 assert.equal(start.data.engine,'platform');assert.equal(start.data.plan_id,'NEW');
});
test('explicit native and docking patrols do not pretend to execute a platform-cleaned preview',async()=>{
 for(const docking of [false,true]){
  const f=fixture();f.ctx.S.pose.Location=0;f.ctx.S.routes[0].options.engine=docking?'platform':'native';
  if(docking)f.ctx.S.routes[0].points.push({x:3,y:2,type:3});
  await f.ctx.runRoute('route');assert.equal(f.calls.some(r=>r.path==='/api/nav/plan'),false);
  assert.equal(f.calls.at(-1).data.engine,'native');assert.equal(f.calls.at(-1).data.plan_id,undefined);
 }
});
test('stored patrol keeps its first waypoint while same-map sync canonicalizes the cache id',async()=>{
 const f=fixture();f.ctx.page='tasks';f.ctx.location.hash='#tasks';f.ctx.mapId='different-editor';f.ctx.S.maps[0].verified_for_robot=false;
 let synced=false;
 f.setResponder((path,data,options,previous)=>{
  if(path==='/api/maps/library/sync'){synced=true;return {map:{...f.ctx.S.maps[0],id:'canonical',verified_for_robot:true}};}
  if(path==='/api/platform'){const state=structuredClone(f.ctx.S);if(synced)state.routes[0].map_id='canonical';return state;}
  return previous(path);
 });
 await f.ctx.runRoute('route');const first=f.calls.find(r=>r.path==='/api/nav/plan');
 assert.equal(first.data.map_id,'canonical');assert.equal(first.data.x,2);assert.equal(first.data.y,1);
 assert.equal(f.calls.at(-1).path,'/api/nav/start');assert.equal(f.calls.at(-1).data.id,'route');
 assert.equal(f.calls.at(-1).data.engine,'platform');
});
test('cancel during same-map automatic sync cannot adopt a late map or start navigation',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;f.ctx.S.maps[0].verified_for_robot=false;f.ctx.navigationMapReady=()=>false;f.ctx.navigationMapCanPrepare=()=>true;let complete;
 f.setResponder((path,data,options,previous)=>path==='/api/maps/library/sync'?new Promise(resolve=>complete=resolve):previous(path));
 const work=f.ctx.ui['go-goal']();await tick();await f.ctx.ui['cancel-nav']();complete({map:{...f.ctx.S.maps[0],id:'late-map',verified_for_robot:true}});await work;
 assert.equal(f.ctx.mapId,'map');assert.equal(started(f),false);assert.equal(f.calls.some(r=>r.path==='/api/nav/plan'),false);
});
test('a recoverable patrol planning conflict is handled before the final route start',async()=>{
 const f=fixture();f.ctx.S.pose.Location=0;let planned=0;
 f.setResponder((path,data,options,previous)=>{if(path==='/api/nav/plan'&&planned++===0)throw Object.assign(Error('map overlap'),{navigation:{recommended_action:'relocalize_and_replan'}});return previous(path);});
 await f.ctx.runRoute('route');assert.equal(planned,2);assert.equal(f.calls.filter(r=>r.path==='/api/maps/localize-auto').length,1);
 assert.equal(f.calls.at(-1).path,'/api/nav/start');assert.equal(f.calls.at(-1).data.engine,'platform');
});
