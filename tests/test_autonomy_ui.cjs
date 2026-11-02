// Mapping modes/ranges: local fixtures only, no server or robot requests.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');

function fixture(){
 const nodes=new Map(),calls=[],listeners={},intervals=[];
 for(const [id,value] of Object.entries({'auto-map-kind':'extend','auto-scope':'map','auto-radius':'6','auto-minutes':'5','auto-forward':'35','auto-yaw':'70','auto-start':'','auto-relocalize':'','auto-scope-field':'','auto-map-help':'','auto-message':'','auto-state':''})){
  const field={hidden:false};nodes.set(id,{value,hidden:false,disabled:false,textContent:'',closest:()=>field,field});
 }
 const lookup=selector=>nodes.get(selector.startsWith('#')?selector.slice(1):selector.match(/data-ui="([^"]+)"/)?.[1]);
 const c=vm.createContext({console,pages:{},paths:{spark:''},page:'autonomy',S:{mode:'live',ip:'fixture',online:true,pose:{Location:0},maps:[],mapping:false,autonomy:{state:'idle',active:false}},mapLibrary:{active:'room'},ui:{},
  $:lookup,text:(id,value)=>{if(nodes.has(id))nodes.get(id).textContent=value;},html(){},fmt:v=>String(v??''),esc:v=>String(v??''),icon:()=>'',heading:()=>'',btn:(name,id)=>`<button data-ui="${id}">${name}</button>`,field:(name,id,value)=>`<label>${name}<input id="${id}" value="${value}"></label>`,
  update(){},render(){},renderNav(){},navigate(){},toast(){},fetchState:async()=>{},
  api:async(path,data)=>{calls.push({path,data});return path==='/api/maps/library'?{active:c.mapLibrary.active}:{ok:true};},
  window:{motion:()=>'',mapsPage:()=>'',navPage:()=>'',addEventListener:(name,fn)=>{(listeners[name]||=[]).push(fn);}},location:{hash:'#autonomy'},setInterval(fn){intervals.push(fn);},setTimeout,clearTimeout,AbortController,performance,
  document:{hidden:false,querySelector:lookup,getElementById:id=>nodes.get(id),addEventListener:(name,fn)=>{(listeners[name]||=[]).push(fn);}}
 });
 vm.runInContext(fs.readFileSync('platform_web/autonomy.js','utf8'),c);
 return {c,nodes,calls,listeners,intervals,change:id=>{for(const fn of listeners.change||[])fn({target:{id}});}};
}
const tick=()=>new Promise(setImmediate);
function apiFixture(f,responder){f.c.api=async(path,data,quiet,options)=>{f.calls.push({path,data,options});return responder(path,data,options);};}
function localizationResult(status='completed'){return {jobs:{status,message:'定位已收敛',result:{localized:true}}};}

test('map method and exploration range are separate controls without duplicate methods',()=>{
 const f=fixture(),markup=f.c.autonomyPage();
 const method=markup.match(/<select id="auto-map-kind">([\s\S]*?)<\/select>/)[1];
 assert.deepEqual([...method.matchAll(/value="([^"]+)"/g)].map(m=>m[1]),['new','extend','continue']);
 assert.match(method,/更新当前地图（补扫＋优化）/);assert.doesNotMatch(method,/整层探索|续建当前活动地图/);
 const scope=markup.match(/<select id="auto-scope">([\s\S]*?)<\/select>/)[1];
 assert.match(scope,/按地图范围/);assert.match(scope,/指定半径/);
 assert.doesNotMatch(markup,/当前房间|整层楼|调试|待验证|电脑端功能已接入/);
});
test('defaults to current-map update, current capture, or new map as available',()=>{
 const f=fixture();assert.equal(f.c.autoDefaultMapKind(),'extend');
 f.c.S.mapping=true;assert.equal(f.c.autoDefaultMapKind(),'continue');
 f.c.S.mapping=false;f.c.mapLibrary.active=null;assert.equal(f.c.autoDefaultMapKind(),'new');
 f.c.S.maps=[{verified_for_robot:true,robot_ip:'fixture'}];assert.equal(f.c.autoDefaultMapKind(),'extend');
 f.c.S.maps[0].robot_ip='another-robot';assert.equal(f.c.autoDefaultMapKind(),'new');
});
test('map-range update hides the unused radius and explains its coverage',()=>{
 const f=fixture();f.nodes.get('auto-scope').value='map';f.change('auto-scope');
 assert.equal(f.nodes.get('auto-scope-field').hidden,false);
 assert.equal(f.nodes.get('auto-radius').field.hidden,true);assert.equal(f.nodes.get('auto-radius').disabled,true);
 assert.match(f.nodes.get('auto-map-help').textContent,/活动地图范围/);
 assert.match(f.nodes.get('auto-map-help').textContent,/优先补扫未扫描区域.*更新已扫描区域/);
 assert.equal(f.calls.length,0);
});
test('new mapping uses an explicit radius without sending commands',()=>{
 const f=fixture();f.nodes.get('auto-scope').value='map';f.nodes.get('auto-map-kind').value='new';f.change('auto-map-kind');
 assert.equal(f.nodes.get('auto-scope-field').hidden,true);assert.equal(f.nodes.get('auto-scope').value,'radius');
 assert.equal(f.nodes.get('auto-radius').field.hidden,false);assert.equal(f.nodes.get('auto-radius').disabled,false);
 assert.match(f.nodes.get('auto-map-help').textContent,/独立的新地图/);assert.equal(f.calls.length,0);
});
test('radius and map-range updates use the same extension workflow',async()=>{
 for(const scope of ['radius','map']){
  const f=fixture();f.nodes.get('auto-scope').value=scope;await f.c.ui['auto-start']();
  assert.deepEqual(f.calls.map(r=>r.path),['/api/maps/library','/api/autonomy/start']);
  const req=f.calls[1].data;assert.equal(req.extend,true);assert.equal(req.base,'room');assert.equal(req.scope,scope);
  assert.equal(req.forward_limit,.35);assert.equal(req.yaw_limit,.70);assert.equal(req.minutes,5);
 }
});
test('new and continued collection submit radius scope without extension requests',async()=>{
 for(const kind of ['new','continue']){
  const f=fixture();f.c.S.mapping=kind==='continue';f.nodes.get('auto-map-kind').value=kind;f.nodes.get('auto-scope').value='map';
  await f.c.ui['auto-start']();assert.deepEqual(f.calls.map(r=>r.path),['/api/autonomy/start']);
  const req=f.calls[0].data;assert.equal(req.scope,'radius');assert.equal(req.extend,undefined);assert.equal(req.use_current,kind==='continue');
 }
});
test('missing active map or missing current collection does not silently create a new map',async()=>{
 const f=fixture();f.c.mapLibrary.active=null;await assert.rejects(f.c.ui['auto-start'](),/启用要更新/);
 assert.deepEqual(f.calls.map(r=>r.path),['/api/maps/library']);
 f.calls.length=0;f.nodes.get('auto-map-kind').value='continue';await assert.rejects(f.c.ui['auto-start'](),/没有正在采集/);assert.equal(f.calls.length,0);
});
test('running and saving states lock the method, scope and motion inputs together',()=>{
 for(const state of ['exploring','saving']){
  const f=fixture();f.c.S.autonomy={active:state==='exploring',state};f.c.paintAutoMapOptions();
  for(const id of ['auto-start','auto-map-kind','auto-scope','auto-radius','auto-minutes','auto-forward','auto-yaw'])assert.equal(f.nodes.get(id).disabled,true,id);
 }
});

test('update automatically localizes before starting and reports preparation status',async()=>{
 const f=fixture();f.c.S.pose.Location=1;f.nodes.get('auto-scope').value='map';let finish;
 apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?{job_id:'loc-1'}:path.startsWith('/api/operations')?new Promise(r=>finish=r):{ok:true});
 const pending=f.c.ui['auto-start']();await tick();
 assert.equal(f.nodes.get('auto-start').disabled,true);assert.equal(f.nodes.get('auto-state').textContent,'恢复定位中');
 assert.match(f.nodes.get('auto-message').textContent,/恢复定位/);assert.equal(f.calls.some(r=>r.path==='/api/autonomy/start'),false);
 finish(localizationResult());await pending;
 assert.deepEqual(f.calls.map(r=>r.path),['/api/maps/library','/api/maps/localize-auto','/api/operations?id=loc-1','/api/autonomy/start']);
 assert.equal(f.calls.at(-1).data.scope,'map');assert.equal(f.calls.at(-1).data.extend,true);
 assert.equal(f.nodes.get('auto-start').disabled,false);
});
test('failed localization releases busy state and never starts movement',async()=>{
 const f=fixture();f.c.S.pose.Location=1;
 apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?{job_id:'loc-1'}:{jobs:{status:'failed',error:'位置无法唯一识别'}});
 await assert.rejects(f.c.ui['auto-start'](),/位置无法唯一识别/);
 assert.equal(f.calls.some(r=>r.path==='/api/autonomy/start'),false);assert.equal(f.nodes.get('auto-start').disabled,false);
 assert.match(f.nodes.get('auto-message').textContent,/未开始自主移动/);
});
test('an accepted request without confirmed localization is not treated as success',async()=>{
 const f=fixture();f.c.S.pose.Location=1;
 apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?{job_id:'loc-1'}:{jobs:{status:'completed',result:{accepted:true}}});
 await assert.rejects(f.c.ui['auto-start'](),/尚未确认定位恢复/);
 assert.equal(f.calls.some(r=>r.path==='/api/autonomy/start'),false);
});
test('pause during localization request prevents a late acceptance from starting or polling',async()=>{
 const f=fixture();f.c.S.pose.Location=1;let finish;
 apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?new Promise(r=>finish=r):{ok:true});
 const pending=f.c.ui['auto-start']();await tick();await f.c.ui['auto-stop']();finish({job_id:'late'});await pending;
 assert.deepEqual(f.calls.map(r=>r.path),['/api/maps/library','/api/maps/localize-auto','/api/autonomy/stop']);
 assert.match(f.nodes.get('auto-message').textContent,/搜索可能仍在进行/);
});
test('manual takeover aborts the read but a late completed job still cannot start motion',async()=>{
 const f=fixture();f.c.S.pose.Location=1;let finish,signal;
 apiFixture(f,(path,data,options)=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?{job_id:'loc-1'}:path.startsWith('/api/operations')?new Promise(r=>{finish=r;signal=options.signal;}):{ok:true});
 const pending=f.c.ui['auto-start']();await tick();await f.c.ui['auto-manual']();assert.equal(signal.aborted,true);
 finish(localizationResult());await pending;assert.equal(f.calls.some(r=>r.path==='/api/autonomy/start'),false);
});
test('changing method or scope invalidates a pending localization result',async()=>{
 for(const id of ['auto-map-kind','auto-scope']){
  const f=fixture();f.c.S.pose.Location=1;let finish;
  apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?{job_id:'loc-1'}:new Promise(r=>finish=r));
  const pending=f.c.ui['auto-start']();await tick();f.nodes.get(id).value=id==='auto-scope'?'radius':'new';f.change(id);
  finish(localizationResult());await pending;assert.equal(f.calls.some(r=>r.path==='/api/autonomy/start'),false,id);
 }
});
test('leaving then returning to the page cannot revive an old start intention',async()=>{
 const f=fixture();f.c.S.pose.Location=1;let finish;
 apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?{job_id:'loc-1'}:new Promise(r=>finish=r));
 const pending=f.c.ui['auto-start']();await tick();f.c.page='motion';f.c.location.hash='#motion';f.c.render();f.c.page='autonomy';f.c.location.hash='#autonomy';f.c.render();
 finish(localizationResult());await pending;assert.equal(f.calls.some(r=>r.path==='/api/autonomy/start'),false);
});
test('a duplicate click cannot launch a second localization; late old failure cannot clear a newer intention',async()=>{
 const f=fixture();f.c.S.pose.Location=1;let rejectOld,finishNew;let n=0;
 apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/maps/localize-auto'?{job_id:'loc-'+(++n)}:path.includes('loc-1')?new Promise((r,j)=>rejectOld=j):path.includes('loc-2')?new Promise(r=>finishNew=r):{ok:true});
 const old=f.c.ui['auto-start']();await tick();await f.c.ui['auto-start']();assert.equal(n,1);
 await f.c.ui['auto-stop']();const current=f.c.ui['auto-start']();await tick();rejectOld(Error('late failure'));await old;
 assert.equal(f.nodes.get('auto-start').disabled,true);assert.match(f.nodes.get('auto-message').textContent,/恢复定位/);
 finishNew(localizationResult());await current;assert.equal(f.calls.filter(r=>r.path==='/api/autonomy/start').length,1);
});

test('pending backend cleanup disables restart without disabling manual stop',()=>{
 const f=fixture();f.c.S.autonomy={active:false,state:'paused',cleanup_pending:true,can_start:false};f.c.paintAutoMapOptions();
 assert.equal(f.nodes.get('auto-start').disabled,true);assert.equal(f.nodes.get('auto-relocalize').disabled,true);
 assert.equal(typeof f.c.ui['auto-stop'],'function');assert.equal(typeof f.c.ui['auto-manual'],'function');
});
test('unresolved preparation is not offered as startable and old snapshots remain compatible',()=>{
 const f=fixture();f.c.S.autonomy={active:false,state:'paused',can_start:false,preparation_unresolved:true};f.c.paintAutoMapOptions();assert.equal(f.nodes.get('auto-start').disabled,true);
 f.c.S.autonomy={active:false,state:'idle'};f.c.paintAutoMapOptions();assert.equal(f.nodes.get('auto-start').disabled,false);
});

test('paused owned exploration exposes resume and active confirmed capture can discard in one click',async()=>{
 const f=fixture();for(const id of ['auto-resume','auto-save','auto-discard'])f.nodes.set(id,{hidden:true,disabled:false});
 f.c.S.mapping=true;f.c.S.autonomy={mode:'explore',active:false,state:'paused',can_resume:true,can_start:false,can_discard:true,discard_supported:true,capture_id:'a'.repeat(32),control_generation:4};
 f.c.paintAutoMapOptions();assert.equal(f.nodes.get('auto-resume').hidden,false);assert.equal(f.nodes.get('auto-resume').disabled,false);
 assert.equal(f.nodes.get('auto-start').disabled,true);assert.equal(f.nodes.get('auto-map-kind').disabled,true);
 f.c.S.autonomy={...f.c.S.autonomy,active:true,state:'exploring',can_resume:false};f.c.paintAutoMapOptions();
 assert.equal(f.nodes.get('auto-discard').hidden,false);assert.equal(f.nodes.get('auto-discard').disabled,false);
 await f.c.ui['auto-discard']();assert.deepEqual(f.calls.map(v=>v.path),['/api/autonomy/discard']);assert.equal(f.calls[0].data.capture_id,'a'.repeat(32));
});
test('unsupported discard stays hidden and lifecycle requests cannot double submit',async()=>{
 const f=fixture();for(const id of ['auto-resume','auto-save','auto-discard'])f.nodes.set(id,{hidden:false,disabled:false});
 f.c.S.mapping=true;f.c.S.autonomy={state:'paused',can_resume:true,can_discard:false,discard_supported:false,capture_id:'a'.repeat(32),control_generation:7};f.c.paintAutoMapOptions();
 assert.equal(f.nodes.get('auto-discard').hidden,true);
 let finish;apiFixture(f,(path)=>path==='/api/autonomy/resume'?new Promise(resolve=>finish=resolve):{ok:true});
 const first=f.c.ui['auto-resume']();await tick();await f.c.ui['auto-resume']();assert.equal(f.calls.length,1);assert.equal(f.calls[0].data.control_generation,7);
 finish({ok:true});await first;
});
test('paused or background client cannot submit a late resume after the preparatory refresh',async()=>{
 const f=fixture();f.c.S.mapping=true;f.c.S.autonomy={state:'paused',can_resume:true,capture_id:'a'.repeat(32),control_generation:9};
 let refresh;f.c.fetchState=()=>new Promise(resolve=>refresh=resolve);
 const request=f.c.ui['auto-resume']();await tick();f.c.fetchState=async()=>{};await f.c.ui['auto-stop']();refresh();await request;
 assert.equal(f.calls.some(v=>v.path==='/api/autonomy/resume'),false);assert.equal(f.calls.filter(v=>v.path==='/api/autonomy/stop').length,1);
});


test('active background jobs do not require browser heartbeats or pause on visibility changes',async()=>{
 const f=fixture();f.c.page='motion';f.c.S.autonomy={active:true,run_id:'active-job'};f.c.S.nav={status:'running',engine:'platform',run_id:'active-nav'};
 for(const interval of f.intervals)interval();await tick();assert.equal(f.calls.length,0);
 f.c.document.hidden=true;for(const fn of f.listeners.visibilitychange||[])fn();
 assert.equal(f.calls.length,0);assert.equal(f.c.S.autonomy.active,true);assert.equal(f.c.S.nav.status,'running');
});
test('hiding the page after dispatch does not claim the accepted start was cancelled',async()=>{
 const f=fixture();let finish;
 apiFixture(f,path=>path==='/api/maps/library'?{active:'room'}:path==='/api/autonomy/start'?new Promise(resolve=>finish=resolve):{ok:true});
 const pending=f.c.ui['auto-start']();await tick();assert.equal(f.calls.at(-1).path,'/api/autonomy/start');
 f.c.document.hidden=true;for(const fn of f.listeners.visibilitychange||[])fn();finish({ok:true});await pending;
 assert.equal(f.calls.filter(v=>v.path==='/api/autonomy/stop').length,0);assert.doesNotMatch(f.nodes.get('auto-message').textContent,/取消/);
});
