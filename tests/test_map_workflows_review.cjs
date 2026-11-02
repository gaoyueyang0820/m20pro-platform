// Actual async workflow functions with local replies; no server or robot calls.
const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const deferred=()=>{let resolve,reject;const promise=new Promise((r,j)=>{resolve=r;reject=j;});return {promise,resolve,reject};};
const tick=()=>new Promise(setImmediate);
function fixture(){
 const calls=[],dialogs=[],events={},nodes={'#modal':{close(){}},'#start-map-name':{value:'synthetic'},'#extension-base':{value:'base'},'#remote-grid-id':{value:'0'}};
 let api=async()=>({}),library=async()=>({maps:c.mapLibrary.maps});
 const c=vm.createContext({console,S:{mode:'live',ip:'fixture',server_instance:'server',connection_generation:1,nav:{status:'idle'},velocity:[0,0,0],mapping:false,map_learning:{grid_id:0}},
  page:'maps',mapId:'original',selectedRemote:'base',routeId:'route',points:[{x:1,y:2}],ui:{},mapDialogGeneration:0,
  mapLibrary:{active:'base',maps:[{name:'base',grids:[0]},{name:'candidate',grids:[0]}]},lastBuildEnd:'',buildLoading:false,liveCloudTopic:'old',
  $:s=>nodes[s],text(){},html(){},esc:String,field:()=>'',select:()=>'',showCloud(){},paintMapLibrary(){},
  toast:(...args)=>calls.push(['toast',...args]),render(){},navigate:p=>{c.page=p;calls.push(['navigate',p]);},
  selectedMap:()=>({robot_map_name:'base',grid_id:0}),previewCloud:async(...args)=>calls.push(['preview',...args]),
  fetchState:async()=>calls.push(['state']),modal:(...args)=>{c.mapDialogGeneration++;dialogs.push(args);},
  api:async(...args)=>{calls.push(['api',...args]);return api(...args);},
  loadMapLibrary:async()=>{calls.push(['library']);return library();},
  job:async(...args)=>{calls.push(['job',...args]);return {ok:true};},
  document:{addEventListener:(name,fn)=>(events[name]||=[]).push(fn)}});
 c.ui['cloud-live']=async()=>calls.push(['cloud-live']);
 const workspace=fs.readFileSync('platform_web/workspace.js','utf8');
 vm.runInContext(workspace.slice(workspace.indexOf('function navigationMapScope()'),workspace.indexOf("ui['apply-selected-map']=")),c);
 const workflow=fs.readFileSync('platform_web/workflows.js','utf8');
 vm.runInContext(workflow.slice(workflow.indexOf('let buildState='),workflow.indexOf("ui['map-build-status']=loadBuildStatus;")+"ui['map-build-status']=loadBuildStatus;".length),c);
 vm.runInContext(workflow.slice(workflow.indexOf('let workflowStartMapIntent='),workflow.indexOf('const workflowMotionUpdate=')),c);
 const learning=fs.readFileSync('platform_web/map_learning.js','utf8');
 const start=learning.indexOf("document.addEventListener('click',async e=>{const b=e.target.closest('[data-learning-apply]')");
 vm.runInContext(learning.slice(start,learning.indexOf('let goalPlan=',start)),c);
 c.openMapActivation=(...args)=>calls.push(['activate-dialog',...args]);
 return {c,calls,dialogs,events,setAPI:f=>{api=f;},setLibrary:f=>{library=f;},get:name=>vm.runInContext(name,c)};
}
test('a late map sync cannot change the next connection selection',async()=>{
 const f=fixture(),gate=deferred();f.setAPI(()=>gate.promise);
 const work=f.c.syncNavigationMap('base',0);f.c.S.connection_generation++;
 gate.resolve({map:{id:'stale'}});assert.equal(await work,null);assert.equal(f.c.mapId,'original');
 assert.equal(f.calls.filter(r=>r[0]==='state').length,0);
});
test('a newer sync owns the result even on the same connection',async()=>{
 const f=fixture(),old=deferred(),fresh=deferred();let n=0;f.setAPI(()=>++n===1?old.promise:fresh.promise);
 const a=f.c.syncNavigationMap('base'),b=f.c.syncNavigationMap('candidate');
 fresh.resolve({map:{id:'new'}});await b;old.resolve({map:{id:'old'}});assert.equal(await a,null);assert.equal(f.c.mapId,'new');
});
test('old build status cannot select a new-connection candidate',async()=>{
 const f=fixture(),gate=deferred();f.setAPI(()=>gate.promise);
 const work=f.c.loadBuildStatus();f.c.S.connection_generation++;
 gate.resolve({phase:'saved',result_maps:['candidate']});await work;
 assert.equal(f.get('buildState'),null);assert.equal(f.c.selectedRemote,'base');assert.equal(f.calls.filter(r=>r[0]==='library').length,0);
});
test('old build finally does not unlock a newer status request',async()=>{
 const f=fixture(),old=deferred(),fresh=deferred();let n=0;f.setAPI(()=>++n===1?old.promise:fresh.promise);
 const a=f.c.loadBuildStatus();f.c.S.connection_generation++;const b=f.c.loadBuildStatus();
 old.resolve({phase:'idle'});await a;assert.equal(f.c.buildLoading,true);
 fresh.resolve({phase:'collecting'});await b;assert.equal(f.c.buildLoading,false);assert.equal(f.get('buildState.phase'),'collecting');
});
test('saved status rechecks ownership after awaiting the map list',async()=>{
 const f=fixture(),gate=deferred();f.setAPI(async()=>({phase:'saved',result_maps:['candidate']}));f.setLibrary(()=>gate.promise);
 const work=f.c.loadBuildStatus();await tick();f.c.S.server_instance='replacement';gate.resolve({maps:f.c.mapLibrary.maps});await work;
 assert.equal(f.c.selectedRemote,'base');assert.equal(f.c.lastBuildEnd,'');assert.ok(!f.calls.some(r=>r[0]==='preview'));
});
test('an extension capability reply cannot open a new-connection confirmation',async()=>{
 const f=fixture(),gate=deferred();f.setAPI(()=>gate.promise);
 const work=f.c.ui['extend-map']();await tick();f.c.S.connection_generation++;
 gate.resolve({extension_ready:true,pipeline_version:2});await work;assert.equal(f.dialogs.length,0);
});
test('extension confirmation is bound to its original connection',async()=>{
 const f=fixture();f.setAPI(async()=>({extension_ready:true,pipeline_version:2}));await f.c.ui['extend-map']();
 f.c.S.connection_generation++;await assert.rejects(f.c.ui['extension-start'](),/连接|确认/);
 assert.ok(!f.calls.some(r=>r[0]==='job'));
});
test('start here creates one automatically named mapping request without a confirmation dialog',async()=>{
 const f=fixture();await f.c.ui['start-map-here']();
 const jobs=f.calls.filter(r=>r[0]==='job');assert.equal(jobs.length,1);assert.equal(jobs[0][1],'/api/maps/start-here');assert.match(jobs[0][2].name,/^m20-map-\d+$/);assert.equal(f.dialogs.length,0);
});
test('duplicate start-here clicks share preparation and a late reply cannot redirect a new connection',async()=>{
 const f=fixture(),gate=deferred();f.c.job=async(...args)=>{f.calls.push(['job',...args]);return gate.promise;};
 const work=f.c.ui['start-map-here']();await f.c.ui['start-map-here']();assert.equal(f.calls.filter(r=>r[0]==='job').length,1);
 f.c.S.connection_generation++;gate.resolve({ok:true});await work;
 assert.equal(f.calls.filter(r=>r[0]==='navigate'||r[0]==='cloud-live').length,0);
});
test('a late learning-list request cannot activate the same named map on another connection',async()=>{
 const f=fixture(),gate=deferred();f.setLibrary(()=>gate.promise);
 const event={target:{closest:()=>({dataset:{learningApply:'candidate'}})}};
 const work=f.events.click[0](event);f.c.S.connection_generation++;gate.resolve({maps:f.c.mapLibrary.maps});await work;
 assert.ok(!f.calls.some(r=>r[0]==='activate-dialog'));
});
