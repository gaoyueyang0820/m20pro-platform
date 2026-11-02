// Local DOM and response fixtures only; no server, robot or production maps.
const {test}=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const tick=()=>new Promise(setImmediate);
const deferred=()=>{let resolve,reject;const promise=new Promise((r,j)=>{resolve=r;reject=j;});return {promise,resolve,reject};};
const esc=v=>String(v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const library=()=>({management_version:1,desktop_management_version:1,catalog_integrity_protocol:1,desktop_catalog_integrity_protocol:1,permanent_delete_supported:true,desktop_permanent_delete_supported:true,active:'base',maps:[{name:'base',active:true,display_name:'Base',grids:[0]},{name:'candidate',display_name:'Candidate',cleanup_base:'base',grids:[0]}]});
function fixture(){
 const nodes={},dialogs=[],jobs=[],calls=[],toasts=[],events={};let closes=0,invalidated=0;
 for(const key of ['#remote-map-list','[data-ui="map-library-edit"]','[data-ui="map-library-trash"]','[data-ui="map-edit-confirm"]','[data-ui="map-purge-confirm"]','[data-ui="hub-confirm-apply"]'])nodes[key]={};
 Object.assign(nodes,{'#modal':{close(){closes++;}},'#map-display-name':{value:'Renamed'},'#map-purge-confirm-text':{value:''},'#map-cleanup-previous':{checked:true}});
 const c=vm.createContext({console,S:{mode:'live',ip:'fixture',server_instance:'server',connection_generation:1,nav:{status:'idle'},autonomy:{state:'idle'},velocity:[0,0,0],mapping:false,sdk:{},maps:[{id:'base-cache',robot_map_name:'base',robot_ip:'fixture',verified_for_robot:true},{id:'candidate-cache',robot_map_name:'candidate',robot_ip:'fixture',verified_for_robot:false}]},
  page:'maps',mapLibrary:library(),selectedRemote:'candidate',mapId:'candidate-cache',routeId:'route',points:[{x:1,y:2}],goalPlan:{old:true},
  cloudViewGeneration:0,cloudData:{old:true},cloudSource:{kind:'map',key:'candidate'},ui:{},$:s=>nodes[s],text:(id,value)=>{nodes['#'+id]??={};nodes['#'+id].textContent=value;},esc,icon:()=>'',sizeLabel:()=>'',empty:(title,body)=>title+' '+body,
  document:{addEventListener:(name,fn)=>(events[name]||=[]).push(fn),querySelectorAll:()=>[]},window:{},update(){},render(){},toast:(...args)=>toasts.push(args),
  modal:(title,body)=>dialogs.push({title,body}),fetchState:async()=>{},previewCloud:async(kind,name)=>calls.push({path:'preview:'+name}),
  invalidateGoalPlan:()=>{invalidated++;c.goalPlan=null;}
 });
 let remote=library(),trash=[],apiResponse=path=>path==='/api/maps/library'?structuredClone(remote):path==='/api/maps/library/trash'?{maps:structuredClone(trash)}:Promise.reject(Error(path));
 let jobResponse=(path,data)=>{
  if(path.endsWith('/delete'))remote.maps=remote.maps.filter(m=>m.name!==data.name);
  if(path.endsWith('/rename')){const row=remote.maps.find(m=>m.name===data.name);if(row)row.display_name=data.display_name;}
  return path.endsWith('/apply')?{ok:true,map:{id:'candidate-cache'},cleanup:{retained:true,message:'保留此前地图'}}:{ok:true,name:data.name};
 };
 c.api=async(path,data)=>{calls.push({path,data});return apiResponse(path,data);};
 c.job=async(path,data,label,isCurrent)=>{jobs.push({path,data,isCurrent});return jobResponse(path,data);};
 vm.runInContext(fs.readFileSync('platform_web/map_library.js','utf8'),c);c.paintMapLibrary();
 return {c,nodes,dialogs,jobs,calls,toasts,events,get closes(){return closes;},get invalidated(){return invalidated;},get remote(){return remote;},set remote(v){remote=v;},set trash(v){trash=v;},get:name=>vm.runInContext(name,c),setAPI:fn=>{const old=apiResponse;apiResponse=(...args)=>fn(...args,old);},setJob:fn=>{const old=jobResponse;jobResponse=(...args)=>fn(...args,old);}};
}

test('same-IP reconnect and server restart invalidate pending map confirmation',async()=>{
 for(const field of ['connection_generation','server_instance']){
  const f=fixture();f.c.editMapDialog('candidate','delete');f.c.S[field]=field==='connection_generation'?2:'another-server';
  await assert.rejects(f.c.ui['map-edit-confirm'](),/连接已变化/);assert.equal(f.jobs.length,0);
 }
});
test('switching away and back cannot revive a prior confirmation',async()=>{
 const f=fixture();f.c.editMapDialog('candidate','delete');f.c.S.ip='other';f.get('mapScope()');f.c.S.ip='fixture';
 await assert.rejects(f.c.ui['map-edit-confirm'](),/连接已变化/);assert.equal(f.jobs.length,0);
});
test('late old library response never overwrites the newer connection list',async()=>{
 const f=fixture(),old=deferred(),fresh=deferred();let n=0;f.setAPI(path=>path==='/api/maps/library'?(++n===1?old.promise:fresh.promise):{maps:[]});
 const a=f.c.loadMapLibrary();f.c.S.connection_generation++;const b=f.c.loadMapLibrary();
 const newer={...library(),active:'new-map',maps:[{name:'new-map',active:true,grids:[0]}]};fresh.resolve(newer);await b;
 old.resolve(library());await a;assert.equal(f.c.mapLibrary.active,'new-map');assert.equal(f.c.selectedRemote,'new-map');
});
test('an in-flight list from before deletion cannot resurrect the removed map',async()=>{
 const f=fixture(),old=deferred();let n=0;f.setAPI((path,data,previous)=>path==='/api/maps/library'&&++n===1?old.promise:previous(path,data));
 const read=f.c.loadMapLibrary();f.c.editMapDialog('candidate','delete');await f.c.ui['map-edit-confirm']();
 old.resolve(library());await read;assert.equal(f.c.mapLibrary.maps.some(m=>m.name==='candidate'),false);
 assert.equal(f.c.mapId,'base-cache');assert.equal(f.c.points.length,0);assert.equal(f.c.routeId,null);assert.ok(f.invalidated);
 assert.equal(f.c.S.maps.some(m=>m.id==='candidate-cache'),false);
});
test('old operation completion cannot release a newer operation busy state',async()=>{
 const f=fixture(),old=deferred(),fresh=deferred();let n=0;f.setJob(()=>++n===1?old.promise:fresh.promise);
 f.c.editMapDialog('candidate','rename');const a=f.c.ui['map-edit-confirm']();
 f.c.S.connection_generation++;await f.c.loadMapLibrary();f.c.editMapDialog('candidate','rename');const b=f.c.ui['map-edit-confirm']();
 old.resolve({ok:true});await a;assert.equal(f.get('mapEditBusy'),true);assert.equal(f.jobs[0].isCurrent(),false);
 fresh.resolve({ok:true});await b;assert.equal(f.get('mapEditBusy'),false);
});
test('completion does not close a newer unrelated dialog',async()=>{
 const f=fixture(),gate=deferred();f.setJob(()=>gate.promise);f.c.editMapDialog('candidate','rename');const work=f.c.ui['map-edit-confirm']();
 f.c.modal('Other task','Unrelated confirmation');const before=f.closes;gate.resolve({ok:true});await work;
 assert.equal(f.closes,before);assert.equal(f.dialogs.at(-1).title,'Other task');
});
test('navigation, resumable capture, greeting and map learning block mutation and recheck confirmation',async()=>{
 const cases=[s=>s.nav.status='running',s=>s.autonomy.can_resume=true,s=>s.mapping=true,s=>s.greeting={active:true,motion:'nod'},s=>s.map_learning={enabled:true},s=>s.background_operation='localize'];
 for(const change of cases){
  const f=fixture();f.c.ui['map-library-edit']();f.c.editMapDialog('candidate','delete');change(f.c.S);f.c.update();
  assert.equal(f.nodes['[data-ui="map-library-edit"]'].disabled,false);assert.equal(f.nodes['[data-ui="map-edit-confirm"]'].disabled,true);
  await assert.rejects(f.c.ui['map-edit-confirm']());assert.equal(f.jobs.length,0);
 }
});
test('display names remain editable during navigation, capture and SDK output',async()=>{
 const f=fixture();f.c.S.nav.status='running';f.c.S.mapping=true;f.c.S.sdk.armed=true;f.c.S.map_learning={enabled:true};
 f.c.update();assert.equal(f.nodes['[data-ui="map-library-edit"]'].disabled,false);
 f.c.ui['map-library-edit']();assert.match(f.nodes['#remote-map-list'].innerHTML,/data-map-rename="candidate" >/);
 f.c.editMapDialog('candidate','rename');f.c.update();assert.equal(f.nodes['[data-ui="map-edit-confirm"]'].disabled,false);
 await f.c.ui['map-edit-confirm']();assert.equal(f.jobs[0].path,'/api/maps/library/rename');assert.equal(f.c.S.nav.status,'running');
});
test('rename keeps map and route identities and escapes label text in rows and dialogs',async()=>{
 const f=fixture(),label='<img src=x onerror="bad()"> & \'';f.c.mapLibrary.maps[1].display_name=label;f.c.paintMapLibrary();
 assert.doesNotMatch(f.nodes['#remote-map-list'].innerHTML,/<img/);assert.match(f.nodes['#remote-map-list'].innerHTML,/&lt;img/);
 f.c.editMapDialog('candidate','rename');assert.doesNotMatch(f.dialogs.at(-1).body,/<img/);
 f.nodes['#map-display-name'].value='New display name';await f.c.ui['map-edit-confirm']();
 assert.equal(f.jobs[0].data.name,'candidate');assert.equal(f.jobs[0].data.display_name,'New display name');assert.equal(f.c.mapId,'candidate-cache');assert.equal(f.c.routeId,'route');assert.equal(f.c.points.length,1);
});
test('cleanup request is bound to the specifically confirmed direct base',async()=>{
 const f=fixture();f.c.openMapActivation('candidate');f.c.mapLibrary.active='unrelated';await f.c.ui['hub-confirm-apply']();
 assert.equal(f.jobs[0].data.cleanup_previous,true);assert.equal(f.jobs[0].data.cleanup_base,'base');
 assert.match(f.toasts.at(-1)[0],/保留/);
 const other=fixture();other.c.mapLibrary.active='unrelated';other.c.openMapActivation('candidate');await other.c.ui['hub-confirm-apply']();
 assert.equal(other.jobs[0].data.cleanup_previous,false);assert.equal(other.jobs[0].data.cleanup_base,undefined);
});
test('a late trash response cannot replace a newer dialog',async()=>{
 const f=fixture(),gate=deferred();f.setAPI((path,data,previous)=>path.endsWith('/trash')?gate.promise:previous(path,data));
 const work=f.c.ui['map-library-trash']();f.c.editMapDialog('candidate','rename');gate.resolve({maps:[{id:'trash',name:'old'}]});await work;
 assert.equal(f.dialogs.at(-1).title,'重命名地图');
});
test('partial permanent deletion cannot be restored and offers explicit retry',async()=>{
 const f=fixture();f.trash=[{id:'partial',name:'old',state:'purge_failed',recoverable:false,purge_retryable:true}];await f.c.ui['map-library-trash']();
 assert.match(f.dialogs.at(-1).body,/删除未完成/);assert.match(f.dialogs.at(-1).body,/data-map-restore="partial" disabled/);assert.match(f.dialogs.at(-1).body,/重试彻底删除/);
 f.c.purgeMapDialog('partial');assert.equal(f.nodes['[data-ui="map-purge-confirm"]'].disabled,false);
 assert.doesNotMatch(f.dialogs.at(-1).body,/请输入/);await f.c.ui['map-purge-confirm']();
 assert.deepEqual(JSON.parse(JSON.stringify(f.jobs[0].data)),{id:'partial',confirmed_name:'old'});
});
test('purge failure reloads partial state instead of leaving a recoverable stale row',async()=>{
 const f=fixture();f.trash=[{id:'partial',name:'old'}];await f.c.ui['map-library-trash']();f.c.purgeMapDialog('partial');f.nodes['#map-purge-confirm-text'].value='删除';
 f.setJob(()=>{f.trash=[{id:'partial',name:'old',state:'purge_failed',recoverable:false,purge_retryable:true}];throw Error('synthetic partial delete');});
 await assert.rejects(f.c.ui['map-purge-confirm'](),/partial delete/);assert.match(f.dialogs.at(-1).body,/删除未完成/);assert.match(f.dialogs.at(-1).body,/data-map-restore="partial" disabled/);
});
test('late activation response cannot retarget maps after a reconnect',async()=>{
 const f=fixture(),gate=deferred();f.setJob(()=>gate.promise);f.c.openMapActivation('candidate');const work=f.c.ui['hub-confirm-apply']();
 f.c.S.connection_generation++;f.c.mapId='new-connection-map';f.c.points=[{x:7,y:8}];f.c.update();gate.resolve({map:{id:'late-map'}});await work;
 assert.equal(f.c.mapId,'new-connection-map');assert.equal(f.c.points[0].x,7);assert.equal(f.jobs[0].isCurrent(),false);
});
test('cold-start library reads tolerate absent status but never enable mutations',async()=>{
 const f=fixture();f.c.S=null;await f.c.loadMapLibrary();f.c.paintMapLibrary();
 assert.equal(f.nodes['[data-ui="map-library-edit"]'].disabled,true);
 assert.throws(()=>f.c.editMapDialog('candidate','delete'),/等待平台状态/);assert.equal(f.jobs.length,0);
});
test('unfinished purging journal follows backend retryability while never enabling restore',async()=>{
 for(const retryable of [true,false]){
  const f=fixture();f.trash=[{id:'journal',name:'old',state:'purging',recoverable:false,purge_retryable:retryable}];await f.c.ui['map-library-trash']();
  assert.match(f.dialogs.at(-1).body,/data-map-restore="journal" disabled/);
  if(retryable){
   assert.match(f.dialogs.at(-1).body,/检查并重试删除/);assert.doesNotMatch(f.dialogs.at(-1).body,/data-map-purge="journal" disabled/);
   f.c.purgeMapDialog('journal');f.nodes['#map-purge-confirm-text'].value='删除';await f.c.ui['map-purge-confirm']();assert.equal(f.jobs[0].data.id,'journal');
  }else{assert.match(f.dialogs.at(-1).body,/data-map-purge="journal" disabled/);assert.throws(()=>f.c.purgeMapDialog('journal'));assert.equal(f.jobs.length,0);}
 }
});
