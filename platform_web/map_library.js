'use strict';

let mapEditMode=false,mapEditBusy=false,mapSearch='',mapEditScope=null,mapTrashScope=null,mapTrashRows=[];
let mapScopeGeneration=0,mapObservedConnection=null,mapListGeneration=0,mapListPending=null,mapListError='';
let mapOperation=null,mapDialogGeneration=0,mapOwnedDialog=null,mapOwnedAction=null,mapTrashRequest=0;
const mapScope=()=>{const connection=JSON.stringify([S?.mode,S?.ip||'',S?.server_instance,S?.connection_generation]);if(mapObservedConnection!==null&&mapObservedConnection!==connection)mapScopeGeneration++;mapObservedConnection=connection;return connection+'|'+mapScopeGeneration;};
const mapLabel=m=>m.display_name||m.name;
const previousMapModal=modal;
modal=function(...args){mapDialogGeneration++;mapOwnedDialog=null;mapOwnedAction=null;return previousMapModal.apply(this,args);};
function mapMutationReason(action){
 if(!S)return '等待平台状态，请稍后再编辑地图';
 if(action==='rename')return '';
 const a=S.autonomy||{};
 if(['running','cancelling'].includes(S.nav?.status))return '请先结束导航并确认停稳';
 if(a.active||a.can_resume||a.cleanup_pending||a.preparation_unresolved||['saving','discarding','preparing'].includes(a.state)||S.mapping)return '请先完成、保存或放弃当前建图任务';
 if(S.greeting?.active&&S.greeting.motion)return '请先停止问候动作';
 if(S.sdk?.armed||S.sdk?.vendor_running)return '请先结束 SDK 输出';
 if(S.velocity?.some(v=>v!==0))return '请先停止移动';
 if(S.map_learning?.enabled||S.map_learning?.state==='generating')return '请先关闭行走地图更新并等待当前版本保存';
 if(S.background_operation)return '后台操作正在进行，请等待完成';
 if(typeof window!=='undefined'&&window.M20NavigationPreparation?.state)return '请先结束导航准备或静态定位';
 return '';
}
function invalidateMapList(){mapListGeneration++;mapListPending=null;}
function syncMapScope(){
 const scope=mapScope();if(mapEditScope===scope)return scope;
 const previous=mapEditScope;mapEditScope=scope;mapEditMode=false;mapSearch='';mapEditBusy=false;mapOperation=null;
 if(previous!==null){mapTrashScope=null;mapTrashRows=[];}mapTrashRequest++;invalidateMapList();mapListError='';
 if(previous!==null){if(mapOwnedDialog===mapDialogGeneration)$('#modal')?.close();mapDialogGeneration++;mapOwnedDialog=null;mapLibrary={maps:[]};selectedRemote=null;}
 return scope;
}
function requireMapMutation(scope,dialog=null,action=null){
 syncMapScope();if(scope!==mapScope())throw Error('连接已变化，请重新选择地图');
 if(dialog!==null&&dialog!==mapDialogGeneration)throw Error('确认窗口已变化，请重新选择地图');
 const reason=mapMutationReason(action);if(reason)throw Error(reason);
}
function mapOperationCurrent(token){return mapOperation===token&&token.scope===mapScope();}
function reconcileMapSelection(deletedName=null){
 if(!S?.maps)return;
 if(deletedName)S.maps=S.maps.filter(m=>!(m.robot_map_name===deletedName&&(S.mode==='demo'?m.demo:m.robot_ip===S.ip)));
 if(typeof mapId==='undefined'||!mapId||S.maps.some(m=>m.id===mapId))return;
 const active=S.maps.find(m=>m.robot_map_name===mapLibrary.active&&m.verified_for_robot&&(S.mode==='demo'||m.robot_ip===S.ip));
 mapId=active?.id||null;if(typeof points!=='undefined')points=[];if(typeof routeId!=='undefined')routeId=null;
 if(typeof invalidateGoalPlan==='function')invalidateGoalPlan();
 if(typeof render==='function'&&['navigation','maps'].includes(page))render();
}
const trashRestorable=m=>m.recoverable!==false&&!['purging','purge_failed','purged'].includes(m.state);
const trashPurgeable=m=>['purging','purge_failed'].includes(m.state)?m.purge_retryable===true:m.state!=='purged';
async function loadMapLibrary(){
 const scope=syncMapScope();if(mapListPending?.scope===scope)return mapListPending.promise;
 const token={scope,generation:++mapListGeneration};mapListPending=token;
 token.promise=(async()=>{
  try{
   const result=await api('/api/maps/library',undefined,true);
   if(mapListPending!==token||scope!==mapScope()||token.generation!==mapListGeneration)return null;
   mapLibrary=result;mapListError='';if(!selectedRemote||!result.maps.some(m=>m.name===selectedRemote))selectedRemote=result.active||result.maps[0]?.name||null;
   paintMapLibrary();return result;
  }catch(error){
   if(mapListPending!==token||scope!==mapScope())return null;
   mapListError=error.message;mapLibrary={maps:[]};selectedRemote=null;paintMapLibrary();return null;
  }finally{if(mapListPending===token)mapListPending=null;}
 })();return token.promise;
}
ui['map-library-refresh']=loadMapLibrary;
function visibleLibraryMaps(){
 const query=mapSearch.trim().toLocaleLowerCase();
 return mapLibrary.maps.filter(m=>(mapLabel(m)+' '+m.name).toLocaleLowerCase().includes(query))
  .slice().sort((a,b)=>Number(!!b.active)-Number(!!a.active)||(b.modified||0)-(a.modified||0)||a.name.localeCompare(b.name));
}
paintMapLibrary=function(){
 syncMapScope();
 const search=$('#map-library-search');if(search&&search.value!==mapSearch)search.value=mapSearch;
 const enabled=mapLibrary.management_version===1&&mapLibrary.desktop_management_version===1,editing=mapEditMode&&enabled,reason=mapMutationReason(),blocked=mapEditBusy||!!reason;
 const button=$('[data-ui="map-library-edit"]');
 if(button){button.textContent=editing?'完成编辑':'编辑地图';button.disabled=!enabled||mapEditBusy||!!mapMutationReason('rename');button.title=enabled?'重命名或移入回收站':'连接地图服务后可编辑';}
 for(const id of ['map-edit-confirm','map-purge-confirm','hub-confirm-apply']){const confirm=$(`[data-ui="${id}"]`);if(confirm&&mapOwnedDialog===mapDialogGeneration){const dialogReason=mapMutationReason(mapOwnedAction);confirm.disabled=mapEditBusy||!!dialogReason;confirm.title=dialogReason;}}
 const activate=$('[data-ui="apply-selected-map"]');if(activate){activate.disabled=blocked||!selectedRemote;activate.title=reason;}
 for(const entry of document.querySelectorAll?.('[data-map-restore],[data-map-purge]')||[]){const row=mapTrashRows.find(m=>m.id===(entry.dataset.mapRestore||entry.dataset.mapPurge));if(row){entry.disabled=blocked||!(entry.dataset.mapRestore?trashRestorable(row):trashPurgeable(row));entry.title=reason;}}
 const list=$('#remote-map-list');if(!list)return;
 const rows=visibleLibraryMaps();
 const markup=rows.map(m=>`<div class="map-library-entry ${m.name===selectedRemote?'selected':''}">
  <button class="map-library-item" data-remote-map="${esc(m.name)}" title="${esc(m.name)}"><span class="map-thumb">${icon('maps')}</span><span class="map-entry-label"><strong>${esc(mapLabel(m))}</strong><small>${m.grids.length} 份栅格 · ${sizeLabel(m.cloud_bytes)}</small>${m.display_name&&m.display_name!==m.name?`<small class="map-identity">${esc(m.name)}</small>`:''}</span><span class="map-active-label">${m.active?'使用中':''}</span></button>
  ${editing?`<div class="map-row-edit"><button class="btn small" data-map-rename="${esc(m.name)}" ${mapEditBusy?'disabled':''}>重命名</button><button class="btn small danger" data-map-delete="${esc(m.name)}" ${m.active||m.name===mapLibrary.active||blocked?'disabled':''} title="${esc(reason||(m.active||m.name===mapLibrary.active?'当前活动地图不可删除':'移入回收站，可恢复'))}">删除</button></div>`:''}</div>`).join('')||empty(mapListError?'地图服务未连接':mapSearch?'没有匹配的地图':'还没有地图',mapListError?esc(mapListError):mapSearch?'换个名称或编号搜索。':'可以开始建图或导入地图包。','maps');
 if(list._editorMarkup!==markup){list.innerHTML=markup;list._editorMarkup=markup;}
 text('map-disk',mapLibrary.disk?'存储可用 '+sizeLabel(mapLibrary.disk.free):'—');
 text('map-library-count',`${rows.length} / ${mapLibrary.maps.length} 张地图`);
 const trash=$('[data-ui="map-library-trash"]');if(trash){trash.disabled=!enabled||mapEditBusy;trash.textContent='回收站'+(mapLibrary.trash_count?' ('+mapLibrary.trash_count+')':'');}
 const selected=mapLibrary.maps.find(m=>m.name===selectedRemote);
 text('selected-map-info',selected?mapLabel(selected)+(selected.active?' · 当前活动地图':' · 未激活'):'请选择一张地图');
 const grid=$('#remote-grid-id'),options=selected?.grids.map(i=>`<option value="${i}">栅格 ${i}</option>`).join('')||'<option value="0">栅格 0</option>';
 if(grid&&(grid._gridMarkup!==options||grid._mapName!==selectedRemote)){
  const previous=grid._mapName===selectedRemote?grid.value:'0';grid.innerHTML=options;grid._gridMarkup=options;grid._mapName=selectedRemote;
  if([...grid.options].some(o=>o.value===previous))grid.value=previous;
 }
};

ui['map-library-edit']=()=>{if(mapEditBusy||mapMutationReason('rename'))return;mapEditMode=!mapEditMode;paintMapLibrary();};
document.addEventListener('input',e=>{
 if(e.target.id==='map-library-search'){mapSearch=e.target.value;paintMapLibrary();}
});

async function editLibrary(action,payload,scope,dialog=mapDialogGeneration){
 requireMapMutation(scope,dialog,action);
 if(mapEditBusy)return;
 const token={scope,dialog};mapOperation=token;mapEditBusy=true;invalidateMapList();paintMapLibrary();
 try{
  const result=await job('/api/maps/library/'+action,payload,action==='rename'?'重命名地图':action==='restore'?'恢复地图':action==='purge'?'彻底删除地图文件':'删除地图',()=>mapOperationCurrent(token));
  if(!mapOperationCurrent(token)||!result)return;
  const changedPreview=action==='delete'&&(selectedRemote===payload.name||typeof cloudSource!=='undefined'&&cloudSource?.kind==='map'&&cloudSource.key===payload.name);
  if(action==='delete')mapLibrary.maps=mapLibrary.maps.filter(m=>m.name!==payload.name);
  if(changedPreview){if(selectedRemote===payload.name)selectedRemote=null;if(typeof cloudViewGeneration!=='undefined')cloudViewGeneration++;if(typeof cloudData!=='undefined')cloudData=null;if(typeof cloudSource!=='undefined')cloudSource=null;}
  if(dialog===mapDialogGeneration)$('#modal')?.close();invalidateMapList();paintMapLibrary();await loadMapLibrary();
  if(!mapOperationCurrent(token))return;await fetchState();if(!mapOperationCurrent(token))return;
  reconcileMapSelection(action==='delete'?payload.name:null);
  if(changedPreview&&selectedRemote&&page==='maps')await previewCloud('map',selectedRemote);
  if(!mapOperationCurrent(token))return;
  toast(action==='rename'?'名称已保存':action==='restore'?'地图已恢复，未自动启用':action==='purge'?'地图文件已彻底删除':'已移入回收站，可随时恢复');
  return result;
 }catch(error){
  if(mapOperationCurrent(token)){
   if(action==='purge'&&dialog===mapDialogGeneration){try{await ui['map-library-trash']();}catch(refreshError){/* Keep the original deletion error. */}}
   throw error;
  }
 }
 finally{if(mapOperation===token){mapOperation=null;mapEditBusy=false;paintMapLibrary();}}
}

function editMapDialog(name,action){
 const scope=syncMapScope();requireMapMutation(scope,null,action);
 const map=mapLibrary.maps.find(m=>m.name===name);if(!map)throw Error('地图列表已变化，请重新选择');
 if(action==='delete'&&(map.active||name===mapLibrary.active))throw Error('当前活动地图不能删除');
 if(action==='rename'){
  modal('重命名地图',`<p>${esc(map.name)}</p><label class="field">地图名称<input id="map-display-name" maxlength="80" value="${esc(mapLabel(map))}"></label><p class="help">支持中文。内部地图编号与已有路线保持不变。</p>`,[{label:'保存名称',ui:'map-edit-confirm'}]);
  const dialog=mapDialogGeneration;mapOwnedDialog=dialog;mapOwnedAction='rename';
  ui['map-edit-confirm']=()=>editLibrary('rename',{name,display_name:$('#map-display-name').value.trim()},scope,dialog);
 }else{
  modal('删除地图',`<p>将 <strong>${esc(mapLabel(map))}</strong> 移入回收站？</p><p class="help">需要时可从回收站恢复。</p>`,[{label:'移入回收站',ui:'map-edit-confirm',style:'danger'}]);
  const dialog=mapDialogGeneration;mapOwnedDialog=dialog;
  ui['map-edit-confirm']=()=>editLibrary('delete',{name},scope,dialog);
 }
}

ui['map-library-trash']=async()=>{
 const scope=syncMapScope(),request=++mapTrashRequest,dialogBefore=mapDialogGeneration,sourcePage=page;
 const result=await api('/api/maps/library/trash',undefined,true);if(scope!==mapScope()||request!==mapTrashRequest||dialogBefore!==mapDialogGeneration||sourcePage!==page)return;
 mapTrashScope=scope;mapTrashRows=result.maps;
 const purge=mapLibrary.permanent_delete_supported&&mapLibrary.desktop_permanent_delete_supported&&mapLibrary.catalog_integrity_protocol===1&&mapLibrary.desktop_catalog_integrity_protocol===1,blocked=mapEditBusy||!!mapMutationReason();
 const state=m=>m.state==='purge_failed'?'删除未完成，原地图已不能完整恢复':m.state==='purging'?(m.purge_retryable?'上次删除结果尚未确认，可检查并重试；不能恢复':'正在删除，暂不可恢复'):m.state==='purged'?'文件已彻底删除':'';
 modal('地图回收站',`<p class="help">恢复后不会自动启用。回收站仍占用存储空间；彻底删除文件后不可恢复。</p><div class="map-trash-list">${result.maps.map(m=>`<div class="map-trash-row"><span><strong>${esc(mapLabel(m))}</strong><small>${esc(m.name)}</small>${state(m)?`<small>${esc(state(m))}</small>`:''}</span><div class="button-row"><button class="btn small" data-map-restore="${esc(m.id)}" ${blocked||!trashRestorable(m)?'disabled':''}>恢复</button>${purge?`<button class="btn small danger" data-map-purge="${esc(m.id)}" ${blocked||!trashPurgeable(m)?'disabled':''}>${m.state==='purging'?'检查并重试删除':m.state==='purge_failed'?'重试彻底删除':'彻底删除'}</button>`:''}</div></div>`).join('')||'<p>回收站为空</p>'}</div>`);
 mapOwnedDialog=mapDialogGeneration;
};
function purgeMapDialog(id){
 const map=mapTrashRows.find(m=>m.id===id),scope=mapTrashScope;
 if(!map||scope!==mapScope())throw Error('回收站状态已变化，请重新打开');
 requireMapMutation(scope);if(!trashPurgeable(map))throw Error('本条地图正在删除或已经删除，请刷新回收站');
 modal('彻底删除地图文件',`<p>永久删除 <strong>${esc(mapLabel(map))}</strong>？</p><p class="help">编号：${esc(map.name)}</p><p>点云、栅格及地图记录将被移除，无法从回收站恢复。已有路线保留，重新使用前需关联可用地图。</p>`,[{label:'永久删除文件',ui:'map-purge-confirm',style:'danger'}]);
 $('[data-ui="map-purge-confirm"]').disabled=false;
 const dialog=mapDialogGeneration;mapOwnedDialog=dialog;
 ui['map-purge-confirm']=()=>{
  return editLibrary('purge',{id,confirmed_name:map.name},scope,dialog);
 };
}
document.addEventListener('click',async e=>{
 const b=e.target.closest('button');if(!b||b.disabled)return;
 try{
  if(b.dataset.mapRename)editMapDialog(b.dataset.mapRename,'rename');
  else if(b.dataset.mapDelete)editMapDialog(b.dataset.mapDelete,'delete');
  else if(b.dataset.mapRestore){const row=mapTrashRows.find(m=>m.id===b.dataset.mapRestore);if(!row||!trashRestorable(row))throw Error('这份地图已开始永久删除，不能恢复');await editLibrary('restore',{id:b.dataset.mapRestore},mapTrashScope);}
  else if(b.dataset.mapPurge)purgeMapDialog(b.dataset.mapPurge);
 }catch(error){toast(error.message,true);}
});

function openMapActivation(name,grid=0){
 const scope=syncMapScope();requireMapMutation(scope);
 const map=mapLibrary.maps.find(m=>m.name===name);
 if(!map)throw Error('地图列表已变化，请刷新后再选择');
 const cleanup=mapLibrary.management_version===1&&mapLibrary.desktop_management_version===1&&mapLibrary.catalog_integrity_protocol===1&&mapLibrary.desktop_catalog_integrity_protocol===1&&map.cleanup_base&&map.cleanup_base===mapLibrary.active&&map.name!==mapLibrary.active;
 const cleanupBase=cleanup?map.cleanup_base:null;
 modal('启用地图',`<p>使用 <strong>${esc(mapLabel(map))}</strong> 进行定位和导航。</p>${cleanup?`<label class="map-cleanup-choice"><input id="map-cleanup-previous" type="checkbox" checked> 将已合并的旧版本移入回收站</label><p class="help">${esc(map.cleanup_base)} 可从回收站恢复；仍有任务或路线使用时会自动保留。</p>`:''}`,[{label:'启用地图',ui:'hub-confirm-apply'}]);
 const dialog=mapDialogGeneration;mapOwnedDialog=dialog;
 ui['hub-confirm-apply']=async()=>{
  requireMapMutation(scope,dialog);if(mapEditBusy)return;
  if(!mapLibrary.maps.some(m=>m.name===name))throw Error('地图列表已变化，请重新选择');
  const cleanupPrevious=!!cleanupBase&&!!$('#map-cleanup-previous')?.checked;
  const token={scope,dialog};mapOperation=token;mapEditBusy=true;invalidateMapList();$('#modal').close();paintMapLibrary();
  try{
   const result=await job('/api/maps/library/apply',{name,grid_id:grid,cleanup_previous:cleanupPrevious,...(cleanupPrevious?{cleanup_base:cleanupBase}:{})},'启用地图并同步导航',()=>mapOperationCurrent(token));
   if(!mapOperationCurrent(token)||!result)return;
   if(mapId!==result.map.id){points=[];routeId=null;if(typeof invalidateGoalPlan==='function')invalidateGoalPlan();}mapId=result.map.id;
   await fetchState();if(!mapOperationCurrent(token))return;invalidateMapList();await loadMapLibrary();if(!mapOperationCurrent(token))return;
   reconcileMapSelection(result.cleanup?.ok?cleanupBase:null);render();
   toast(result.message||(result.cleanup?.ok?'新图已启用，旧图已移入回收站':result.cleanup?.retained?'新图已启用，旧版本保留':'地图已启用'));
   return result;
  }catch(error){if(mapOperationCurrent(token))throw error;}
  finally{if(mapOperation===token){mapOperation=null;mapEditBusy=false;paintMapLibrary();}}
 };
}
ui['apply-selected-map']=()=>{if(!selectedRemote)throw Error('请先选择地图');openMapActivation(selectedRemote,+($('#remote-grid-id')?.value||0));};
if(typeof update==='function'){const previousMapUpdate=update;update=function(...args){previousMapUpdate.apply(this,args);syncMapScope();if(page==='maps'||mapOwnedDialog===mapDialogGeneration)paintMapLibrary();};}
document.addEventListener('click',e=>{if(e.target.closest('[data-ui="hub-connect"],[data-ui="hub-demo"],[data-connection-role="nos"],[data-service-role="nos"]')){mapScopeGeneration++;syncMapScope();paintMapLibrary();}},true);
