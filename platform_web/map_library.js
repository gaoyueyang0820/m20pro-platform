'use strict';

let mapEditMode=false,mapEditBusy=false,mapSearch='',mapEditScope=null,mapTrashScope=null,mapTrashRows=[];
const mapScope=()=>S.mode+'|'+(S.ip||'');
const mapLabel=m=>m.display_name||m.name;
function visibleLibraryMaps(){
 const query=mapSearch.trim().toLocaleLowerCase();
 return mapLibrary.maps.filter(m=>(mapLabel(m)+' '+m.name).toLocaleLowerCase().includes(query))
  .slice().sort((a,b)=>Number(!!b.active)-Number(!!a.active)||(b.modified||0)-(a.modified||0)||a.name.localeCompare(b.name));
}
paintMapLibrary=function(){
 if(mapEditScope!==mapScope()){mapEditScope=mapScope();mapEditMode=false;mapSearch='';}
 const search=$('#map-library-search');if(search&&search.value!==mapSearch)search.value=mapSearch;
 const enabled=mapLibrary.management_version===1&&mapLibrary.desktop_management_version===1,editing=mapEditMode&&enabled;
 const button=$('[data-ui="map-library-edit"]');
 if(button){button.textContent=editing?'完成编辑':'编辑地图';button.disabled=!enabled||mapEditBusy;button.title=enabled?'重命名或移入回收站':'需加载新版电脑端与 NOS 地图服务';}
 const list=$('#remote-map-list');if(!list)return;
 const rows=visibleLibraryMaps();
 const markup=rows.map(m=>`<div class="map-library-entry ${m.name===selectedRemote?'selected':''}">
  <button class="map-library-item" data-remote-map="${esc(m.name)}" title="${esc(m.name)}"><span class="map-thumb">${icon('maps')}</span><span class="map-entry-label"><strong>${esc(mapLabel(m))}</strong><small>${m.grids.length} 份栅格 · ${sizeLabel(m.cloud_bytes)}</small>${m.display_name&&m.display_name!==m.name?`<small class="map-identity">${esc(m.name)}</small>`:''}</span><span class="map-active-label">${m.active?'使用中':''}</span></button>
  ${editing?`<div class="map-row-edit"><button class="btn small" data-map-rename="${esc(m.name)}" ${mapEditBusy?'disabled':''}>重命名</button><button class="btn small danger" data-map-delete="${esc(m.name)}" ${m.active||mapEditBusy?'disabled':''} title="${m.active?'当前活动地图不可删除':'移入回收站，可恢复'}">删除</button></div>`:''}</div>`).join('')||empty(mapSearch?'没有匹配的地图':'还没有地图',mapSearch?'换个名称或编号搜索。':'可以开始建图或导入地图包。','maps');
 if(list._editorMarkup!==markup){list.innerHTML=markup;list._editorMarkup=markup;}
 text('map-disk',mapLibrary.disk?'存储可用 '+sizeLabel(mapLibrary.disk.free):'—');
 text('map-library-count',`${rows.length} / ${mapLibrary.maps.length} 张地图`);
 const trash=$('[data-ui="map-library-trash"]');if(trash){trash.disabled=!enabled||mapEditBusy;trash.textContent='回收站'+(mapLibrary.trash_count?' ('+mapLibrary.trash_count+')':'');}
 const selected=mapLibrary.maps.find(m=>m.name===selectedRemote);
 if(selected)text('selected-map-info',mapLabel(selected)+(selected.active?' · 当前活动地图':' · 未激活'));
 const grid=$('#remote-grid-id'),options=selected?.grids.map(i=>`<option value="${i}">栅格 ${i}</option>`).join('')||'<option value="0">栅格 0</option>';
 if(grid&&(grid._gridMarkup!==options||grid._mapName!==selectedRemote)){
  const previous=grid._mapName===selectedRemote?grid.value:'0';grid.innerHTML=options;grid._gridMarkup=options;grid._mapName=selectedRemote;
  if([...grid.options].some(o=>o.value===previous))grid.value=previous;
 }
};

ui['map-library-edit']=()=>{mapEditMode=!mapEditMode;paintMapLibrary();};
document.addEventListener('input',e=>{
 if(e.target.id==='map-library-search'){mapSearch=e.target.value;paintMapLibrary();}
 if(e.target.id==='map-purge-confirm-text'){const b=$('[data-ui="map-purge-confirm"]');if(b)b.disabled=e.target.value.trim()!=='删除';}
});

async function editLibrary(action,payload,scope){
 if(scope!==mapScope())throw Error('连接已变化，请重新选择地图');
 if(mapEditBusy)return;
 mapEditBusy=true;paintMapLibrary();
 try{
  const result=await job('/api/maps/library/'+action,payload,action==='rename'?'重命名地图':action==='restore'?'恢复地图':action==='purge'?'彻底删除地图文件':'删除地图');
  if(scope!==mapScope())return;
  const changedPreview=action==='delete'&&selectedRemote===payload.name;
  if(changedPreview){selectedRemote=null;cloudViewGeneration++;cloudData=null;cloudSource=null;}
  $('#modal').close();await loadMapLibrary();await fetchState();
  if(changedPreview&&selectedRemote&&page==='maps')await previewCloud('map',selectedRemote);
  toast(action==='rename'?'名称已保存':action==='restore'?'地图已恢复，未自动启用':action==='purge'?'地图文件已彻底删除':'已移入回收站，可随时恢复');
  return result;
 }finally{mapEditBusy=false;paintMapLibrary();}
}

function editMapDialog(name,action){
 const map=mapLibrary.maps.find(m=>m.name===name);if(!map)throw Error('地图列表已变化，请重新选择');
 if(action==='delete'&&map.active)throw Error('当前活动地图不能删除');
 const scope=mapScope();
 if(action==='rename'){
  modal('重命名地图',`<p>${esc(map.name)}</p><label class="field">地图名称<input id="map-display-name" maxlength="80" value="${esc(mapLabel(map))}"></label><p class="help">支持中文。内部地图编号与已有路线保持不变。</p>`,[{label:'保存名称',ui:'map-edit-confirm'}]);
  ui['map-edit-confirm']=()=>editLibrary('rename',{name,display_name:$('#map-display-name').value.trim()},scope);
 }else{
  modal('删除地图',`<p>将 <strong>${esc(mapLabel(map))}</strong> 移入回收站？</p><p class="help">可恢复，不会永久清除地图文件。当前活动地图和运行中的建图数据受保护。</p>`,[{label:'移入回收站',ui:'map-edit-confirm',style:'danger'}]);
  ui['map-edit-confirm']=()=>editLibrary('delete',{name},scope);
 }
}

ui['map-library-trash']=async()=>{
 const scope=mapScope(),result=await api('/api/maps/library/trash');if(scope!==mapScope())return;
 mapTrashScope=scope;mapTrashRows=result.maps;
 const purge=mapLibrary.permanent_delete_supported&&mapLibrary.desktop_permanent_delete_supported;
 modal('地图回收站',`<p class="help">恢复后不会自动启用。回收站仍占用存储空间；彻底删除文件后不可恢复。</p><div class="map-trash-list">${result.maps.map(m=>`<div class="map-trash-row"><span><strong>${esc(mapLabel(m))}</strong><small>${esc(m.name)}</small></span><div class="button-row"><button class="btn small" data-map-restore="${esc(m.id)}">恢复</button>${purge?`<button class="btn small danger" data-map-purge="${esc(m.id)}">彻底删除</button>`:''}</div></div>`).join('')||'<p>回收站为空</p>'}</div>`);
};
function purgeMapDialog(id){
 const map=mapTrashRows.find(m=>m.id===id),scope=mapTrashScope;
 if(!map||scope!==mapScope())throw Error('回收站状态已变化，请重新打开');
 modal('彻底删除地图文件',`<p>永久删除 <strong>${esc(mapLabel(map))}</strong>？</p><p class="help">编号：${esc(map.name)}</p><p>这份地图的点云、栅格和所包含的记录将被移除，无法从回收站恢复。</p><label class="field">请输入“删除”确认<input id="map-purge-confirm-text" autocomplete="off" placeholder="删除"></label>`,[{label:'永久删除文件',ui:'map-purge-confirm',style:'danger'}]);
 $('[data-ui="map-purge-confirm"]').disabled=true;
 ui['map-purge-confirm']=()=>{
  if($('#map-purge-confirm-text').value.trim()!=='删除')throw Error('请输入“删除”确认');
  return editLibrary('purge',{id,confirmed_name:map.name},scope);
 };
}
document.addEventListener('click',async e=>{
 const b=e.target.closest('button');if(!b||b.disabled)return;
 try{
  if(b.dataset.mapRename)editMapDialog(b.dataset.mapRename,'rename');
  else if(b.dataset.mapDelete)editMapDialog(b.dataset.mapDelete,'delete');
  else if(b.dataset.mapRestore)await editLibrary('restore',{id:b.dataset.mapRestore},mapTrashScope);
  else if(b.dataset.mapPurge)purgeMapDialog(b.dataset.mapPurge);
 }catch(error){toast(error.message,true);}
});

function openMapActivation(name,grid=0){
 const map=mapLibrary.maps.find(m=>m.name===name),scope=mapScope();
 if(!map)throw Error('地图列表已变化，请刷新后再选择');
 const cleanup=mapLibrary.management_version===1&&mapLibrary.desktop_management_version===1&&map.cleanup_base&&map.cleanup_base===mapLibrary.active&&map.name!==mapLibrary.active;
 modal('确认并启用地图',`<p>请先核对合并后的墙线、通道和未重扫区域。</p><p>启用 <strong>${esc(mapLabel(map))}</strong>，重启定位并同步导航栅格 ${grid}。</p>${cleanup?`<label class="map-cleanup-choice"><input id="map-cleanup-previous" type="checkbox" checked> 确认完整合并，启用成功后自动清理旧图</label><p class="help">${esc(map.cleanup_base)} 将移入回收站，可恢复。启用或同步失败时保留旧图。</p>`:''}`,[{label:'确认并启用',ui:'hub-confirm-apply'}]);
 ui['hub-confirm-apply']=async()=>{
  if(scope!==mapScope())throw Error('连接已变化，请重新选择地图');
  const cleanupPrevious=!!$('#map-cleanup-previous')?.checked;
  $('#modal').close();const result=await job('/api/maps/library/apply',{name,grid_id:grid,cleanup_previous:cleanupPrevious},'启用地图并同步导航');
  if(scope!==mapScope())return;
  if(mapId!==result.map.id){points=[];routeId=null;}mapId=result.map.id;
  await fetchState();await loadMapLibrary();render();
  toast(result.cleanup?.ok?'新图已启用，旧图已移入回收站':result.cleanup?.message||'地图已启用，请核对定位位置与朝向');
 };
}
ui['apply-selected-map']=()=>{if(!selectedRemote)throw Error('请先选择地图');openMapActivation(selectedRemote,+($('#remote-grid-id')?.value||0));};
