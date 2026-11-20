'use strict';

let navSyncBusy=false,navSyncOwner=null;
let driveView='front';
const originalMotionPage=motion;
motion=function(){
 const layout=document.createElement('div');layout.innerHTML=originalMotionPage();
 const cards=[...layout.querySelectorAll('.card')];
 const take=title=>cards.find(c=>c.querySelector('h2')?.textContent.includes(title))?.outerHTML||'';
 const control=cards.find(c=>c.querySelector('h2')?.textContent.includes('手动遥控'));
 const posture=cards.find(c=>c.querySelector('h2')?.textContent.includes('姿态与步态'));
 if(posture){
  const quick=posture.querySelector('.card-pad>.button-row');
  if(quick&&control){quick.classList.add('drive-quick-actions');quick.remove();control.querySelector('.joystick-wrap')?.before(quick);}
 }
 const feeds=['front','rear'].map((cam,i)=>`<div class="drive-feed drive-feed-${cam}"><div class="camera-feed"><img id="camera-${cam}" alt="${i?'后置':'前置'}相机" hidden><div class="camera-placeholder" id="placeholder-${cam}">${icon('vision')}<strong>${S.mode==='demo'?'演示模式没有实机画面':'正在连接'+(i?'后置':'前置')+'相机…'}</strong><small>${i?'后视':'前视'} · 相机预览</small></div><div class="camera-hud"><span>${i?'后视':'前视'}</span><span id="cam-${cam}-tag">等待画面</span></div></div><span class="drive-camera-note" id="cam-${cam}-note"></span></div>`).join('');
 const video=card('驾驶视角',`<div class="drive-camera-toolbar"><div class="segmented">${[['front','前视'],['rear','后视'],['both','双视角']].map(([v,label])=>`<button data-drive-view="${v}" class="${driveView===v?'active':''}">${label}</button>`).join('')}</div><div class="button-row">${btn('重连画面','drive-camera-retry','small')}${btn('YOLO / 视觉识别','drive-vision','small')}</div></div><div id="drive-feeds" class="drive-feeds view-${driveView}">${feeds}</div>${cameraProfileControls()}<div class="drive-camera-footer"><span id="drive-video-state">等待相机反馈</span><span>旧画面会标记暂停，不作为实时反馈</span></div>`,'vision');
 return heading(btn('停稳并建图','start-map-here','','maps')+cmd('停止运动','stop','danger'))+
 `<div class="drive-layout">
   <div class="drive-camera-panel">${video}</div>
   <div class="drive-control-panel">${control?control.outerHTML:''}${take('姿态与步态')}</div>
   <div class="drive-settings drive-settings-secondary">${take('照明与方向')}${take('电源与充电')}</div>
  </div>
  <div id="drive-advanced"></div>`;
};

ui['drive-vision']=()=>navigate('vision');

function cachedMapFor(name,gridId=0){return S?.maps.find(m=>m.robot_map_name===name&&(m.grid_id||0)===gridId&&m.robot_ip===S.ip);}
function mapFlowStatus(){
 const map=page==='navigation'?selectedMap():cachedMapFor(selectedRemote,+($('#remote-grid-id')?.value||0));
 const name=page==='navigation'?map?.robot_map_name:selectedRemote;
 const demo=S.mode==='demo';const active=demo?(!!map?.demo||!!name):mapLibrary.active===name;
 const synced=!!map?.verified_for_robot&&map.robot_ip===S.ip&&active;
 const localized=demo||active&&S.online&&S.pose.Location===0;
 const step=(n,title,detail,ok)=>`<div class="map-flow-step ${ok?'complete':''}"><span>${ok?'✓':n}</span><div><strong>${title}</strong><small>${esc(detail)}</small></div></div>`;
 return `<div class="map-flow">${step(1,demo?'演示地图':'机器人活动地图',demo?name||map?.name||mapLibrary.active:mapLibrary.active||'正在读取地图',active)}${step(2,'导航地图',demo?'演示地图':synced?'已同步':active?'开始时自动同步':'请显示机器人当前地图',demo||synced)}${step(3,'定位状态',demo?'演示位置':localized?'定位已就绪':active?'开始时自动恢复定位':'等待当前地图',localized)}<div class="map-flow-action">${demo||synced||page==='navigation'&&active?'':btn(page==='navigation'?'显示当前地图':'用于导航',page==='navigation'?'goal-sync-active-map':'map-use-navigation','primary small')}</div></div>`;
}
function navigationMapReady(){
 const map=selectedMap();return S.mode==='demo'||!!(map?.verified_for_robot&&map.robot_ip===S.ip&&map.robot_map_name===mapLibrary.active);
}
function navigationMapCanPrepare(map=selectedMap()){
 return S.mode==='demo'||!!(map?.robot_map_name&&!map.demo&&!map.archived&&
  (mapLibrary.active?map.robot_map_name===mapLibrary.active:map.robot_ip===S.ip));
}
function paintMapFlow(){
 if(!['maps','navigation'].includes(page)||!$('#main .page-heading'))return;
 let el=$('#map-navigation-flow');if(!el){el=document.createElement('div');el.id='map-navigation-flow';$('#main .page-heading')?.after(el);}
 const markup=mapFlowStatus();if(el._flowMarkup!==markup){el.innerHTML=markup;el._flowMarkup=markup;}
 if(page==='navigation'){
  const ready=navigationMapCanPrepare(),start=$('[data-ui="start-nav"]');if(start){start.disabled=!ready||S.mapping||S.autonomy?.active||['running','cancelling'].includes(S.nav?.status);start.title=ready?'自动准备并执行巡检路线':'请显示机器人当前地图后设置路线';}
  // Single-goal readiness also depends on the plan and task state. One owner
  // must apply all conditions, including on asynchronous map-library refreshes.
  if(typeof paintGoalPlan==='function')paintGoalPlan();
 }
}
const previousPaintMapLibrary=paintMapLibrary;
paintMapLibrary=function(){
 previousPaintMapLibrary();paintMapFlow();
 for(const button of $$('.map-library-item')){
  const map=mapLibrary.maps.find(m=>m.name===button.dataset.remoteMap);if(!map)continue;
  let badge=button.querySelector('.map-state-label');if(!badge){badge=document.createElement('b');badge.className='map-state-label';button.append(badge);}
  const synced=cachedMapFor(map.name);
  badge.textContent=map.active?(synced?.verified_for_robot?'活动 · 已同步':'活动 · 待同步'):map.requires_alignment_review?'融合候选 · 待核对':'未启用';
 }
};
function navigationMapScope(){return typeof mapScope==='function'?mapScope():JSON.stringify([S?.mode,S?.ip,S?.server_instance,S?.connection_generation]);}
let navigationMapSyncGeneration=0;
async function syncNavigationMap(name,gridId=0){
 const scope=navigationMapScope(),generation=++navigationMapSyncGeneration,previousId=mapId;
 const current=()=>scope===navigationMapScope()&&generation===navigationMapSyncGeneration&&(mapId===previousId||mapId===null);
 const result=await api('/api/maps/library/sync',{name,grid_id:gridId},true);if(!current())return null;
 await fetchState();if(!current())return null;await loadMapLibrary();if(!current())return null;
 mapId=result.map.id;return result;
}
ui['map-use-navigation']=async()=>{
 const name=page==='navigation'?selectedMap()?.robot_map_name:selectedRemote;
 if(!name)throw Error('请先选择机器人地图');
 if(mapLibrary.active!==name){selectedRemote=name;return ui['apply-selected-map']();}
 const previousId=mapId,scope=navigationMapScope();const synced=await syncNavigationMap(name,page==='maps'?+$('#remote-grid-id').value:selectedMap()?.grid_id||0);if(!synced||scope!==navigationMapScope())return;
 if(previousId!==mapId){points=[];routeId=null;}navigate('navigation');toast('活动地图已同步，请核对机器人位置后设置航点');
};
ui['apply-selected-map']=()=>{
 if(!selectedRemote)throw Error('请先选择地图');
 const grid=+($('#remote-grid-id')?.value||0),name=selectedRemote;
 const review=mapLibrary.maps.find(m=>m.name===name)?.requires_alignment_review;
 modal('启用地图并同步导航',`${review?'<p>启用前请检查墙线与通道是否对齐。</p>':''}<p>将机器人活动地图切换为 <strong>${esc(name)}</strong>，重启定位并自动同步栅格 ${grid}。完成后请核对机器人在地图中的位置与朝向。</p>`,[{label:'切换并同步',ui:'hub-confirm-apply'}]);
 ui['hub-confirm-apply']=async()=>{
  $('#modal').close();const result=await job('/api/maps/library/apply',{name,grid_id:grid},'切换地图并同步导航');
  if(mapId!==result.map.id){points=[];routeId=null;}mapId=result.map.id;await fetchState();await loadMapLibrary();render();toast('活动地图和导航栅格已同步，可进入导航设置路线');
 };
};
const workspaceRender=render,workspaceUpdate=update;
render=function(){
 const previewWasOpen=page==='navigation'&&$('#nav-camera-preview')?.hidden===false;
 for(const cam of ['front','rear'])cameraClose(cam);
 workspaceRender();
 mountCameraPreviewControls();
 if(page==='motion'&&S.mode==='live')for(const cam of activeDriveCameras())cameraOpen(cam);
 paintCameraStatus();
 if(previewWasOpen)ui['nav-camera-toggle']();
 if(page==='navigation'){
  const scope=navigationMapScope(),selectedBefore=mapId;
  loadMapLibrary().then(async()=>{
  if(page!=='navigation'||scope!==navigationMapScope()||selectedBefore!==mapId)return;
  const current=selectedMap();
  if(!mapLibrary.maps.some(m=>m.name===current?.robot_map_name)){
   const active=cachedMapFor(mapLibrary.active);if(active&&active.id!==mapId){points=[];routeId=null;if(typeof invalidateGoalPlan==='function')invalidateGoalPlan();mapId=active.id;render();return;}
  }
  if(navSyncOwner?.scope!==scope){navSyncBusy=false;navSyncOwner=null;}
  if(window.M20NavigationPreparation?.state){paintMapFlow();return;}
  if(page==='navigation'&&!navSyncBusy&&current?.robot_map_name===mapLibrary.active&&current?.robot_ip===S.ip&&!current?.verified_for_robot&&S.mode==='live'){
   const token={scope};navSyncOwner=token;navSyncBusy=true;paintMapFlow();
   try{const previous=mapId;await syncNavigationMap(mapLibrary.active,current.grid_id||0);if(scope===navigationMapScope()&&page==='navigation'&&previous!==mapId)render();}
   catch(e){if(scope===navigationMapScope())toast('地图暂未同步，开始时将自动重试',true);}
   finally{if(navSyncOwner===token){navSyncBusy=false;navSyncOwner=null;}}
  }paintMapFlow();
 }).catch(()=>{});
 }
 paintMapFlow();
};
update=function(){workspaceUpdate();paintCameraStatus();paintMapFlow();};
document.addEventListener('click',event=>{
 const button=event.target.closest('[data-drive-view]');if(!button||button.disabled)return;
 for(const cam of ['front','rear'])cameraClose(cam);driveView=button.dataset.driveView;$('#drive-feeds').className='drive-feeds view-'+driveView;
 $$('[data-drive-view]').forEach(b=>b.classList.toggle('active',b.dataset.driveView===driveView));if(S.mode==='live')for(const cam of activeDriveCameras())cameraOpen(cam);
});
setInterval(()=>{if(!document.hidden&&['maps','navigation'].includes(page))loadMapLibrary().catch(()=>{});},5000);
