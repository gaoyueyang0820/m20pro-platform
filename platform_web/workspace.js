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
 const feeds=['front','rear'].map((cam,i)=>`<div class="drive-feed drive-feed-${cam}"><div class="camera-feed"><img id="camera-${cam}" alt="${i?'后置':'前置'}相机" hidden><div class="camera-placeholder" id="placeholder-${cam}">${icon('vision')}<strong>${S.mode==='demo'?'演示模式没有实机画面':'正在连接'+(i?'后置':'前置')+'相机…'}</strong><small>${i?'后视':'前视'} · 实时画面</small></div><div class="camera-hud"><span>${i?'后视':'前视'}</span><span id="cam-${cam}-tag">等待画面</span></div></div><span class="drive-camera-note" id="cam-${cam}-note"></span></div>`).join('');
 const video=card('驾驶视角',`<div class="drive-camera-toolbar"><div class="segmented">${[['front','前视'],['rear','后视'],['both','双视角']].map(([v,label])=>`<button data-drive-view="${v}" class="${driveView===v?'active':''}">${label}</button>`).join('')}</div><div class="button-row">${btn('重连画面','drive-camera-retry','small')}${btn('YOLO / 视觉识别','drive-vision','small')}</div></div><div id="drive-feeds" class="drive-feeds view-${driveView}">${feeds}</div><div class="drive-camera-footer"><span id="drive-video-state">等待相机反馈</span><span>画面卡顿时可点击重连</span></div>`,'vision');
 return heading(btn('停稳并建图','start-map-here','','maps')+cmd('停止运动','stop','danger'))+
 `<div class="drive-layout">
   <div class="drive-camera-panel">${video}</div>
   <div class="drive-control-panel">${control?control.outerHTML:''}${take('姿态与步态')}</div>
   <div class="drive-settings drive-settings-secondary">${take('照明与方向')}${take('电源与充电')}</div>
  </div>
  <div id="drive-advanced"></div>`;
};

ui['drive-vision']=()=>navigate('vision');

const cameraSessions=new Map();
function cameraUnavailableReason(cam){
 if(S?.basic?.Sleep===1)return '机器人已休眠，相机未开启；请唤醒后再查看画面';
 if(S?.device?.DevEnable?.Video?.[cam==='front'?'Front':'Back']===0)return (cam==='front'?'前视':'后视')+'相机回报关闭；请先恢复相机开启状态';
 return '';
}
// The image itself starts hidden. Test its container so an unopened frame can load,
// while CSS-hidden views, collapsed panels and background tabs do not pull frames.
function previewVisible(image){return !document.hidden&&!!image?.isConnected&&!!image.parentElement?.getClientRects().length;}
function syncCameraVisibility(){
 for(const [cam,state] of cameraSessions){
  const visible=previewVisible($('#camera-'+cam))&&!cameraUnavailableReason(cam);
  if(!visible&&!state.suspended){state.revision++;state.controller?.abort();}
  state.suspended=!visible;
 }
}
document.addEventListener('visibilitychange',syncCameraVisibility);
document.addEventListener('click',event=>{if(event.target.closest('[data-vision-tab]'))syncCameraVisibility();});
function cameraClose(cam){
 const state=cameraSessions.get(cam);if(state){state.closed=true;clearInterval(state.timer);state.controller?.abort();if(state.url)URL.revokeObjectURL(state.url);cameraSessions.delete(cam);}
 const image=$('#camera-'+cam);if(image){image.removeAttribute('src');for(const key of ['requested','overlay','received','roundTrip','sourceAge','fps'])delete image.dataset[key];image.hidden=true;image.classList.remove('camera-stale');}
 $('#placeholder-'+cam)?.removeAttribute('hidden');$('#camera-notice-'+cam)?.remove();
}
function activeDriveCameras(){return driveView==='both'?['front','rear']:[driveView];}
cameraOpen=function(cam){
 if(S.mode==='demo')return;
 const image=$('#camera-'+cam);if(!image)return;
 cameraClose(cam);image.dataset.requested='true';
 const hasOverlay=()=>$('#ppe-overlay')?.checked&&(S.ppe.cameras?.[cam]?.enabled??(S.ppe.enabled&&S.ppe.camera===cam));
 const overlay=hasOverlay();
 if(overlay)image.dataset.overlay='true';
 const notice=document.createElement('div');notice.id='camera-notice-'+cam;notice.className='camera-notice';notice.hidden=true;notice.setAttribute('role','status');image.parentElement.append(notice);
 const state={closed:false,sequence:-1,failures:0,url:null,times:[],error:'',revision:0,suspended:!previewVisible(image)};cameraSessions.set(cam,state);
 state.timer=setInterval(()=>{if(!document.hidden)paintCameraStatus();},1000);
 const poll=async()=>{
  while(!state.closed&&image.isConnected){
   if(Boolean(overlay)!==Boolean(hasOverlay())){cameraOpen(cam);return;}
   if(!previewVisible(image)||cameraUnavailableReason(cam)){await new Promise(r=>setTimeout(r,250));continue;}
   state.suspended=false;
   const revision=state.revision;
   state.controller=new AbortController();const timer=setTimeout(()=>state.controller.abort(),state.sequence<0?5500:2500),started=performance.now();let pendingURL=null;
   try{
    const response=await fetch((overlay?'/api/ppe/frame?camera=':'/api/camera/frame?camera=')+cam+'&after='+state.sequence,{cache:'no-store',signal:state.controller.signal});
    if(!response.ok){const err=await response.json();throw Error(err.err||'视频暂时不可用');}
    const blob=await response.blob();if(state.closed)break;
    if(revision!==state.revision||!previewVisible(image)||cameraUnavailableReason(cam))continue;
    // Decode offscreen; a pending or broken image must never replace the visible frame.
    pendingURL=URL.createObjectURL(blob);const decoded=new Image();decoded.src=pendingURL;await decoded.decode();
    if(state.closed)break;
    if(revision!==state.revision||!previewVisible(image)||cameraUnavailableReason(cam))continue;
    const now=performance.now(),old=state.url;state.url=pendingURL;pendingURL=null;
    image.src=state.url;image.hidden=false;if(old)URL.revokeObjectURL(old);
    state.sequence=+response.headers.get('X-Frame-Sequence');state.failures=0;state.error='';
    state.times=state.times.filter(t=>now-t<2000);state.times.push(now);
    image.dataset.fps=state.times.length>4&&now-state.times[0]>=500?((state.times.length-1)*1000/(now-state.times[0])).toFixed(1):'—';
    image.dataset.received=String(now);image.dataset.roundTrip=String(Math.round(now-started));
    image.dataset.sourceAge=String(Math.max(0,Number(response.headers.get('X-Frame-Age')||0))*1000);
    paintCameraStatus();
   }catch(e){
    if(pendingURL){URL.revokeObjectURL(pendingURL);pendingURL=null;}
    if(state.closed)break;
    if(revision!==state.revision||!previewVisible(image)||cameraUnavailableReason(cam))continue;
    state.failures++;state.error=e.name==='AbortError'?'连接超时，正在重试':e.message;paintCameraStatus();
    await new Promise(r=>setTimeout(r,Math.min(1200,200*state.failures)));
   }finally{clearTimeout(timer);if(pendingURL)URL.revokeObjectURL(pendingURL);}
   // The server waits for a new frame; no extra sleep or queued frame requests.
  }
 };
 poll();
};
for(const cam of ['front','rear'])ui['camera-close-'+cam]=()=>cameraClose(cam);
ui['nav-camera-toggle']=()=>{
 const preview=$('#nav-camera-preview'),toggle=$('#nav-camera-toggle');
 if(!preview||!toggle)return;
 const open=preview.hidden;
 preview.hidden=!open;
 toggle.setAttribute('aria-expanded',String(open));
 if(open&&S.mode==='live')cameraOpen('front');
 if(!open){cameraClose('front');toggle.focus({preventScroll:true});}
};
ui['drive-camera-retry']=async()=>{
 if(S.mode!=='live')return;
 for(const cam of activeDriveCameras()){if(cameraUnavailableReason(cam))continue;await api('/api/camera/reconnect',{camera:cam},true);cameraOpen(cam);}
};
function paintCameraStatus(){
 if(!['motion','vision','navigation'].includes(page)||!S)return;
 syncCameraVisibility();
 const retry=$('[data-ui="drive-camera-retry"]');if(retry){retry.disabled=activeDriveCameras().every(cam=>!!cameraUnavailableReason(cam));retry.title=retry.disabled?'相机未开启，恢复后会自动取图':'重新建立所选相机连接';}
 let live=0;
 for(const cam of ['front','rear']){
  const image=$('#camera-'+cam),status=S.cameras?.[cam],session=cameraSessions.get(cam);if(!image||!session)continue;
  const received=Number(image.dataset.received||0),overlay=image.dataset.overlay==='true';
  const hasFrame=received>0&&image.naturalWidth>0;
  const age=hasFrame?performance.now()-received+Number(image.dataset.sourceAge||0):Infinity;
  const unavailable=cameraUnavailableReason(cam),fresh=hasFrame&&age<1000&&!unavailable;if(fresh)live++;
  image.classList.toggle('camera-stale',hasFrame&&!fresh);
  const placeholder=$('#placeholder-'+cam);if(placeholder)placeholder.hidden=hasFrame;
  const message=S.mode==='live'&&!S.online?'机器人未连接 · 等待开机或连接网络':unavailable|| (S.cameras?.available===false?'请安装视频依赖后重启平台':session.error||status?.error||'正在连接相机…');
  const label=placeholder?.querySelector('strong');if(label)label.textContent=message;
  const notice=$('#camera-notice-'+cam);if(notice){notice.hidden=!hasFrame||fresh;if(!notice.hidden)notice.textContent='画面已暂停 · 等待恢复 · '+message;}
  const fps=Number(image.dataset.fps),latency=Number(image.dataset.roundTrip);
  const tag=fresh?(overlay?'YOLO · ':'')+(Number.isFinite(fps)?Math.round(fps)+' 帧/秒':'—')+' · 取图 '+(Number.isFinite(latency)?Math.round(latency/10)*10+' ms':'—'):hasFrame?'画面暂停':'等待画面';
  text('cam-'+cam+'-tag',tag);
  text('cam-'+cam+'-note',fresh?status?.width+' × '+status?.height:message);
 }
 text('drive-video-state',S.mode==='demo'?'演示模式 · 未连接真实相机':!S.online?'机器人未连接 · 等待开机':live===2?'前后视角实时画面':live?(driveView==='rear'?'后视':'前视')+'实时画面':'等待相机画面');
}

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
 if(page==='motion'&&S.mode==='live')for(const cam of activeDriveCameras())cameraOpen(cam);
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
 const button=event.target.closest('[data-drive-view]');if(!button)return;
 for(const cam of ['front','rear'])cameraClose(cam);driveView=button.dataset.driveView;$('#drive-feeds').className='drive-feeds view-'+driveView;
 $$('[data-drive-view]').forEach(b=>b.classList.toggle('active',b.dataset.driveView===driveView));if(S.mode==='live')for(const cam of activeDriveCameras())cameraOpen(cam);
});
setInterval(()=>{if(!document.hidden&&['maps','navigation'].includes(page))loadMapLibrary().catch(()=>{});},5000);
