'use strict';

// Preview ownership is independent of inference/navigation task ownership.
const cameraSessions=new Map();
const cameraProfiles={
 standard:{label:'标准预览',fps:12,maxStreams:2,remote:false,help:'机器人原码流。关闭或隐藏画面后停止本预览取流。'},
 economy:{label:'电脑节流',fps:2,maxStreams:1,remote:false,help:'仅降低电脑到网页的图量，机器人到电脑仍是原码流。'},
 weak:{label:'弱网预览',fps:2,maxStreams:1,remote:true,help:'机载单路小图，最多 2 帧/秒。不用于实时驾驶安全判断。'},
 snapshot:{label:'按需快照',fps:0,maxStreams:1,remote:true,help:'仅在点击后取一张机载小图，不连续拉流。不用于实时驾驶判断。'}
};
let cameraProfile='standard',cameraPreviewPaused=false,cameraPageSuspended=false;
try{const saved=sessionStorage.getItem('m20-camera-profile');if(cameraProfiles[saved])cameraProfile=saved;}catch{}
function cameraConsumer(){return 'preview-'+Date.now().toString(36)+'-'+Math.random().toString(36).slice(2)+'-'+Math.random().toString(36).slice(2);}
function cameraScope(){return JSON.stringify([S?.mode,S?.ip,S?.server_instance,S?.connection_generation]);}
function cameraUnavailableReason(cam){
 if(S?.basic?.Sleep===1)return '机器人已休眠，相机未开启；请唤醒后再查看画面';
 if(S?.device?.DevEnable?.Video?.[cam==='front'?'Front':'Back']===0)return (cam==='front'?'前视':'后视')+'相机回报关闭；请先恢复相机开启状态';
 return '';
}
// Test the container: a new image is hidden until a complete JPEG decodes.
function previewVisible(image){
 if(document.hidden||!image?.isConnected||!image.parentElement?.getClientRects().length)return false;
 const rect=image.parentElement.getBoundingClientRect?.();
 if(rect&&typeof window!=='undefined'&&window.innerHeight&&window.innerWidth)
  return rect.bottom>0&&rect.right>0&&rect.top<window.innerHeight&&rect.left<window.innerWidth;
 return true;
}
function cameraWake(state){clearTimeout(state.waitTimer);const wake=state.wake;state.wake=null;if(wake)wake();}
function cameraWait(state,ms){return new Promise(resolve=>{state.wake=resolve;state.waitTimer=setTimeout(()=>{state.wake=null;resolve();},Math.max(0,ms));});}
function releaseCameraDemand(state){
 const consumer=state.consumer;state.consumer=null;
 if(!consumer||state.overlay)return;
 // A dropped release is safe: the server and robot each expire demand. Keepalive
 // lets pagehide release it without using a credential-bearing beacon URL.
 fetch('/api/camera/release',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({consumer}),cache:'no-store',keepalive:true}).catch(()=>{});
}
function suspendCamera(state){
 state.revision++;clearTimeout(state.requestTimer);state.controller?.abort();releaseCameraDemand(state);state.suspended=true;cameraWake(state);
}
function syncCameraVisibility(){
 for(const [cam,state] of cameraSessions){
  const image=$('#camera-'+cam),scope=cameraScope();
  if(state.scope!==scope){
   suspendCamera(state);state.scope=scope;state.sequence=-1;state.times=[];
   if(state.url)URL.revokeObjectURL(state.url);state.url=null;
   if(image){image.removeAttribute('src');image.hidden=true;delete image.dataset.received;}
  }
  const visible=previewVisible(image)&&S.mode==='live'&&!cameraPageSuspended&&!cameraPreviewPaused&&!cameraUnavailableReason(cam);
  if(!visible&&!state.suspended)suspendCamera(state);
  if(visible&&state.suspended){state.suspended=false;state.consumer=state.overlay?null:cameraConsumer();state.sequence=-1;cameraWake(state);}
 }
}
document.addEventListener('visibilitychange',syncCameraVisibility);
document.addEventListener('click',event=>{if(event.target.closest('[data-vision-tab]'))syncCameraVisibility();});
if(typeof window!=='undefined'){
 window.addEventListener('pagehide',()=>{cameraPageSuspended=true;for(const state of cameraSessions.values())suspendCamera(state);});
 window.addEventListener('pageshow',()=>{cameraPageSuspended=false;syncCameraVisibility();});
}
function cameraClose(cam){
 const state=cameraSessions.get(cam);
 if(state){state.closed=true;suspendCamera(state);clearInterval(state.timer);state.observer?.disconnect();if(state.url)URL.revokeObjectURL(state.url);cameraSessions.delete(cam);}
 const image=$('#camera-'+cam);
 if(image){image.removeAttribute('src');for(const key of ['requested','overlay','received','roundTrip','sourceAge','fps','width','height'])delete image.dataset[key];image.hidden=true;image.classList.remove('camera-stale');}
 $('#placeholder-'+cam)?.removeAttribute('hidden');$('#camera-notice-'+cam)?.remove();
}
function activeDriveCameras(){return driveView==='both'&&cameraProfiles[cameraProfile].maxStreams>1?['front','rear']:[driveView==='rear'?'rear':'front'];}
function cameraOpen(cam,{snapshot=false}={}){
 if(S.mode==='demo'||!['front','rear'].includes(cam))return;
 const image=$('#camera-'+cam);if(!image)return;
 if(cameraProfiles[cameraProfile].maxStreams===1)for(const other of [...cameraSessions.keys()])if(other!==cam)cameraClose(other);
 cameraClose(cam);image.dataset.requested='true';
 const hasOverlay=()=>cameraProfile==='standard'&&$('#ppe-overlay')?.checked&&(S.ppe?.cameras?.[cam]?.enabled??(S.ppe?.enabled&&S.ppe.camera===cam));
 const overlay=!!hasOverlay();if(overlay)image.dataset.overlay='true';
 const notice=document.createElement('div');notice.id='camera-notice-'+cam;notice.className='camera-notice';notice.hidden=true;notice.setAttribute('role','status');image.parentElement.append(notice);
 const state={closed:false,sequence:-1,failures:0,url:null,times:[],error:'',revision:0,suspended:true,consumer:null,overlay,scope:cameraScope(),profile:cameraProfile,snapshotRequested:snapshot};
 cameraSessions.set(cam,state);
 state.timer=setInterval(()=>{if(!document.hidden)paintCameraStatus();},1000);
 if(typeof IntersectionObserver!=='undefined'){state.observer=new IntersectionObserver(syncCameraVisibility);state.observer.observe(image.parentElement);}
 syncCameraVisibility();
 const poll=async()=>{
  while(!state.closed&&image.isConnected){
   if(Boolean(overlay)!==Boolean(hasOverlay())){cameraOpen(cam);return;}
   syncCameraVisibility();
   if(state.suspended||state.profile==='snapshot'&&!state.snapshotRequested){await cameraWait(state,250);continue;}
   const revision=state.revision,profile=state.profile,spec=cameraProfiles[profile];
   if(!state.consumer&&!overlay)state.consumer=cameraConsumer();
   state.controller=new AbortController();
   const timer=setTimeout(()=>state.controller.abort(),state.sequence<0?(spec.remote?9000:6500):(spec.remote?4500:2500));state.requestTimer=timer;
   const started=performance.now();let pendingURL=null,delay=0;
   try{
    const url=(overlay?'/api/ppe/frame?camera=':'/api/camera/frame?camera=')+cam+'&after='+state.sequence+(overlay?'':'&profile='+profile+'&consumer='+encodeURIComponent(state.consumer));
    const response=await fetch(url,{cache:'no-store',signal:state.controller.signal});
    if(!response.ok){
     let detail={};try{detail=await response.json();}catch{}
     const error=Error(detail.err||'视频暂时不可用');error.code=detail.code;
     error.retryAfter=Number(detail.retry_after??response.headers.get('Retry-After'));throw error;
    }
    const sequence=Number(response.headers.get('X-Frame-Sequence')),sourceAge=Number(response.headers.get('X-Frame-Age'));
    if(!Number.isSafeInteger(sequence)||sequence<0||!Number.isFinite(sourceAge)||sourceAge<0)throw Error('视频帧时间或序号无效');
    const blob=await response.blob();if(state.closed)break;
    if(revision!==state.revision||state.scope!==cameraScope()||!previewVisible(image)||state.suspended)continue;
    pendingURL=URL.createObjectURL(blob);const decoded=new Image();decoded.src=pendingURL;await decoded.decode();
    if(state.closed)break;
    if(revision!==state.revision||state.scope!==cameraScope()||!previewVisible(image)||state.suspended)continue;
    const now=performance.now(),old=state.url;state.url=pendingURL;pendingURL=null;
    image.src=state.url;image.hidden=false;if(old)URL.revokeObjectURL(old);
    state.sequence=sequence;state.failures=0;state.error='';
    state.times=state.times.filter(t=>now-t<2000);state.times.push(now);
    image.dataset.fps=state.times.length>4&&now-state.times[0]>=500?((state.times.length-1)*1000/(now-state.times[0])).toFixed(1):'—';
    image.dataset.received=String(now);image.dataset.roundTrip=String(Math.round(now-started));
    // Conservative browser age: do not reset frame age at download/decode time.
    image.dataset.sourceAge=String(sourceAge*1000+(now-started));
    image.dataset.width=String(decoded.naturalWidth||Number(response.headers.get('X-Frame-Width'))||0);
    image.dataset.height=String(decoded.naturalHeight||Number(response.headers.get('X-Frame-Height'))||0);
    if(profile==='snapshot'){state.snapshotRequested=false;releaseCameraDemand(state);}
    paintCameraStatus();
    delay=spec.fps?Math.max(0,1000/spec.fps-(performance.now()-started)):0;
   }catch(e){
    if(state.closed)break;
    if(revision!==state.revision||!previewVisible(image)||state.suspended)continue;
    if(e.code==='cancelled'){releaseCameraDemand(state);state.consumer=overlay?null:cameraConsumer();state.sequence=-1;}
    state.failures++;state.error=e.name==='AbortError'?'连接超时，正在退避重试':e.message;
    const backoff=Math.min(8000,500*2**Math.min(state.failures-1,4));
    delay=Math.max(backoff,Number.isFinite(e.retryAfter)?Math.min(8000,Math.max(0,e.retryAfter*1000)):0);
    if(profile==='snapshot'){state.snapshotRequested=false;releaseCameraDemand(state);}
    paintCameraStatus();
   }finally{clearTimeout(timer);if(pendingURL)URL.revokeObjectURL(pendingURL);}
   if(delay&&!state.closed)await cameraWait(state,delay);
  }
  if(cameraSessions.get(cam)===state)cameraClose(cam);
 };
 poll();
}
function cameraProfileControls(){
 return `<div id="camera-preview-controls" class="camera-preview-controls"><label>视频档位 <select data-camera-profile aria-label="视频档位">${Object.entries(cameraProfiles).map(([key,spec])=>`<option value="${key}" ${cameraProfile===key?'selected':''}>${spec.label}</option>`).join('')}</select></label><div class="button-row"><button class="btn small" type="button" data-ui="camera-preview-pause">${cameraPreviewPaused?'继续预览':'暂停预览'}</button><button class="btn small" type="button" data-ui="camera-preview-snapshot" ${cameraProfile==='snapshot'?'':'hidden'}>刷新快照</button></div><p class="help" id="camera-preview-help">${cameraProfiles[cameraProfile].help}</p></div>`;
}
function mountCameraPreviewControls(){
 if($('#camera-preview-controls'))return;
 const slot=page==='navigation'?$('#nav-camera-preview'):page==='vision'?$('.camera-grid'):null;
 if(!slot)return;
 const controls=document.createElement('div');controls.innerHTML=cameraProfileControls();
 if(page==='navigation')slot.prepend?.(controls);else slot.before?.(controls);
}
function setCameraProfile(profile){
 if(!cameraProfiles[profile]||profile===cameraProfile)return;
 const requested=[...cameraSessions.keys()];for(const cam of requested)cameraClose(cam);
 cameraProfile=profile;try{sessionStorage.setItem('m20-camera-profile',profile);}catch{}
 if(cameraProfiles[profile].maxStreams===1&&driveView==='both')driveView='front';
 const feeds=$('#drive-feeds');if(feeds)feeds.className='drive-feeds view-'+driveView;
 const selector=$('[data-camera-profile]');if(selector)selector.value=profile;
 if(typeof $$==='function')$$('[data-drive-view]').forEach(button=>button.classList.toggle('active',button.dataset.driveView===driveView));
 const views=page==='motion'?activeDriveCameras():requested.slice(0,cameraProfiles[profile].maxStreams);
 for(const cam of views)cameraOpen(cam);
 paintCameraStatus();
}
function takeCameraSnapshot(){
 if(cameraProfile!=='snapshot')return;
 cameraPreviewPaused=false;
 const selected=page==='motion'?activeDriveCameras()[0]:[...cameraSessions.keys()][0]||'front';
 const state=cameraSessions.get(selected);
 if(!state){cameraOpen(selected,{snapshot:true});return;}
 suspendCamera(state);state.snapshotRequested=true;state.sequence=-1;state.failures=0;syncCameraVisibility();cameraWake(state);paintCameraStatus();
}
for(const cam of ['front','rear']){
 ui['camera-'+cam]=()=>cameraOpen(cam,{snapshot:cameraProfile==='snapshot'});
 ui['camera-close-'+cam]=()=>cameraClose(cam);
}
ui['camera-preview-pause']=()=>{cameraPreviewPaused=!cameraPreviewPaused;syncCameraVisibility();paintCameraStatus();};
ui['camera-preview-snapshot']=takeCameraSnapshot;
document.addEventListener('change',event=>{if(event.target.matches?.('[data-camera-profile]'))setCameraProfile(event.target.value);});
ui['nav-camera-toggle']=()=>{
 const preview=$('#nav-camera-preview'),toggle=$('#nav-camera-toggle');if(!preview||!toggle)return;
 const open=preview.hidden;preview.hidden=!open;toggle.setAttribute('aria-expanded',String(open));
 if(open&&S.mode==='live')cameraOpen('front');
 if(!open){cameraClose('front');toggle.focus({preventScroll:true});}
};
ui['drive-camera-retry']=async()=>{
 if(S.mode!=='live')return;
 if(cameraProfile==='snapshot'){takeCameraSnapshot();return;}
 for(const cam of activeDriveCameras()){
  if(cameraUnavailableReason(cam))continue;
  // Weak mode must not open/reset the PC's original-bitrate decoder.
  if(!cameraProfiles[cameraProfile].remote)await api('/api/camera/reconnect',{camera:cam},true);
  cameraOpen(cam);
 }
};
function paintCameraStatus(){
 if(!['motion','vision','navigation'].includes(page)||!S)return;
 syncCameraVisibility();const spec=cameraProfiles[cameraProfile];
 const retry=$('[data-ui="drive-camera-retry"]');if(retry){retry.disabled=activeDriveCameras().every(cam=>!!cameraUnavailableReason(cam));retry.title=retry.disabled?'相机未开启，恢复后可查看':'重新请求所选预览；不重启机器人相机服务';}
 const pause=$('[data-ui="camera-preview-pause"]');if(pause)pause.textContent=cameraPreviewPaused?'继续预览':'暂停预览';
 const shot=$('[data-ui="camera-preview-snapshot"]');if(shot){shot.hidden=cameraProfile!=='snapshot';shot.disabled=S.mode!=='live';}
 const both=$('[data-drive-view="both"]');if(both){both.disabled=spec.maxStreams===1;both.title=both.disabled?'此档位仅查看单路相机':'';}
 text('camera-preview-help',spec.help+(spec.remote&&(S.ppe?.enabled||S.people_counts?.enabled||S.autonomy?.vision?.enabled)?' 独立识别任务仍可能取流；本预览不叠加检测框。':''));
 let live=0;
 for(const cam of ['front','rear']){
  const image=$('#camera-'+cam),status=S.cameras?.[cam],session=cameraSessions.get(cam);if(!image||!session)continue;
  const received=Number(image.dataset.received||0),overlay=image.dataset.overlay==='true';
  const hasFrame=received>0&&image.naturalWidth>0;
  const age=hasFrame?performance.now()-received+Number(image.dataset.sourceAge||0):Infinity;
  const unavailable=cameraUnavailableReason(cam),snapshot=session.profile==='snapshot';
  const fresh=hasFrame&&age<1000&&!unavailable&&!session.suspended&&!snapshot;if(fresh)live++;
  image.classList.toggle('camera-stale',hasFrame&&!fresh);
  const placeholder=$('#placeholder-'+cam);if(placeholder)placeholder.hidden=hasFrame;
  const idleMessage=cameraPreviewPaused?'预览已暂停，点击继续预览':snapshot?'点击“刷新快照”获取一张画面':session.suspended?'当前视角已暂停取流':'正在连接相机…';
  const message=unavailable||(S.cameras?.available===false&&!spec.remote?'请安装视频依赖后重启平台':session.error||(!spec.remote&&status?.error)||idleMessage);
  const label=placeholder?.querySelector('strong');if(label)label.textContent=message;
  const ageText=Number.isFinite(age)?(age/1000).toFixed(1)+' 秒':'—';
  const notice=$('#camera-notice-'+cam);if(notice){notice.hidden=!hasFrame||fresh;if(!notice.hidden)notice.textContent=snapshot?'按需快照 · 图龄 '+ageText+' · 不连续更新':'画面已暂停 · 图龄 '+ageText+' · '+message;}
  const fps=Number(image.dataset.fps),latency=Number(image.dataset.roundTrip);
  const tag=snapshot&&hasFrame?'按需快照 · '+ageText:fresh?(spec.remote?'弱网预览 · ':(overlay?'YOLO · ':''))+(Number.isFinite(fps)?Math.round(fps)+' 帧/秒':'—')+' · 取图 '+(Number.isFinite(latency)?Math.round(latency/10)*10+' ms':'—'):hasFrame?'画面暂停':'等待画面';
  text('cam-'+cam+'-tag',tag);
  text('cam-'+cam+'-note',hasFrame?`${Number(image.dataset.width)||status?.width||'—'} × ${Number(image.dataset.height)||status?.height||'—'} · ${cameraProfiles[session.profile].label}`:message);
 }
 text('drive-video-state',S.mode==='demo'?'演示模式 · 未连接真实相机':cameraPreviewPaused?'预览已暂停':cameraProfile==='snapshot'?'按需快照 · 非连续画面':spec.remote?'弱网低帧预览 · 非实时驾驶依据':live===2?'前后视角预览':live?'相机预览':'等待相机画面');
}
