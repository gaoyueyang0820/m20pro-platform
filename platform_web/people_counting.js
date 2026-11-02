'use strict';
function peoplePanel(){return `<section class="card people-panel"><div class="card-header"><div><span class="eyebrow">YOLO12n · PEOPLE COUNTING</span><h2 class="card-title">人数统计</h2></div><span class="pill gray" id="people-state">未启动</span></div><div class="people-body"><div class="people-control"><div class="people-kpis"><div><strong id="people-current">—</strong><span>当前画面人数</span></div><div><strong id="people-peak">0</strong><span>本轮峰值</span></div><div><strong id="people-total">0</strong><span>累计轨迹数</span></div></div><div class="form-grid">${select('统计视角','people-camera',[['front','前置相机'],['rear','后置相机']])}<label class="field">置信度 <span id="people-conf-label">0.30</span><input id="people-confidence" type="range" min="0.1" max="0.9" step="0.05" value="0.3"></label></div><div class="button-row">${btn('开始统计','people-start','primary','play')}${btn('暂停','people-pause')}${btn('清零本轮','people-reset')}${btn('停止','people-stop')}</div><div class="button-row section-gap">${btn('保存截图','people-snapshot','small','download')}${btn('用图片检查人数','check-counting','small','vision')}</div><p id="people-note" class="people-note" role="status">连接机器人后开始统计，也可先查看图片识别结果。</p><p class="help">累计值按人员轨迹计数；同一人离开画面后再出现可能重复计入。</p><small id="people-performance">YOLO12n · 共用相机连接</small></div><div class="people-view"><img id="people-preview" alt="人数统计实时检测框" hidden><div id="people-placeholder"><strong>人数统计画面</strong><span>${S.mode==='demo'?'当前未连接机器人，可使用图片检查':'开启后显示检测框和跟踪编号'}</span></div><div id="people-video-badge" class="people-video-badge" hidden>等待画面</div></div></div></section>`;}
const peopleVision=vision;
vision=function(){
 const el=document.createElement('div');el.innerHTML=peopleVision();
 const slot=el.querySelector('[data-vision-pane="people"]');if(slot)slot.innerHTML=peoplePanel();
 const start=el.querySelector('[data-ui="ppe-start"]'),stop=el.querySelector('[data-ui="ppe-stop"]');if(start)start.innerHTML=icon('play')+'启动安全帽识别';if(stop)stop.innerHTML=icon('stop')+'停止安全帽识别';
 return el.innerHTML;
};
let peoplePreview=null;
function closePeoplePreview(){if(!peoplePreview)return;peoplePreview.closed=true;peoplePreview.controller?.abort();if(peoplePreview.url)URL.revokeObjectURL(peoplePreview.url);peoplePreview=null;}
function openPeoplePreview(){
 const image=$('#people-preview');if(!previewVisible(image)||peoplePreview||S.people_counts?.paused)return;
 const state={closed:false,sequence:-1,url:null};peoplePreview=state;
 (async()=>{
  while(!state.closed&&image.isConnected){
   if(document.hidden||S.people_counts?.paused){await new Promise(r=>setTimeout(r,250));continue;}
   state.controller=new AbortController();const timer=setTimeout(()=>state.controller.abort(),4000);let pending=null;
   try{
    const r=await fetch('/api/people/frame?after='+state.sequence,{cache:'no-store',signal:state.controller.signal});if(!r.ok)throw Error((await r.json()).err||'等待统计画面');
    const blob=await r.blob();if(state.closed||!previewVisible(image))break;pending=URL.createObjectURL(blob);const decoded=new Image();decoded.src=pending;await decoded.decode();if(state.closed||!previewVisible(image))break;
    const old=state.url;state.url=pending;pending=null;image.src=state.url;image.hidden=false;if(old)URL.revokeObjectURL(old);
    state.sequence=+r.headers.get('X-Frame-Sequence');state.received=performance.now();state.age=+r.headers.get('X-Frame-Age')||0;$('#people-placeholder')?.setAttribute('hidden','');
   }catch(e){if(pending){URL.revokeObjectURL(pending);pending=null;}if(state.closed)break;await new Promise(r=>setTimeout(r,300));}
   finally{clearTimeout(timer);if(pending)URL.revokeObjectURL(pending);}
  }
  if(peoplePreview===state)closePeoplePreview();
 })();
}
function paintPeople(){
 if(page!=='vision'||!$('#people-state'))return;const c=S.people_counts||{},enabled=!!c.enabled;
 text('people-state',c.paused?'已暂停':c.fresh?'统计中':enabled?'等待画面':c.error?'需要处理':'未启动');
 text('people-current',c.fresh?c.current:'—');text('people-peak',c.peak??0);text('people-total',c.total_tracks??0);
 text('people-performance',`${c.fps||0} 统计帧/秒 · ${c.infer_ms||0} ms / 次 · YOLO12n + BoT-SORT`);
 text('people-note',c.error||(S.mode==='demo'?'机器人未连接，可先用图片检查人数。':c.paused?'统计已暂停，累计和峰值保留。':enabled?'正在统计所选画面；每次启动开始新一轮。':'请选择视角并开始统计。'));
 for(const [id,disabled] of [['people-start',S.mode==='demo'||enabled],['people-pause',!enabled],['people-stop',!enabled],['people-snapshot',!c.fresh]]){const el=$(`[data-ui="${id}"]`);if(el)el.disabled=disabled;}
 const pause=$('[data-ui="people-pause"]');if(pause)pause.textContent=c.paused?'继续':'暂停';
 $('#people-camera').disabled=enabled;
 if(enabled)$('#people-camera').value=c.camera;
 if(document.activeElement!==$('#people-confidence'))$('#people-confidence').value=c.conf??.3;
 text('people-conf-label',Number($('#people-confidence').value).toFixed(2));
 const image=$('#people-preview'),badge=$('#people-video-badge');
 if(enabled&&!c.paused&&previewVisible(image)){if(!peoplePreview)openPeoplePreview();}else closePeoplePreview();
 const fresh=c.fresh&&peoplePreview?.received&&performance.now()-peoplePreview.received+(peoplePreview.age||0)*1000<1200;
 if(badge){badge.hidden=!enabled||!!fresh;badge.textContent=c.paused?'已暂停 · 保留最后画面':'画面尚未更新';}
 if(image)image.classList.toggle('camera-stale',enabled&&!fresh);
 if(!enabled){closePeoplePreview();if(image){image.hidden=true;image.removeAttribute('src');}$('#people-placeholder')?.removeAttribute('hidden');}
}
ui['people-start']=async()=>{await api('/api/people',{action:'start',camera:$('#people-camera').value,conf:+$('#people-confidence').value});await fetchState();paintPeople();};
ui['people-stop']=async()=>{await api('/api/people',{action:'stop'});closePeoplePreview();await fetchState();};
ui['people-pause']=async()=>{await api('/api/people',{action:S.people_counts?.paused?'resume':'pause'});await fetchState();};
ui['people-reset']=async()=>{await api('/api/people',{action:'reset'});await fetchState();toast('本轮累计与峰值已清零，下一批画面重新计数');};
ui['people-snapshot']=()=>{const a=document.createElement('a');a.href='/api/people/snapshot';a.download='m20-people-count.jpg';a.click();};
document.addEventListener('input',e=>{if(e.target.id==='people-confidence')text('people-conf-label',Number(e.target.value).toFixed(2));});
document.addEventListener('change',async e=>{if(e.target.id==='people-confidence'){try{await api('/api/people',{action:'configure',conf:+e.target.value});await fetchState();}catch(err){toast(err.message,true);}}});
const peopleUpdate=update;update=function(){peopleUpdate();paintPeople();};
const peopleRender=render;render=function(){closePeoplePreview();peopleRender();paintPeople();};
document.addEventListener('visibilitychange',()=>{if(document.hidden)closePeoplePreview();else paintPeople();});
