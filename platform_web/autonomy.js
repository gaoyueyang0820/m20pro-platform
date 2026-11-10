'use strict';
pages.autonomy=['自主作业','AUTONOMOUS OPERATIONS','自主探索、更新地图或跟随人员，随时暂停与手动接管。'];
paths.autonomy=paths.spark;
let autoTab='explore',autoVision=null,autoVisionBusy=false;
let autoObservationClock=null,autoStateReceipt=null;
let autoStartIntent=null,autoStartGeneration=0,autoStartNotice='';
let autoLifecyclePending=null;
function cancelAutoResumeRequest(){if(autoLifecyclePending?.kind==='resume'){autoLifecyclePending.cancelled=true;autoLifecyclePending.controller?.abort();}}
const autoFormIds=['auto-map-kind','auto-scope','auto-radius','auto-minutes','auto-forward','auto-yaw'];
function autoFormKey(){return JSON.stringify(autoFormIds.map(id=>$('#'+id)?.value));}
function autoStartCurrent(intent){
 return autoStartIntent===intent&&intent.generation===autoStartGeneration&&page==='autonomy'&&location.hash==='#autonomy'&&autoTab==='explore'&&!document.hidden&&S.mode===intent.mode&&S.ip===intent.ip&&S.online!==false&&autoFormKey()===intent.form;
}
function invalidateAutoStart(message='已取消自动开始；已经提交的定位搜索可能仍在进行'){
 const intent=autoStartIntent;autoStartGeneration++;autoStartIntent=null;
 if(intent){intent.readAbort?.abort();autoStartNotice=message;text('auto-message',message);paintAutoMapOptions();}
 else autoStartNotice='';
}
function showAutoStart(intent,message,phase=intent.phase){
 if(!autoStartCurrent(intent))return;
 intent.message=message;intent.phase=phase;text('auto-message',message);text('auto-state',phase==='localizing'?'恢复定位中':'准备中');paintAutoMapOptions();
}
async function waitAutoLocalization(start,intent){
 if(start.localized===true)return start;
 if(!start.job_id)throw Error('定位请求没有返回可查询的任务编号');
 const deadline=performance.now()+120000;
 while(autoStartCurrent(intent)){
  if(performance.now()>deadline)throw Error('等待定位超时；后端搜索可能仍在进行，请查看任务状态后重试');
  const controller=new AbortController();intent.readAbort=controller;
  const timer=setTimeout(()=>controller.abort(),5000);let response;
  try{response=await api('/api/operations?id='+encodeURIComponent(start.job_id),undefined,true,{signal:controller.signal});}
  catch(error){if(!autoStartCurrent(intent))return null;if(error.name==='AbortError')throw Error('定位状态读取超时，请检查连接后重试');throw error;}
  finally{clearTimeout(timer);if(intent.readAbort===controller)intent.readAbort=null;}
  if(!autoStartCurrent(intent))return null;
  const job=response.jobs;if(!job)throw Error('定位任务状态不可用');
  showAutoStart(intent,'正在恢复定位：'+(job.message||'等待机器人状态'),'localizing');
  if(job.status==='failed')throw Error(job.error||'自动定位失败');
  if(job.status==='completed'){
   if(job.result?.localized!==true)throw Error('定位任务已结束，但尚未确认定位恢复');
   return job.result;
  }
  await new Promise(resolve=>setTimeout(resolve,300));
 }
 return null;
}
function autoDefaultMapKind(){
 if(S.mapping)return 'continue';
 const active=typeof mapLibrary!=='undefined'&&mapLibrary.active;
 const synced=(S.maps||[]).some(m=>m.verified_for_robot&&(S.mode==='demo'||m.robot_ip===S.ip));
 return active||synced?'extend':'new';
}
function autoMapHelp(kind,scope){
 if(S?.autonomy?.can_resume)return '继续本轮探索并保留已采集区域，也可保存或放弃本轮更新。';
 if(kind==='new')return '从当前位置向外探索，保存为一张独立的新地图。';
 if(kind==='continue')return '继续当前采集，按设定半径探索并保存地图。';
 return (scope==='map'?'按活动地图范围向外扩展。':'在设定半径内探索。')+'优先补扫未扫描区域，再更新已扫描区域；与原图合并生成新版本，由你选择启用。';
}
function paintAutoMapOptions(){
 const kind=$('#auto-map-kind')?.value;if(!kind)return;
 const a=S.autonomy||{},busy=!!a.active||a.state==='saving'||a.state==='discarding'||!!a.cleanup_pending||!!autoLifecyclePending,updating=kind==='extend';
 const scope=$('#auto-scope');if(scope&&!updating)scope.value='radius';
 const mapScope=updating&&scope?.value==='map';
 const scopeField=$('#auto-scope-field');if(scopeField)scopeField.hidden=!updating;
 const radiusField=$('#auto-radius')?.closest('.field');if(radiusField)radiusField.hidden=mapScope;
 for(const id of ['auto-start','auto-relocalize','auto-map-kind','auto-scope','auto-radius','auto-minutes','auto-forward','auto-yaw']){
  const el=document.querySelector(`[data-ui="${id}"]`)||document.getElementById(id);
  if(el)el.disabled=busy||((a.can_start===false||a.preparation_unresolved)&&['auto-start','auto-relocalize'].includes(id))||(!!autoStartIntent&&['auto-start','auto-relocalize'].includes(id))||(id==='auto-scope'&&!updating)||(id==='auto-radius'&&mapScope);
 }
 const closing=a.state==='saving'||a.state==='discarding'||!!a.cleanup_pending||!!a.preparation_unresolved||!!autoLifecyclePending;
 const save=document.querySelector('[data-ui="auto-save"]');if(save)save.disabled=!S.mapping||closing;
 const resume=document.querySelector('[data-ui="auto-resume"]');if(resume){resume.hidden=!a.can_resume;resume.disabled=busy||!a.can_resume;}
 const discard=document.querySelector('[data-ui="auto-discard"]');if(discard){discard.hidden=!a.discard_supported;discard.disabled=closing||a.state==='preparing'||!a.can_discard;}
 if(a.can_resume)for(const id of ['auto-map-kind','auto-scope','auto-radius','auto-forward','auto-yaw']){const el=$('#'+id);if(el)el.disabled=true;}
 text('auto-map-help',autoMapHelp(kind,scope?.value));
}
function autonomyPage(){
 const live=S.mode==='live',a=S.autonomy||{},mapKind=autoDefaultMapKind();
 return heading(btn('手动接管','auto-manual','','motion')+btn('暂停移动','auto-stop','danger','stop'))+
 `<div class="auto-tabs"><button class="btn ${autoTab==='explore'?'primary':''}" data-ui="auto-explore-tab">自主建图</button><button class="btn ${autoTab==='follow'?'primary':''}" data-ui="auto-follow-tab">跟随人员</button><span class="pill ${live?'amber':'gray'}">${live?'实机模式':'演示模式'}</span></div>
 <div class="auto-layout"><section class="card auto-stage"><div class="card-header"><h2 class="card-title">${icon(autoTab==='explore'?'maps':'vision')}${autoTab==='explore'?'探索中的空间':'选择要跟随的人'}</h2><span class="pill" id="auto-state">待开始</span></div>
 <div class="auto-canvas"><svg id="auto-view" viewBox="0 0 680 460" role="img" aria-label="自主作业实时视图"></svg><div id="auto-empty" class="auto-empty">${autoTab==='explore'?'开始后显示实时地图、障碍与行进路径':live?'开启人员识别后，点击画面中的人物框':'开启识别后选择跟随目标'}</div></div>
 <div class="auto-legend">${autoTab==='explore'?'<span><i class="floor"></i>已观测地面</span><span><i class="obstacle"></i>障碍</span><span><i class="path"></i>规划路径</span>':'<span>点选人物即开始跟随 · 绿色框为锁定目标</span>'}</div><div class="auto-metrics"><div><strong id="auto-area">—</strong><small>${autoTab==='explore'?'已观测面积 / m²':'画面延迟 / 秒'}</small></div><div><strong id="auto-distance">—</strong><small>${autoTab==='explore'?'本轮行走 / m':'检测到的人物'}</small></div><div><strong id="auto-time">—</strong><small>运行 / 秒</small></div></div><div class="auto-legend" id="auto-observation-status" role="status">等待本轮探索观测</div><div class="auto-legend" id="auto-navigation-status" hidden></div></section>
 <section class="card card-pad auto-controls"><span class="eyebrow">${autoTab==='explore'?'EXPLORE & MAP':'PERSON FOLLOWING'}</span><h2>${autoTab==='explore'?'开始一次自主探索':'点选你要跟随的人'}</h2><p class="help">${autoTab==='explore'?'自动探索未覆盖区域并更新地图，支持暂停、继续、保存或放弃本轮。':'在期望间距站定，全身进入画面后点选自己。失去目标或多人交叉时停车。'}</p>
 <div class="auto-message" role="status" id="auto-message">${esc(autoTab==='explore'&&(autoStartIntent?.message||autoStartNotice)||a.message||'等待开始')}</div>
 ${autoTab==='explore'?`<label class="field">建图方式<select id="auto-map-kind"><option value="new" ${mapKind==='new'?'selected':''}>新建地图</option><option value="extend" ${mapKind==='extend'?'selected':''}>更新当前地图（补扫＋优化）</option><option value="continue" ${mapKind==='continue'?'selected':''}>继续正在采集的地图</option></select></label><label class="field" id="auto-scope-field" ${mapKind==='extend'?'':'hidden'}>探索范围<select id="auto-scope"><option value="map">按地图范围</option><option value="radius">指定半径</option></select></label><p class="help" id="auto-map-help">${autoMapHelp(mapKind,mapKind==='extend'?'map':'radius')}</p><div class="auto-actions">${btn('开始探索','auto-start','primary small','play')}<button class="btn primary small" data-ui="auto-resume" hidden>继续探索</button>${btn('结束并保存','auto-save','small','check')}<button class="btn small" data-ui="auto-discard" hidden>结束并放弃</button>${btn('自动恢复定位','auto-relocalize','small','refresh')}</div><div class="section-gap"><h4 class="field-heading">探索参数</h4><div class="form-grid">${field('探索半径 / 米','auto-radius',6,'number')}${field('本轮时长 / 分钟','auto-minutes',5,'number')}${field('前进幅度 / %','auto-forward',35,'number')}${field('转向幅度 / %','auto-yaw',70,'number')}</div></div>`:
 `<div class="auto-actions">${btn('开启人员识别','auto-vision-start','primary','vision')}${btn('关闭识别','auto-vision-stop')}</div><div id="auto-people" class="auto-people"></div><p id="auto-vision-error" class="help"></p><div class="section-gap"><h4 class="field-heading">YOLO 模型设置</h4><div class="field"><label for="auto-model">本机人员检测模型路径</label><input id="auto-model" placeholder="默认已配好 YOLO11n，可留空"></div><p class="help">留空使用默认人员模型，也可选择自定义模型。</p></div>`}
 
 ${live?'':'<p class="help">当前显示模拟图像与运动。</p>'}</section></div>`;
}
function autoMapObservation(m,a=S?.autonomy||{}){
 const now=performance.now();if(autoStateReceipt?.state!==S)autoStateReceipt={state:S,at:now};
 const stateAge=Math.max(0,(now-autoStateReceipt.at)/1000),o=m?.observation;
 if(!m?.floor?.length)return {state:'empty',live:false,drawPose:false,message:'等待本轮探索观测',stateAge};
 if(!o)return {state:a.active?'unverified':'history',live:false,drawPose:true,message:a.active?'正在同步实时观测，显示最近位置':'最后探索观测（非当前位置）',stateAge};
 const sameFrame=o.frame==='slam_session'&&(!m.capture_id||!o.capture_id||m.capture_id===o.capture_id);
 if(!sameFrame)return {state:'wrong_frame',live:false,drawPose:false,message:'地图位置同步中',stateAge};
 let age=Number.isFinite(o.age)&&o.age>=0?o.age+stateAge:null;
 const key=JSON.stringify([S?.server_instance,S?.mode,S?.ip,m.capture_id||o.capture_id,o.frame,o.stamp]);
 if(age!==null){
  if(autoObservationClock?.key===key)age=Math.max(age,autoObservationClock.age+(now-autoObservationClock.at)/1000);
  autoObservationClock={key,age,at:now};
 }
 const live=o.live===true&&S?.mapping===true&&S?.online!==false&&Number.isFinite(o.stamp)&&age!==null&&age<=2;
 const message=live?(o.message||'实时位置'):
  o.live===true?'观测已过期，显示最后观测位置':o.message||'采集已停止，显示最后观测（非当前位置）';
 return {state:live?'live':'history',live,drawPose:true,age,message,stateAge};
}
function autoPaintMapObservation(m,a=S?.autonomy||{}){
 const status=autoMapObservation(m,a);text('auto-observation-status',status.message+(status.live&&status.age!==null?' · '+fmt(status.age,1)+' 秒前':''));
 const svg=$('#auto-view'),marker=svg?.querySelector('#auto-robot-marker');
 if(marker){marker.setAttribute('data-observation-state',status.state);marker.querySelector('circle')?.setAttribute('fill',status.live?'#217c62':status.state==='unverified'?'#b08a3c':'#82918b');const title=marker.querySelector('title');if(title)title.textContent=status.live?'当前位置':'最后观测位置，非当前位置';}
 svg?.querySelector('#auto-planned-path')?.setAttribute('display',status.live&&a.active?'inline':'none');
 const nav=$('#auto-navigation-status');if(nav){
  nav.hidden=S?.mapping===true;
  const view=S?.pose_view,p=view?.display_pose,h=view?.health;
  const ready=S?.online!==false&&h?.display_usable===true&&Number.isFinite(h.pose_age_s)&&h.pose_age_s+status.stateAge<=2&&p&&[p.PosX,p.PosY,p.Yaw].every(Number.isFinite);
  nav.textContent=ready?`当前导航位置：X ${fmt(p.PosX,2)} m，Y ${fmt(p.PosY,2)} m，朝向 ${fmt(p.Yaw*180/Math.PI,1)}°`:'导航定位未就绪';
 }
 return status;
}
function autoDrawMap(m,a=S?.autonomy||{}){
 const svg=$('#auto-view');if(!svg)return;
 const status=autoPaintMapObservation(m,a);
 if(!m?.floor?.length||!Number.isFinite(m.resolution)||m.resolution<=0){svg.innerHTML='';$('#auto-empty').hidden=false;return;}
 $('#auto-empty').hidden=true;
 const pose=status.drawPose&&Array.isArray(m.pose)&&m.pose.length>=3&&m.pose.slice(0,3).every(Number.isFinite)?m.pose:null;
 const points=m.floor.concat(m.obstacles||[]);if(pose)points.push([pose[0]/m.resolution,pose[1]/m.resolution]);
 const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
 const minX=Math.min(...xs)-3,maxX=Math.max(...xs)+3,minY=Math.min(...ys)-3,maxY=Math.max(...ys)+3;
 const scale=Math.min(640/(maxX-minX),420/(maxY-minY));
 const px=x=>20+(x-minX)*scale,py=y=>440-(y-minY)*scale;
 const cells=(list,color)=>list.map(p=>`<rect x="${px(p[0])-scale/2}" y="${py(p[1])-scale/2}" width="${scale+.3}" height="${scale+.3}" fill="${color}"/>`).join('');
 let body=cells(m.floor,'#d9e6e1')+cells(m.obstacles,'#6b7c77');
 body+=`<polyline id="auto-planned-path" display="${status.live&&a.active?'inline':'none'}" points="${(m.path||[]).map(p=>`${px(p[0])},${py(p[1])}`).join(' ')}" fill="none" stroke="#248869" stroke-width="3" stroke-linecap="round"/>`;
 if(pose){const [x,y,t]=pose;body+=`<g id="auto-robot-marker" transform="translate(${px(x/m.resolution)},${py(y/m.resolution)}) rotate(${-t*180/Math.PI})"><title>${status.live?'当前位置':'最后观测位置，非当前位置'}</title><circle r="12" fill="${status.live?'#217c62':'#82918b'}" stroke="white" stroke-width="3"/><path d="M0 -5L16 0 0 5Z" fill="white"/></g>`;}
 svg.innerHTML=body;autoPaintMapObservation(m,a);
}
function autoDrawVision(v){
 const svg=$('#auto-view');if(!svg)return;
 $('#auto-empty').hidden=!!v.enabled;
 if(!v.enabled){svg.innerHTML='';svg.dataset.visionCount='';html('auto-people','');return;}
 const detections=v.detections||[],key=String(detections.length)+String(!!v.demo);
 if(svg.dataset.visionCount!==key){
  svg.dataset.visionCount=key;
  svg.innerHTML=`<rect width="680" height="460" fill="#e1e9e4"/><image class="auto-image" width="680" height="460" preserveAspectRatio="none"/>${v.demo?'<path d="M0 400L300 200 680 410" fill="none" stroke="#b9c8bf" stroke-width="2"/><text x="20" y="32" fill="#537063" font-size="16">离线模拟人物 · 非真实相机</text>':''}`+detections.map((d,i)=>`<g data-auto-person="${i}" tabindex="0" role="button" aria-label="跟随人物 ${i+1}" class="auto-person">${v.demo?'<g class="auto-avatar"><ellipse cx="0" cy="0" rx="17" ry="20" fill="#82958b"/><path d="M0 20v70m0 -60l-25 40m25 -40l25 40m-25 20l-20 60m20 -60l20 60" stroke="#82958b" stroke-width="12" fill="none"/></g>':''}<rect class="auto-box" fill="transparent" stroke-width="3"/><rect class="auto-label-bg" width="115" height="24" rx="3" fill="#173b31"/><text class="auto-label" fill="white" font-size="13"></text></g>`).join('');
  html('auto-people',detections.map((d,i)=>`<button class="btn" data-auto-person="${i}">跟随人物 ${i+1}</button>`).join('')||'<p class="help">暂未检测到完整人物</p>');
 }
 if(v.image)svg.querySelector('.auto-image').setAttribute('href',v.image);
 const set=(el,attrs)=>Object.entries(attrs).forEach(([k,val])=>el.setAttribute(k,String(val)));
 detections.forEach((d,i)=>{const [x1,y1,x2,y2]=d.box,t=S.autonomy?.target;
  const selected=S.autonomy?.active&&S.autonomy.mode==='follow'&&t&&Math.abs((t[0]+t[2]-x1-x2)/2)<.08&&Math.abs(t[1]-y1)<.1;
  const group=svg.querySelector(`[data-auto-person="${i}"]`);group.dataset.sequence=v.sequence;
  set(group.querySelector('.auto-box'),{x:x1*680,y:y1*460,width:(x2-x1)*680,height:(y2-y1)*460,stroke:selected?'#2fcd8a':'#eeac4a'});
  set(group.querySelector('.auto-label-bg'),{x:x1*680,y:y1*460-25});
  const label=group.querySelector('.auto-label');set(label,{x:x1*680+7,y:y1*460-8});label.textContent=selected?'已锁定':'点选人物 '+(i+1);
  if(v.demo)group.querySelector('.auto-avatar').setAttribute('transform',`translate(${(x1+x2)*340} ${y1*460+20})`);
  const button=$(`#auto-people [data-auto-person="${i}"]`);button.dataset.sequence=v.sequence;button.disabled=!!S.autonomy?.active;
 });
 text('auto-vision-error',v.error||(!v.fresh&&v.enabled?'等待最新画面…':''));
}
const autoOriginalUpdate=update;
update=function(){autoOriginalUpdate();if(page!=='autonomy'||!S?.autonomy)return;const a=S.autonomy;
 if(autoStartIntent&&!autoStartCurrent(autoStartIntent))invalidateAutoStart('连接或设置已变化，已取消本次自动开始');
 text('auto-state',autoStartIntent?(autoStartIntent.phase==='localizing'?'恢复定位中':'准备中'):({idle:'待开始',preparing:'准备中',exploring:'正在探索',following:'正在跟随',paused:'已暂停',saving:'保存中',discarding:'放弃中',discarded:'已放弃',completed:'已完成',failed:'需处理'})[a.state]||a.state);
 text('auto-message',autoTab==='explore'&&(autoStartIntent?.message||autoStartNotice)||a.message);text('auto-area',autoTab==='explore'?fmt(a.map?.area):fmt(a.vision?.age,2));text('auto-distance',autoTab==='explore'?fmt(a.map?.distance):a.vision?.detections?.length||0);text('auto-time',a.elapsed||0);
 if(autoTab==='explore'&&a.active&&a.map?.frontier_metrics?.path_length_m!=null&&!autoStartIntent){
  const f=a.map.frontier_metrics;const value=a.map.target_kind==='expand'?'扩展未扫描区域':a.map.target_kind==='refine'?'补充旧区域观察角度':'探索可见边界';
  text('auto-message',`${value} · 目标路程 ${f.path_length_m} 米 · 预计新增可见区域 ${f.information_gain_m2||0} ㎡ · ${a.message}`);
 }
 paintAutoMapOptions();
 if(autoTab==='explore')autoDrawMap(a.map);
};
const autoPreviousRender=render;
render=function(...args){if(page!=='autonomy')invalidateAutoStart('已离开自主作业，已取消本次自动开始');return autoPreviousRender.apply(this,args);};
ui['auto-explore-tab']=()=>{autoTab='explore';render();};ui['auto-follow-tab']=()=>{invalidateAutoStart();autoTab='follow';render();};
ui['auto-start']=async()=>{
 if(autoStartIntent)return;
 paintAutoMapOptions();
 const kind=$('#auto-map-kind').value;
 if(kind==='continue'&&!S.mapping)throw Error('当前没有正在采集的地图，请选择新建或更新');
 const req={mode:'explore',use_current:kind==='continue',scope:kind==='extend'?$('#auto-scope').value:'radius',radius:+$('#auto-radius').value,minutes:+$('#auto-minutes').value,forward_limit:+$('#auto-forward').value/100,yaw_limit:+$('#auto-yaw').value/100};
 const intent={generation:++autoStartGeneration,mode:S.mode,ip:S.ip,form:autoFormKey(),phase:'checking',message:'正在检查当前地图'};
 autoStartIntent=intent;autoStartNotice='';paintAutoMapOptions();showAutoStart(intent,intent.message);
 try{
  if(kind==='extend'){
   const library=await api('/api/maps/library',undefined,true);if(!autoStartCurrent(intent))return;
   if(!library.active)throw Error('请先启用要更新的地图');req.extend=true;req.base=library.active;
   if(S.mode==='live'&&S.pose?.Location!==0){
    showAutoStart(intent,'正在自动恢复定位…','localizing');
    const start=await api('/api/maps/localize-auto',{},true);if(!autoStartCurrent(intent))return;
    const result=await waitAutoLocalization(start,intent);if(!result||!autoStartCurrent(intent))return;
    await fetchState();if(!autoStartCurrent(intent))return;
   }
  }
  if(!autoStartCurrent(intent))return;
  showAutoStart(intent,'正在开始自主探索…','starting');
  await api('/api/autonomy/start',req,true);if(!autoStartCurrent(intent))return;
  autoStartNotice='';await fetchState();
 }catch(error){
  if(!autoStartCurrent(intent))return;
  autoStartNotice=(intent.phase==='starting'?'开始请求未获确认，请查看任务状态或按暂停：':'未开始自主移动：')+error.message;
  text('auto-message',autoStartNotice);throw error;
 }finally{if(autoStartIntent===intent){autoStartIntent=null;paintAutoMapOptions();}}
};
ui['auto-relocalize']=async()=>{invalidateAutoStart();await api('/api/maps/localize-auto',{});toast('正在自动恢复定位，请等待任务结果');await fetchState();};
ui['auto-stop']=async()=>{invalidateAutoStart();cancelAutoResumeRequest();await api('/api/autonomy/stop',{});await fetchState();};
async function autoLifecycleAction(kind,path,data={}){
 if(autoLifecyclePending)return;
 const token={kind,mode:S.mode,ip:S.ip,capture_id:S.autonomy?.capture_id,controller:new AbortController(),cancelled:false};autoLifecyclePending=token;invalidateAutoStart();paintAutoMapOptions();
 const timer=setTimeout(()=>token.controller.abort(),12000);
 try{
  if(kind==='resume'){
   await fetchState();if(token.cancelled||S.mode!==token.mode||S.ip!==token.ip||S.autonomy?.capture_id!==token.capture_id)return;
   data={...data,control_generation:S.autonomy?.control_generation,server_instance:S.server_instance};
  }
  await api(path,data,true,{signal:token.controller.signal});if(!token.cancelled)await fetchState();
 }catch(error){if(!token.cancelled)throw error;}
 finally{clearTimeout(timer);if(autoLifecyclePending===token)autoLifecyclePending=null;paintAutoMapOptions();}
}
ui['auto-save']=()=>autoLifecycleAction('save','/api/autonomy/save',{capture_id:S.autonomy?.capture_id});
ui['auto-resume']=()=>autoLifecycleAction('resume','/api/autonomy/resume',{capture_id:S.autonomy?.capture_id,minutes:+($('#auto-minutes')?.value||5)});
ui['auto-discard']=()=>{if(!S.autonomy?.discard_supported||!S.autonomy?.can_discard)throw Error('当前服务尚未支持放弃本轮');return autoLifecycleAction('discard','/api/autonomy/discard',{capture_id:S.autonomy.capture_id});};
ui['auto-manual']=async()=>{invalidateAutoStart();cancelAutoResumeRequest();await api('/api/autonomy/stop',{});navigate('motion');};
ui['auto-vision-start']=async()=>{await api('/api/autonomy/vision/start',{model_path:$('#auto-model')?.value||''});await autoFetchVision();};
ui['auto-vision-stop']=async()=>{await api('/api/autonomy/vision/stop',{});await autoFetchVision();};
async function autoFetchVision(){if(page!=='autonomy'||autoTab!=='follow'||autoVisionBusy)return;autoVisionBusy=true;try{autoVision=await api('/api/autonomy/vision',undefined,true);autoDrawVision(autoVision);}catch(e){text('auto-message',e.message);}finally{autoVisionBusy=false;}}
async function autoChoose(target){if(!target||S.autonomy?.active)return;try{await api('/api/autonomy/start',{mode:'follow',person:+target.dataset.autoPerson,sequence:+target.dataset.sequence,minutes:20});await fetchState();}catch(e){/* api displays the specific reason */}}
document.addEventListener('click',e=>autoChoose(e.target.closest('[data-auto-person]')));
document.addEventListener('change',e=>{if(autoFormIds.includes(e.target.id)){invalidateAutoStart('设置已变化，已取消自动开始；定位搜索可能仍在进行');paintAutoMapOptions();}});
document.addEventListener('input',e=>{if(autoFormIds.includes(e.target.id))invalidateAutoStart('设置已变化，已取消本次自动开始');});
document.addEventListener('visibilitychange',()=>{if(document.hidden){
 if(autoStartIntent?.phase!=='starting')invalidateAutoStart('启动准备已暂停，返回后可重新开始');
 cancelAutoResumeRequest();
}});
document.addEventListener('click',e=>{if(e.target.closest('[data-action="stop"],[data-action="estop"]')){invalidateAutoStart();cancelAutoResumeRequest();}},true);
document.addEventListener('keydown',e=>{if(e.code==='Space'&&!e.target.matches('input,textarea,select')){invalidateAutoStart();cancelAutoResumeRequest();}});
window.addEventListener?.('hashchange',()=>{if(location.hash!=='#autonomy'){invalidateAutoStart('已离开自主作业，已取消本次自动开始');cancelAutoResumeRequest();}});
document.addEventListener('keydown',e=>{if(['Enter',' '].includes(e.key)&&e.target.matches('.auto-person')){e.preventDefault();autoChoose(e.target);}});
setInterval(()=>{if(!document.hidden){autoFetchVision();if(page==='autonomy'&&autoTab==='explore')autoPaintMapObservation(S?.autonomy?.map,S?.autonomy);}},400);
// Entry points stay next to the work the operator is already doing.
for(const name of ['motion','mapsPage','navPage']){
 const old=window[name];window[name]=function(){return old()+`<div class="auto-entry card"><div><strong>自主建图与人员跟随</strong><p class="help">自动探索、更新地图或跟随人员。</p></div><button class="btn" data-page="autonomy">打开自主作业 ${icon('arrow')}</button></div>`;};
}

if(location.hash==='#autonomy'){page='autonomy';if(S){renderNav();render();}}
