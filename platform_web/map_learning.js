'use strict';
function learningPanel(){
 return `<section class="card learning-panel"><div class="learning-title"><div><strong>行走中优化地图</strong><p>行走时采集地图更新，保存为新版本。</p></div><label class="learning-switch"><input type="checkbox" role="switch" id="map-learning-toggle" aria-label="行走中优化地图"><span></span></label></div><div id="map-learning-message" role="status">等待状态</div><div class="learning-metrics"><span id="map-learning-count">0 帧</span><span id="map-learning-base">活动地图</span>${btn('保存地图更新','map-learning-generate','small')}</div><div id="map-learning-revisions"></div><small>新版本保留旧区域；点击启用后用于导航。</small></section>`;
}
// The optional map-refinement switch lives below the driving workspace, not above
// the camera or the navigation map. It remains available without crowding the
// primary task.
{
 const oldMotion=motion;motion=function(){const markup=oldMotion(),container=document.createElement('div');container.innerHTML=markup;
  const target=container.querySelector('#drive-advanced');if(target)target.insertAdjacentHTML('afterbegin',learningPanel());return container.innerHTML;};
}
function paintLearning(){
 const r=S?.map_learning;if(!r||!$('#map-learning-toggle'))return;
 $('#map-learning-toggle').checked=!!r.enabled;
 text('map-learning-message',(r.demo?'离线演示 · ':'')+(r.message||'尚未开启'));
 text('map-learning-count',(r.total_frames??r.frames??0)+' 帧累计 · 本轮 '+(r.frames||0)+' 帧');text('map-learning-base',r.base||'当前活动地图');
 const button=$('[data-ui="map-learning-generate"]');if(button)button.disabled=!r.frames||r.state==='generating';
 html('map-learning-revisions',(r.revisions||[]).slice().reverse().slice(0,3).map(v=>`<div class="learning-version"><div><strong>${esc(v.name)}</strong><small>${v.report.frames} 帧 · 稳定点 ${v.report.stable_points} · 栅格更新 ${v.report.occupied_added+v.report.free_updated}</small></div><button class="btn small" data-learning-apply="${esc(v.name)}" ${S.maps.some(m=>m.verified_for_robot&&m.robot_map_name===v.name)?'disabled':''}>${S.maps.some(m=>m.verified_for_robot&&m.robot_map_name===v.name)?'当前使用':'启用此版本'}</button></div>`).join(''));
}
const learningUpdate=update;update=function(){learningUpdate();paintLearning();paintGoalPlan();};
document.addEventListener('change',async e=>{if(e.target.id!=='map-learning-toggle')return;const el=e.target;el.disabled=true;try{await api('/api/maps/refinement',{action:el.checked?'enable':'disable'});await fetchState();}catch(err){el.checked=!!S.map_learning?.enabled;}finally{el.disabled=false;}});
ui['map-learning-generate']=async()=>{await api('/api/maps/refinement',{action:'generate'});await fetchState();};
document.addEventListener('click',async e=>{const b=e.target.closest('[data-learning-apply]');if(!b)return;try{
 if(S.nav.status==='running'||S.nav.status==='cancelling'||S.velocity.some(v=>v!==0))throw Error('先结束导航并停稳，再启用地图更新');
 const scope=navigationMapScope(),name=b.dataset.learningApply,grid=S.map_learning.grid_id||0;await loadMapLibrary();if(scope!==navigationMapScope())return;openMapActivation(name,grid);
 }catch(err){toast(err.message,true);}});

let goalPlan=null,goalPlanKey='',goalPlanBusy=false,goalPlanError='',goalPlanGeneration=0,goalSubmitting=false;
let goalPlanDiagnostics=null;
let goalPlanRequest=null,goalReplanning=false;
function goalNavigationEngine(){return S.mode!=='demo'&&$('#route-navigation-engine')?.value==='gos'?'gos':'native';}
function goalKey(){if(page!=='navigation'||navIntent!=='goal'||points.length!==1)return '';return [mapId,points[0].x,points[0].y,$('#goal-yaw-mode')?.value,$('#goal-yaw')?.value,goalNavigationEngine()].join('|');}
function validGoalPlan(){return !!(goalKey()&&goalPlan&&goalKey()===goalPlanKey&&performance.now()-goalPlan.received<55000);}
function invalidateGoalPlan(){
 goalPlanGeneration++;goalPlan=null;goalPlanKey='';goalPlanError='';goalPlanDiagnostics=null;$('#goal-plan-preview')?.remove();$('#goal-recovery-preview')?.remove();
 const request=goalPlanRequest;goalPlanRequest=null;goalPlanBusy=false;
 if(request){clearTimeout(request.timer);request.controller.abort();}
}
const plannedRenderPoints=renderPoints;renderPoints=function(){invalidateGoalPlan();plannedRenderPoints();paintGoalPlan();};
const plannedRender=render;render=function(){if(page!=='navigation'||navIntent!=='goal')invalidateGoalPlan();plannedRender();};
async function refreshGoalPlan(){
 const key=goalKey();if(!key||goalPlanBusy||goalSubmitting)return;goalPlanBusy=true;goalPlanError='';goalPlanDiagnostics=null;goalPlan=null;goalPlanKey=key;const generation=++goalPlanGeneration;paintGoalPlan();drawMap();
 const request={controller:new AbortController(),timedOut:false};goalPlanRequest=request;
 request.timer=setTimeout(()=>{request.timedOut=true;request.controller.abort();},15000);
 try{const manual=$('#goal-yaw-mode')?.value==='manual',degrees=+($('#goal-yaw')?.value||0);const yaw=manual?(((degrees+180)%360+360)%360-180)*Math.PI/180:null;
  const r=await api('/api/nav/plan',{map_id:mapId,x:points[0].x,y:points[0].y,yaw,engine:goalNavigationEngine()},true,{signal:request.controller.signal});if(generation===goalPlanGeneration&&goalKey()===key)goalPlan={...r,received:performance.now()};
 }catch(err){if(generation===goalPlanGeneration&&goalKey()===key){goalPlanError=request.timedOut?'规划请求超时，请检查平台连接后点击“重新规划”':err.message;goalPlanDiagnostics=err.navigation||null;}}
 finally{clearTimeout(request.timer);if(goalPlanRequest===request){goalPlanRequest=null;goalPlanBusy=false;paintGoalPlan();drawMap();}}
}
function goalDiagnosticPresentation(d=goalPlanDiagnostics){
 if(!d)return null;
 const code=d.reason_code||d.classification||'',overlap=d.overlap_diagnostics||{},parts=[];
 const actions=['relocalize_and_replan','sync_active_map','revalidate_observation'];
 let action=actions.includes(d.recommended_action)?d.recommended_action:null;
 if(/map_identity|active_map_mismatch|map_changed/.test(code)||d.classification==='map_identity_mismatch')action='sync_active_map';
 else if(!action&&['live_evidence_expired','insufficient_ground_votes'].includes(code))action='revalidate_observation';
 else if(!action&&d.classification==='map_surface_disagreement')action='relocalize_and_replan';
 const count=(key,label)=>{if(Number.isFinite(overlap[key]))parts.push(label+' '+overlap[key]+' 格');};
 count('overlap_cells','地图重叠');count('unobserved_body_cells','机身盲区尚无观测');count('insufficient_vote_cells','独立观测不足');count('live_blocked_cells','实测障碍');
 const surface={expired:'临时通行观测已过期',waiting_evidence:'正在收集独立观测',projection_unconfirmed:'扫描与地图结构尚未核对'}[overlap.surface_state];if(surface)parts.push(surface);
 const title=action==='sync_active_map'?'当前导航图与机器人活动图不一致':d.classification==='map_surface_disagreement'?'当前位置与地图占据存在冲突':d.classification==='live_body_collision'?'实测障碍与机身范围重叠':'本次路线尚未通过核验';
 return {title,summary:parts.join('；')||d.reason||goalPlanError,action};
}
function paintGoalDiagnostics(){
 const status=$('#goal-plan-status'),view=goalDiagnosticPresentation();let panel=$('#goal-plan-diagnostics');
 if(!status||!view){if(panel)panel.hidden=true;return;}
 if(!panel){panel=document.createElement('div');panel.id='goal-plan-diagnostics';panel.className='help';panel.setAttribute('role','status');status.parentNode.insertBefore(panel,status.nextSibling);}
 panel.hidden=false;const key=JSON.stringify(view);
 if(panel._diagnosticKey!==key){
  panel._diagnosticKey=key;panel.replaceChildren();panel._actionButton=null;
  const title=document.createElement('strong');title.textContent=view.title;panel.append(title);
  const detail=document.createElement('p');detail.textContent=view.summary;panel.append(detail);
  const action={sync_active_map:['同步机器人当前活动地图','goal-sync-active-map'],revalidate_observation:['重新读取观测并检查路线','goal-observation-recheck'],relocalize_and_replan:['重新识别位置并重新规划','goal-localize-preview']}[view.action];
  if(action){const button=document.createElement('button');button.type='button';button.className='btn small';button.dataset.ui=action[1];button.textContent=action[0];panel.append(button);panel._actionButton=button;}
  const note=document.createElement('p');note.textContent=view.action==='sync_active_map'?'先同步活动地图，再重新选点；不会切换机器人地图。':'上述检查只更新定位或路线预览，不开始行走，也不修改旧地图。';panel.append(note);
 }
 const button=panel._actionButton;
 if(button){button.disabled=goalPlanBusy||goalSubmitting||goalReplanning||['running','cancelling'].includes(S.nav?.status);button.title='';if(view.action==='relocalize_and_replan'&&S.mode==='live'&&S.navigation_preparation?.verify_current_localization!==true){button.disabled=true;button.title='请加载支持静态结构识别的新版平台后刷新';}}
}
function paintGoalPlan(){
 if(page!=='navigation'||navIntent!=='goal')return;const key=goalKey();
 const valid=validGoalPlan(),ready=navigationMapReady(),active=['running','cancelling'].includes(S.nav.status);
 const reason=!ready?'请先同步当前活动地图':goalSubmitting?'正在提交导航，请稍候':goalReplanning?'正在重新规划，请稍候':active?'当前导航尚未结束':!key?'请先选定目标':goalPlanBusy?'正在规划，请稍候':!valid?goalPlanError||'请等待有效路线或重新规划':'';
 const b=$('[data-ui="go-goal"]');if(b){const disabled=!!reason;if(b.disabled!==disabled)b.disabled=disabled;b.title=reason||'前往所选目标';}
 const replan=$('[data-ui="goal-replan"]');if(replan){replan.disabled=!key||goalReplanning||goalSubmitting;replan.title=!key?'请先选择目标':goalReplanning?'正在重新规划':'重新计算当前目标的路线';}
 const description=goalPlan?.plan_on_start?'目标已选定':goalPlan?.needs_local_plan?'GOS 将用机内雷达规划通路':valid?goalPlan.length_m+' 米 · '+Math.max(0,goalPlan.points.length-1)+' 个参考折点 · 到达朝向 '+(goalPlan.goal.yaw*180/Math.PI).toFixed(0)+'°':'等待规划';
 const message=!key?'在地图上选定目标':goalPlanBusy?'正在准备目标…':goalPlanError||description;
 const recovery=valid&&goalPlan.recovery?.required?` · 起点先平移退让 ${Math.round(goalPlan.recovery.recovery_distance_m*100)} 厘米再执行导航`:'';
 text('goal-plan-status',message+recovery+(valid&&S.mode==='live'?' · '+goalPlan.obstacle_note:''));
 paintGoalDiagnostics();
 const feedback=$('#nav-feedback');if(feedback&&S.nav.warning&&!feedback.querySelector('.nav-progress-warning')){const note=document.createElement('p');note.className='help nav-progress-warning';note.textContent=S.nav.warning;feedback.append(note);}
}
ui['goal-replan']=async()=>{
 if(goalReplanning||goalSubmitting)return;
 const key=goalKey();if(!key)throw Error('请先选择目标，再重新规划');
 goalReplanning=true;invalidateGoalPlan();paintGoalPlan();
 try{
  if(['running','cancelling'].includes(S.nav.status)){
   await api('/api/nav/cancel',{});for(let i=0;i<30;i++){await fetchState();if(S.nav.status!=='cancelling')break;await new Promise(r=>setTimeout(r,200));}
   if(S.nav.status==='cancelling')throw Error('等待机器人确认停止后再规划');
  }
  if(goalKey()===key)await refreshGoalPlan();
 }finally{goalReplanning=false;paintGoalPlan();}
};
ui['go-goal']=async()=>{
 if(goalSubmitting||goalReplanning)return;
 if(!validGoalPlan()||goalPlanBusy)throw Error('先规划一条可通行路径');
 if(!navigationMapReady())throw Error('请先同步当前活动地图');
 if(['running','cancelling'].includes(S.nav.status))throw Error('请先结束当前导航');
 goalSubmitting=true;paintGoalPlan();
 try{const r=await api('/api/nav/goal',{map_id:mapId,plan_id:goalPlan.plan_id,engine:goalNavigationEngine(),disconnect_policy:'stop'});await fetchState();toast(r.message);}finally{goalSubmitting=false;paintGoalPlan();}
};
const plannedDraw=drawMap;drawMap=function(){plannedDraw();$('#goal-plan-preview')?.remove();$('#goal-recovery-preview')?.remove();if(page!=='navigation'||navIntent!=='goal')return;const m=selectedMap(),svg=$('#nav-map');if(!m||!svg)return;
 const p=(S?.nav?.engine==='platform'||S?.nav?.executor==='gos')&&S.nav.status==='running'?S.nav.plan:goalKey()&&goalKey()===goalPlanKey?goalPlan:null;if(!p?.path?.length)return;
 const line=document.createElementNS('http://www.w3.org/2000/svg','polyline');line.id='goal-plan-preview';line.setAttribute('points',p.path.map(v=>[(v.x-m.origin[0])/m.resolution,m.height-(v.y-m.origin[1])/m.resolution].join(',')).join(' '));line.setAttribute('fill','none');line.setAttribute('stroke','#2d9675');line.setAttribute('stroke-width','3');line.setAttribute('stroke-linejoin','round');line.setAttribute('pointer-events','none');svg.append(line);
 if(p.recovery?.required&&p.recovery.prefix?.length){const prefix=document.createElementNS('http://www.w3.org/2000/svg','polyline');prefix.id='goal-recovery-preview';prefix.setAttribute('points',p.recovery.prefix.map(v=>[(v.x-m.origin[0])/m.resolution,m.height-(v.y-m.origin[1])/m.resolution].join(',')).join(' '));prefix.setAttribute('fill','none');prefix.setAttribute('stroke','#c78b26');prefix.setAttribute('stroke-width','4');prefix.setAttribute('stroke-dasharray','5 3');prefix.setAttribute('pointer-events','none');svg.append(prefix);}
};
document.addEventListener('change',e=>{if(['goal-yaw','goal-yaw-mode','route-navigation-engine'].includes(e.target.id)){if($('#goal-yaw-field'))$('#goal-yaw-field').hidden=$('#goal-yaw-mode').value!=='manual';invalidateGoalPlan();paintGoalPlan();drawMap();}});
setInterval(()=>{if(document.hidden||goalReplanning)return;const key=goalKey();if(key&&!goalPlanBusy&&(key!==goalPlanKey||goalPlan&&performance.now()-goalPlan.received>55000)&&!['running','cancelling'].includes(S?.nav?.status))refreshGoalPlan();},300);
