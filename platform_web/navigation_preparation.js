'use strict';
// Explicit start actions only. Map preview and status polling never initialize
// localization or start motion. The server still validates every final request.
(()=>{
 let pending=null,generation=0,notice='';
 const supported=s=>s?.navigation_preparation?.client_intent_protocol===1;
 const cancelled=()=>Object.assign(new Error('导航准备已取消'),{navigationPreparationCancelled:true});
 const connection=s=>JSON.stringify([s?.mode,s?.ip,s?.server_instance,s?.connection_generation]);
 const routeKey=id=>{const r=S.routes?.find(r=>r.id===id);return r?JSON.stringify([r.id,r.map_id,r.points,r.options]):'';};
 const currentOptions=id=>{const r=S.routes?.find(r=>r.id===id),options=$('#nav-mode')&&r?.id===routeId?routeOptions():r?.options||{nav_mode:1,speed:1,gait:'flat',manner:0,pause_at_point:false};return {...options,engine:r?.points?.some(p=>Number(p.type)===3)?'native':options.engine||'platform'};};
 const routeSignature=id=>{const r=S.routes?.find(r=>r.id===id);return r?JSON.stringify([r.id,r.points,r.options]):'';};
 const selection=()=>JSON.stringify([mapId,routeId,typeof mapLibrary==='undefined'?null:mapLibrary.active]);
 const draftKey=()=>JSON.stringify([mapId,points,routeOptions(),$('#route-name')?.value||'新巡检路线']);
 const backgroundContext=t=>t.startDispatched&&(t.background||document.hidden||page!==t.page||location.hash!==t.hash);
 function current(t){
  return pending===t&&t.generation===generation&&connection(S)===t.connection&&S.online!==false&&
   (backgroundContext(t)||page===t.page&&location.hash===t.hash&&!document.hidden&&selection()===t.selection&&
   (t.kind==='goal'?goalKey()===t.key:t.kind==='draft'?navIntent==='route'&&draftKey()===t.key:(t.mapAliases?t.mapAliases.includes(S.routes?.find(r=>r.id===t.id)?.map_id)&&routeSignature(t.id)===t.routeSignature:routeKey(t.id)===t.key)&&JSON.stringify(currentOptions(t.id))===t.optionsKey));
 }
 function guard(t){if(!current(t))throw cancelled();}
 async function revokeStart(t,cancelGeneration){
  // Keepalive survives leaving the page; the server records this ID even if
  // cancellation arrives before the original start POST. Never send a global
  // stop here: another browser may already own a different navigation task.
  let timer;
  try{
   const sent=fetch('/api/nav/intent/cancel',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(t.clientRequest),keepalive:true})
    .then(async response=>{const result=await response.json();if(!response.ok||result.ok===false)throw Error(result.err||'撤销未获确认');return result;});
   const result=await Promise.race([sent,new Promise((_,reject)=>{timer=setTimeout(()=>reject(Error('等待撤销确认超时')),8000);})]);
   if(generation===cancelGeneration&&!pending){notice=result.status==='cancelling'?'已请求停止本次导航，等待实际停稳反馈':'本次导航启动意图已撤销';paintGoalPlan();paint();}
  }catch(error){
   if(generation===cancelGeneration&&!pending){notice='本次导航撤销尚未获确认：'+error.message+'；请核对任务状态或按停止';toast(notice,true);paintGoalPlan();paint();}
  }finally{clearTimeout(timer);}
 }
 function cancel(message='已取消后续自动开始；已经提交的定位搜索可能仍在进行'){
  const old=pending;generation++;pending=null;
  if(old){old.readAbort?.abort();if(old.kind==='goal')goalSubmitting=false;notice=old.startDispatched?'正在撤销本次导航启动并核对停止状态…':message;paintGoalPlan();paint();if(old.startDispatched)return revokeStart(old,generation);}
  else notice='';
 }
 function status(t,message,phase=t.phase){guard(t);t.message=message;t.phase=phase;paint();}
 function canPrepareMap(){return typeof navigationMapCanPrepare==='function'?navigationMapCanPrepare():navigationMapReady();}
 function humanReason(d,fallback='暂时没有可通行路线'){
  if(d?.recommended_action==='sync_active_map')return d.selected_map===d.active_map?'正在更新导航地图':'目标不在机器人当前地图上，请显示当前地图后重新选点';
  if(d?.recommended_action==='relocalize_and_replan')return '暂时无法确定当前位置，请停稳后重试';
  if(d?.recommended_action==='revalidate_observation')return '等待通路恢复，请稍后再试';
  if(d?.classification==='live_body_collision')return '通路被障碍挡住，请等待障碍移开或选择其他目标';
  if(d?.classification==='unsupported_floor')return '前方路况暂不清楚，请停稳等待';
  return fallback;
 }
 const previousRouteOptions=routeOptions;
 routeOptions=function(){const options=previousRouteOptions();return {...options,engine:points.some(p=>Number(p.type)===3)?'native':$('#route-navigation-engine')?.value||S.routes?.find(r=>r.id===routeId)?.options?.engine||'platform'};};
 function paintRouteEngine(){
  if(page!=='navigation'||navIntent!=='route')return;
  let select=$('#route-navigation-engine');const anchor=$('#nav-mode');
  if(!select&&anchor){
   const field=anchor.closest?.('.field')||anchor.parentNode;
   const label=document.createElement('label');label.className='field';label.innerHTML='<span>寻路方式</span><select id="route-navigation-engine"><option value="platform">平台寻路 · 自动绕行</option><option value="native">原厂导航</option></select>';
   field.parentNode.insertBefore(label,field);select=$('#route-navigation-engine');
   if(select){select.value=S.routes?.find(r=>r.id===routeId)?.options?.engine||'platform';select.addEventListener('change',()=>{cancel('寻路方式已变化，请重新开始');paintRouteEngine();});}
  }
  if(select){
   const docking=points.some(p=>Number(p.type)===3);
   if(docking&&!select.dataset.chargeEngine){select.dataset.chargeEngine=select.value;select.value='native';}
   else if(!docking&&select.dataset.chargeEngine){select.value=select.dataset.chargeEngine;delete select.dataset.chargeEngine;}
   select.disabled=docking||!!pending||['running','cancelling'].includes(S.nav?.status);select.title=docking?'回充路线使用机器人内置导航':'';
   for(const id of ['nav-mode','nav-speed','nav-gait','nav-manner']){const field=$('#'+id)?.closest?.('.field');if(field)field.hidden=select.value==='platform';}
  }
 }
 function paint(){
  const busy=!!pending,goal=page==='navigation'&&navIntent==='goal',needs=S?.mode==='live'&&S.pose?.Location!==0;
  const syncing=typeof navSyncBusy!=='undefined'&&navSyncBusy;
  const diagnostic=typeof goalPlanDiagnostics==='undefined'?null:goalPlanDiagnostics;
  const mapMismatch=diagnostic?.recommended_action==='sync_active_map'&&diagnostic.active_map&&diagnostic.selected_map!==diagnostic.active_map;
  paintRouteEngine();
  const button=$('[data-ui="go-goal"]');
  if(button&&goal){
   if(!supported(S)){button.disabled=true;button.textContent='请先加载新版导航服务';button.title='后台尚不支持可撤销的导航准备，请加载新版平台后刷新';}
   else if(busy){button.disabled=true;button.textContent=pending.phase==='localizing'?'正在恢复定位…':'正在准备导航…';}
   else if(syncing){button.disabled=true;button.textContent='正在同步地图…';}
   else if(mapMismatch){button.disabled=true;button.textContent='请先同步活动地图';button.title='当前目标属于另一张地图，请同步机器人当前活动地图后重新选点';}
   else{
    button.textContent='执行导航';
    button.disabled=!goalKey()||!canPrepareMap()||goalReplanning||goalSubmitting||S.mapping||S.autonomy?.active||['running','cancelling'].includes(S.nav?.status);
    button.title=button.disabled?'请选择当前地图上的目标':'自动准备定位与路线并执行导航';
   }
  }
  for(const b of document.querySelectorAll('[data-route-run], [data-ui="start-nav"], [data-ui="goal-replan"], [data-ui="goal-localize-preview"], [data-ui="goal-observation-recheck"], [data-ui="goal-sync-active-map"]')){
   if(busy||syncing||!supported(S)){if(b.dataset.navigationWasDisabled===undefined)b.dataset.navigationWasDisabled=String(b.disabled);b.disabled=true;if(!supported(S))b.title='请加载新版平台后刷新，后台尚不支持可撤销的导航准备';}
   else if(b.dataset.navigationWasDisabled!==undefined){b.disabled=b.dataset.navigationWasDisabled==='true';delete b.dataset.navigationWasDisabled;}
  }
  if(busy){for(const name of ['pause-nav','cancel-nav']){const b=$(`[data-ui="${name}"]`);if(b)b.disabled=false;}}
  const message=pending?.message||notice;
  if(goal&&message)text('goal-plan-status',message);
  const host=$('#nav-feedback')||$('#task-current');
  if(host){let label=$('#navigation-preparation-status');if(!label&&message){label=document.createElement('p');label.id='navigation-preparation-status';label.className='help';label.setAttribute('role','status');host.prepend(label);}if(label){label.hidden=!message;label.textContent=message;}}
 }
 async function request(t,path,data,timeout=5000){
  guard(t);const controller=new AbortController();t.readAbort=controller;
  const timer=setTimeout(()=>controller.abort(),timeout);
  try{const result=await api(path,data,true,{signal:controller.signal});guard(t);return result;}
  catch(error){if(!current(t))throw cancelled();if(error.name==='AbortError')throw Error('导航准备请求超时；已提交的后台请求可能仍在进行，请查看任务状态');throw error;}
  finally{clearTimeout(timer);if(t.readAbort===controller)t.readAbort=null;}
 }
 async function refresh(t){
  // fetchState may return early during another poll or swallow a network error.
  // This explicit read must succeed before a prepared intent may progress.
  const fresh=await request(t,'/api/platform');
  if(connection(fresh)!==t.connection||fresh.online===false)throw cancelled();
  if(!supported(fresh))throw Error('后台版本已改变，请加载支持可撤销导航准备的版本');
  S=fresh;guard(t);update();guard(t);
  if(S.mapping)throw Error('请先结束建图采集，再开始导航');
  if(S.autonomy?.active||['running','cancelling'].includes(S.nav?.status))throw Error('已有任务运行中，请先停止');
 }
 async function localize(t){
  if(S.mode!=='live'||S.pose?.Location===0&&!t.forceLocalization)return;
  if(t.forceLocalization&&S.navigation_preparation?.verify_current_localization!==true)throw Error('后台尚不支持静态结构识别，请加载新版平台后刷新');
  status(t,'正在恢复当前位置','localizing');
  const start=await request(t,'/api/maps/localize-auto',t.forceLocalization?{verify_current:true}:{});
  if(start.localized!==true){
   if(!start.job_id)throw Error('定位请求没有返回可查询的任务编号');
   const deadline=performance.now()+120000;
   for(;;){
    guard(t);if(performance.now()>deadline)throw Error('等待定位超时；后台搜索可能仍在进行');
    const result=await request(t,'/api/operations?id='+encodeURIComponent(start.job_id));
    const job=result.jobs;if(!job)throw Error('定位任务状态不可用');
    status(t,'正在恢复当前位置','localizing');
    if(['failed','cancelled','canceled'].includes(job.status))throw Error(job.error||job.message||'自动定位未完成');
    if(job.status==='completed'){if(job.result?.localized!==true)throw Error('定位任务已结束，但尚未确认恢复');break;}
    await new Promise(resolve=>setTimeout(resolve,300));
   }
  }
  await refresh(t);if(S.mode==='live'&&S.pose?.Location!==0)throw Error('定位反馈仍未就绪，未开始导航');
 }
 function begin(kind,extra){
  if(pending)return null;
  if(!supported(S))throw Error('后台尚不支持可撤销的导航准备，请加载新版平台后刷新');
  if(document.hidden||S.online===false)throw Error('请保持页面可见并连接机器人后再开始');
  if(S.mapping||S.autonomy?.active||['running','cancelling'].includes(S.nav?.status))throw Error('请先结束当前建图或运动任务');
  const words=crypto.getRandomValues(new Uint32Array(4));
  const clientRequest={client_intent:'nav-'+Array.from(words,v=>v.toString(16).padStart(8,'0')).join(''),server_instance:S.server_instance,connection_generation:S.connection_generation};
  notice='';const t={kind,...extra,clientRequest,generation:++generation,page,hash:location.hash,connection:connection(S),selection:selection(),phase:'checking',message:'正在检查连接、地图和定位'};
  pending=t;if(kind==='goal'){goalSubmitting=true;invalidateGoalPlan();}paintGoalPlan();paint();return t;
 }
 async function finish(t,operation){
  try{await refresh(t);if(!t.skipLocalization)await localize(t);guard(t);return await operation();}
  catch(error){
   if(error.navigationPreparationCancelled||!current(t)){if(pending===t)cancel('连接或设置已变化，已取消后续自动开始');return;}
   if(error.navigation){error.originalMessage=error.message;error.message=humanReason(error.navigation,error.message);}
   if(t.kind==='goal'&&error.navigation){goalPlanError=error.message;goalPlanKey=t.key;goalPlanDiagnostics=error.navigation;}
   if(t.startDispatched&&backgroundContext(t)){notice='启动请求已提交，请在任务中心查看执行状态';return;}
   if(t.startDispatched){cancel('启动请求结果未确认，正在撤销本次意图');throw error;}
   notice=(t.previewOnly?'路线预览尚未恢复：':t.phase==='submitting'?'开始请求未获确认，请查看任务状态或按暂停：':'未开始导航：')+error.message;throw error;
  }finally{
   if(pending===t){pending=null;if(t.kind==='goal')goalSubmitting=false;paintGoalPlan();paint();}
  }
 }
 function goalRequest(){
  if(!goalKey())throw Error('请先选择目标');
  const manual=$('#goal-yaw-mode')?.value==='manual',degrees=+($('#goal-yaw')?.value||0);
  const goal={map_id:mapId,x:points[0].x,y:points[0].y,yaw:manual?(((degrees+180)%360+360)%360-180)*Math.PI/180:null,engine:pointNavigationEngine};
  if(![goal.x,goal.y].every(Number.isFinite)||goal.yaw!==null&&!Number.isFinite(goal.yaw))throw Error('目标坐标或朝向无效');
  return goal;
 }
 async function ensureNavigationMap(t,payload,force=false){
  if(S.mode==='demo')return;
  const selected=S.maps?.find(m=>m.id===payload.map_id);
  if(!force&&payload.map_id===mapId&&navigationMapReady())return;
  if(!force&&selected?.verified_for_robot&&selected.robot_ip===S.ip&&typeof mapLibrary!=='undefined'&&selected.robot_map_name===mapLibrary.active)return;
  const name=selected?.robot_map_name,grid=selected?.grid_id||0;
  const library=await request(t,'/api/maps/library');
  if(!name||selected.robot_ip!==S.ip||library.active!==name){
   throw Object.assign(Error('目标不在机器人当前地图上，请显示当前地图后重新选点'),{navigation:{classification:'map_identity_mismatch',reason_code:'active_map_mismatch',recommended_action:'sync_active_map',selected_map:name,active_map:library.active}});
  }
  const entry=library.maps?.find(m=>m.name===name);
  if(!entry?.grids?.includes(grid))throw Error('所选地图分区当前不可用，请重新选择地图');
  status(t,'正在更新导航地图','syncing');
  const synced=await request(t,'/api/maps/library/sync',{name,grid_id:grid},15000),map=synced.map;
  if(!map?.id||map.robot_map_name!==name||(map.grid_id||0)!==grid||map.verified_for_robot!==true)throw Error('地图仍在更新，请稍后再试');
  const previous=payload.map_id;payload.map_id=map.id;
  S.maps=S.maps.filter(m=>m.id!==map.id).concat([map]);
  if(typeof mapLibrary!=='undefined')mapLibrary=library;
  if(t.kind==='route')t.mapAliases=[...new Set([...(t.mapAliases||[previous]),map.id])];
  else{mapId=map.id;t.key=t.kind==='goal'?goalKey():draftKey();}
  t.selection=selection();guard(t);
  if(t.kind!=='route'){
   const svg=$('#nav-map'),image=svg?.querySelector?.('image');
   if(image&&map.image)image.setAttribute('href',map.image+(map.image.includes('?')?'&':'?')+'sync='+Date.now());
   if(svg&&Number.isFinite(map.width)&&Number.isFinite(map.height))svg.setAttribute('viewBox',`0 0 ${map.width} ${map.height}`);
  }
 }
 async function findRoute(t,goal){
  let synchronized=false,localized=false,observed=false;
  for(let attempt=0;attempt<4;attempt++){
   status(t,attempt?'正在重新绕行':'正在寻路','planning');
   try{return await request(t,'/api/nav/plan',goal,15000);}
   catch(error){
    if(!current(t))throw error;
    const action=error.navigation?.recommended_action;
    if(action==='sync_active_map'&&!synchronized){synchronized=true;await ensureNavigationMap(t,goal,true);continue;}
    if(action==='relocalize_and_replan'&&!localized&&S.navigation_preparation?.verify_current_localization===true){
     localized=true;t.forceLocalization=true;await localize(t);t.forceLocalization=false;continue;
    }
    if(action==='revalidate_observation'&&!observed){observed=true;status(t,'等待通路恢复，正在重新寻路','planning');await new Promise(resolve=>setTimeout(resolve,250));guard(t);continue;}
    throw error;
   }
  }
  throw Error('通路暂时不可用，请稍后再试');
 }
 async function updatePreview(t,goal){
  if(!navigationMapReady())throw Error('活动地图已改变，请先同步机器人当前活动地图');
  status(t,'正在读取同步观测并重新检查路线','planning');
  const plan=await request(t,'/api/nav/plan',goal,15000);
  if(!plan.plan_id)throw Error('规划未返回可用的路线预览');
  goalPlan={...plan,received:performance.now()};goalPlanKey=t.key;goalPlanError='';
  if(typeof goalPlanDiagnostics!=='undefined')goalPlanDiagnostics=null;
  paintGoalPlan();drawMap();return plan;
 }
 async function previewRecovery(forceLocalization){
  if(pending||goalSubmitting||goalReplanning)return;
  if(!navigationMapReady()||typeof goalDiagnosticPresentation==='function'&&goalDiagnosticPresentation()?.action==='sync_active_map')throw Error('请先同步机器人当前活动地图，再核对路线');
  if(forceLocalization&&S.mode==='live'&&S.navigation_preparation?.verify_current_localization!==true)throw Error('请加载支持静态结构识别的新版平台后刷新');
  const goal=goalRequest(),t=begin('goal',{key:goalKey(),previewOnly:true,forceLocalization,skipLocalization:!forceLocalization});if(!t)return;
  return finish(t,async()=>{
   const plan=await updatePreview(t,goal);
   notice=forceLocalization?'位置已重新识别，路线预览已更新；尚未开始行走':'已重新读取观测并更新路线预览；尚未开始行走';
   return plan;
  });
 }
 ui['goal-localize-preview']=()=>previewRecovery(true);
 ui['goal-observation-recheck']=()=>previewRecovery(false);
 ui['goal-sync-active-map']=async()=>{
  if(pending||goalSubmitting||goalReplanning)return;
  const t=begin('goal',{key:goalKey(),previewOnly:true,skipLocalization:true});if(!t)return;
  return finish(t,async()=>{
   status(t,'正在读取机器人当前活动地图','syncing');
   const library=await request(t,'/api/maps/library');if(!library.active)throw Error('机器人尚未报告活动地图');
   const currentMap=typeof selectedMap==='function'?selectedMap():null;
   const synced=await request(t,'/api/maps/library/sync',{name:library.active,grid_id:currentMap?.robot_map_name===library.active?currentMap.grid_id||0:0},15000);
   if(!synced.map?.id)throw Error('同步结果缺少地图编号');
   if(typeof mapLibrary!=='undefined')mapLibrary=library;
   mapId=synced.map.id;routeId=null;points=[];t.key=goalKey();t.selection=selection();invalidateGoalPlan();
   notice='已同步机器人当前活动地图，请重新选择目标；没有切换机器人地图或开始行走';
   await fetchState();guard(t);render();return synced;
  });
 };
 ui['go-goal']=async()=>{
  if(pending||goalSubmitting||goalReplanning)return;
  const diagnostic=typeof goalPlanDiagnostics==='undefined'?null:goalPlanDiagnostics;
  if(diagnostic?.active_map&&diagnostic.selected_map!==diagnostic.active_map)throw Error('请显示机器人当前地图，再重新选择目标');
  if(!goalKey()||!canPrepareMap())throw Error('请选择当前地图上的目标');
  const goal=goalRequest(),t=begin('goal',{key:goalKey(),skipLocalization:true});if(!t)return;
  return finish(t,async()=>{
   await ensureNavigationMap(t,goal);await localize(t);
   const plan=await findRoute(t,goal);
   if(!plan.plan_id)throw Error('规划未返回可执行路线');
   goalPlan={...plan,received:performance.now()};goalPlanKey=t.key;goalPlanError='';if(typeof goalPlanDiagnostics!=='undefined')goalPlanDiagnostics=null;paintGoalPlan();drawMap();
   status(t,'已重新规划，正在开始导航','submitting');guard(t);
   t.startDispatched=true;
   const result=await request(t,'/api/nav/goal',{...t.clientRequest,map_id:goal.map_id,plan_id:plan.plan_id,engine:goal.engine},20000);
   notice='';toast(result.message||'导航已提交');await fetchState();return result;
  });
 };
 runRoute=async function(id){
  if(pending)return;
  const key=routeKey(id);if(!key)throw Error('路线不存在，请重新选择');
  const route=S.routes.find(r=>r.id===id),options={...currentOptions(id)},t=begin('route',{id,key,optionsKey:JSON.stringify(options),skipLocalization:true,mapAliases:[route.map_id],routeSignature:routeSignature(id)});if(!t)return;
  return finish(t,async()=>{
   const map={map_id:route.map_id};await ensureNavigationMap(t,map);await localize(t);let plan=null;
   if(options.engine==='platform'){const first=route.points[0];plan=await findRoute(t,{map_id:map.map_id,x:first.x,y:first.y,yaw:route.points.length>1?null:first.yaw??null,engine:'platform'});if(!plan.plan_id)throw Error('暂时无法前往第一个航点');}
   status(t,'正在开始巡检','submitting');guard(t);
   t.startDispatched=true;
   const result=await request(t,'/api/nav/start',{...t.clientRequest,id,...options,...(plan?{plan_id:plan.plan_id}:{})},20000);
   notice='';toast(result.message||'路线已提交，等待导航反馈');await fetchState();return result;
  });
 };
 ui['start-nav']=async()=>{
  if(pending)return;
  if(page!=='navigation'||navIntent!=='route'||!points.length||!canPrepareMap())throw Error('请在当前地图设置巡检点位');
  const options={...routeOptions()},draft={id:routeId,map_id:mapId,name:$('#route-name')?.value||'新巡检路线',points:JSON.parse(JSON.stringify(points)),options};
  const t=begin('draft',{key:draftKey(),skipLocalization:true});if(!t)return;
  return finish(t,async()=>{
   await ensureNavigationMap(t,draft);await localize(t);let plan=null;
   if(options.engine==='platform'){const first=draft.points[0],goal={map_id:draft.map_id,x:first.x,y:first.y,yaw:draft.points.length>1?null:first.yaw??null,engine:'platform'};plan=await findRoute(t,goal);draft.map_id=goal.map_id;if(!plan.plan_id)throw Error('暂时无法前往第一个航点');}
   status(t,'正在保存巡检路线','saving');
   const saved=await request(t,'/api/route/save',draft);
   if(!saved.route?.id)throw Error('路线保存未返回有效编号');
   status(t,'正在开始巡检','submitting');guard(t);
   t.startDispatched=true;
   const result=await request(t,'/api/nav/start',{...t.clientRequest,id:saved.route.id,...options,...(plan?{plan_id:plan.plan_id}:{})},20000);
   routeId=saved.route.id;t.selection=selection();notice='';toast(result.message||'路线已提交，等待导航反馈');await fetchState();return result;
  });
 };
 const previousPaint=paintGoalPlan;paintGoalPlan=function(...args){previousPaint.apply(this,args);paint();};
 const previousUpdate=update;update=function(...args){if(pending&&!current(pending))cancel('连接、目标或页面已变化，已取消后续自动开始');previousUpdate.apply(this,args);paint();};
 const previousRender=render;render=function(...args){if(pending&&!current(pending))cancel('已离开本次导航准备，已取消后续自动开始');previousRender.apply(this,args);paint();};
 for(const key of ['pause-nav','cancel-nav','resume-nav','nav-takeover','motion-stop','clear-points','nav-goal-mode','nav-route-mode','goal-replan','hub-connect','hub-demo']){
  const previous=ui[key];if(previous)ui[key]=function(...args){cancel();return previous.apply(this,args);};
 }
 const changed=e=>{if(e.target.matches('#map-select,#goal-yaw,#goal-yaw-mode,#point-navigation-engine,#route-navigation-engine,#nav-mode,#nav-speed,#nav-gait,#nav-manner,#route-arrival,#route-name,[data-point]'))cancel('目标、地图或路线设置已变化，已取消后续自动开始');};
 document.addEventListener('input',changed,true);document.addEventListener('change',changed,true);
 document.addEventListener('click',e=>{if(e.target.closest('[data-action],[data-route-load],[data-route-delete],[data-connection-role],[data-service-role]'))cancel();},true);
 document.addEventListener('pointerdown',e=>{if(e.target.closest('#nav-map,[data-velocity]'))cancel('已手动操作，已取消后续自动开始');},true);
 document.addEventListener('keydown',e=>{if(['Space','KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE'].includes(e.code)&&!(e.target.closest?.('textarea,[contenteditable="true"]')||(e.target.tagName==='INPUT'&&['text','search','password','email','url','tel','number'].includes(e.target.type))))cancel();},true);
 document.addEventListener('visibilitychange',()=>{if(document.hidden){if(pending?.startDispatched)pending.background=true;else cancel('页面已进入后台，已取消尚未提交的导航准备');}});
 window.addEventListener('hashchange',()=>{if(pending&&!current(pending))cancel('已离开页面，已取消后续自动开始');});
 window.addEventListener('pagehide',()=>{if(pending?.startDispatched)pending.background=true;else cancel('已离开页面，撤销尚未提交的导航准备');});
 // Expose cancellation/status for diagnostics; no programmatic start API.
 window.M20NavigationPreparation={cancel,get state(){return pending?{kind:pending.kind,phase:pending.phase,message:pending.message}:null;}};
})();
