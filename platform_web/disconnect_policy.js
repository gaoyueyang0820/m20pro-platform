'use strict';
(function(){
 let capabilities=null,capabilityKey='',loading=false,checked=0;
 const connection=()=>[S?.server_instance,S?.connection_generation,S?.mode,S?.ip,S?.online].join('|');
 function paint(){
  if(page!=='navigation')return;
  const current=capabilityKey===connection()?capabilities:null,gos=current?.gos;
  const engineChoice=$('#route-navigation-engine'),policyChoice=$('#disconnect-policy');
  const previousEngine=engineChoice?.value,previousPolicy=policyChoice?.value;
  const docking=points.some(point=>point.type===3);
  const collectors=taskCollectorOptions();
  const stations=points.map(point=>point.station_action).filter(Boolean);
  const actions=stations.some(action=>!['front','rear'].includes(action.camera)||action.pan&&action.pan!=='none'||action.iot_device_id||action.iot_device_ids?.length)
   ||collectors.some(spec=>spec.kind!=='people_count'||!['front','rear'].includes(spec.camera));
  const active=['running','cancelling'].includes(S?.nav?.status);
  const requiredEngine=docking?'native':actions?'gos':'';
  if(engineChoice){
   if(requiredEngine){
    if(!engineChoice.dataset.beforeTask){
     engineChoice.dataset.beforeTask=engineChoice.value;
     if(policyChoice)policyChoice.dataset.beforeTask=policyChoice.value;
    }
    engineChoice.value=requiredEngine;
   }else if(engineChoice.dataset.beforeTask){
    engineChoice.value=engineChoice.dataset.beforeTask;delete engineChoice.dataset.beforeTask;
    if(policyChoice){policyChoice.value=policyChoice.dataset.beforeTask||'stop';delete policyChoice.dataset.beforeTask;}
   }
   engineChoice.disabled=!!requiredEngine||active;
  }
  const engine=engineChoice?.value||'gos';
  const desktopStations=engine==='native'&&stations.length>0&&!actions;
  const nativeOptionsReady=($('#route-arrival')?.value||'continue')==='continue'&&($('#nav-mode')?.value||'1')==='1'&&
   ($('#nav-speed')?.value||'1')==='1'&&($('#nav-gait')?.value||'flat')==='flat'&&($('#nav-manner')?.value||'0')==='0';
  if(policyChoice){
   if(requiredEngine||desktopStations)policyChoice.value='stop';
   policyChoice.disabled=!!requiredEngine||active||desktopStations;
   const canConfigure=engine==='native'&&!requiredEngine&&!desktopStations&&nativeOptionsReady;
   for(const option of policyChoice.options)option.disabled=option.value!=='stop'&&!canConfigure;
  }
  const status=S?.mode==='demo'?'本地演示':docking?'含充电航点 · 自动使用回桩流程':
   engine==='native'?(S?.online===false?'等待连接机器人':'原厂导航'):
   S?.online===false?'等待连接机器人':
   gos?.available?'GOS 寻路服务已连接':gos?.message||'正在连接 GOS 寻路服务';
  text('navigation-executor-status',status);
  let help='';
  if(docking)help='回桩路线采用断联停车。';
  else if(actions)help='外接球机和物联网采集使用 GOS 机载寻路与断联停车。';
  else if(desktopStations)help='到点停稳后由电脑统计人数，电脑断联后停车。';
  else if(engine==='gos')help='GOS 断联 5 秒后请求停车。选择「原厂导航」可设置续行或返程。';
  else if(!nativeOptionsReady)help='续行或返程需要自动接续、低速敏捷平地、自主导航和正向行走。';
  else if(policyChoice?.value&&policyChoice.value!=='stop'){
   help=current?.available===true&&current.policies?.includes(policyChoice.value)?
    '断联达到 5 秒后，NOS 按所选策略执行。':
    '可先保存路线。执行前请在连接中心启用 NOS 驻机导航。';
  }
  if(engine==='native'&&collectors.length&&!actions)help=(help?help+' ':'')+'人数由电脑采集，电脑断联期间暂停。';
  text('disconnect-policy-help',help);
  if((previousEngine!==engineChoice?.value||previousPolicy!==policyChoice?.value)&&typeof refreshNavActions==='function'){
   refreshNavActions();paintGoalPlan();
  }
 }
 async function refresh(){
  if(page!=='navigation'||S?.mode!=='live'||S?.online===false)return;
  const current=connection();if(current!==capabilityKey){capabilities=null;checked=0;}
  if(loading||Date.now()-checked<30000)return;
  loading=true;capabilityKey=current;
  try{const result=await api('/api/nav/resident/capabilities',undefined,true);if(current===connection())capabilities=result;}
  catch(_){if(current===connection())capabilities={gos:{message:'GOS 寻路服务暂不可用'}};}
  finally{loading=false;checked=Date.now();paint();}
 }
 document.addEventListener('change',event=>{
  if(page!=='navigation')return;
  if(event.target.id==='route-navigation-engine'){
   const policy=$('#disconnect-policy');
   if(policy){
    if(event.target.value==='gos'){policy.dataset.nativePolicy=policy.value;policy.value='stop';}
    else if(policy.dataset.nativePolicy)policy.value=policy.dataset.nativePolicy;
   }
   if(typeof refreshNavActions==='function'){refreshNavActions();paintGoalPlan();}
  }
  if(event.target.matches('#route-navigation-engine,#disconnect-policy,#route-arrival,#nav-mode,#nav-speed,#nav-gait,#nav-manner,[data-point][data-key="type"],[data-station-key="camera"],[data-collector-enable],[data-collector-field="camera"]'))paint();
 });
 const previousRender=render;render=function(...args){previousRender.apply(this,args);paint();refresh();};
 const previousUpdate=update;update=function(...args){previousUpdate.apply(this,args);paint();refresh();};
 const presentation=navigationPresentation;
 navigationPresentation=function(nav){
  if(nav?.engine!=='resident')return presentation(nav);
  if(nav.executor==='gos'){
   const label=({station_parking:'航点停车',station_action:'到点作业',preparing:'GOS 正在规划',local_bypass:'GOS 正在绕行'})[nav.phase]
    || ({completed:'已完成',failed:'任务未完成',cancelled:'已取消',paused:'已暂停',cancelling:'等待停车确认'})[nav.status]
    || 'GOS 自主寻路';
   return {label,title:label,detail:nav.message||'正在同步 GOS 任务状态',severity:nav.status==='failed'?'error':'normal'};
  }
  const row=nav.resident_mission||{},phase=row.phase;
  const label=phase==='returning'?'沿原路返回':nav.status==='completed'?'已完成':nav.status==='failed'?'任务未完成':nav.status==='cancelled'?'已取消':nav.status==='cancelling'?'等待停车确认':nav.phase==='resident_unknown'?'链路中断 · 状态待同步':'机器人端执行';
  return {label,title:label,detail:nav.message||'正在同步驻机任务状态',severity:nav.status==='failed'?'error':'normal'};
 };
})();
