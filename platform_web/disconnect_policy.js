'use strict';
(function(){
 let capabilities=null,capabilityKey='',loading=false,checked=0;
 const connection=()=>[S?.server_instance,S?.connection_generation,S?.mode].join('|');
 function paint(){
  if(page!=='navigation')return;
  const gos=capabilityKey===connection()?capabilities?.gos:null;
  const status=S?.mode==='demo'?'本地演示':navIntent==='route'&&points.some(p=>p.type===3)?'含充电航点 · 自动使用回桩流程':
   gos?.available?'GOS 机载寻路 · 持续断联停车':gos?.message||'正在连接 GOS 寻路服务';
  text('navigation-executor-status',status);
 }
 async function refresh(){
  if(page!=='navigation'||S?.mode!=='live')return;
  const current=connection();if(current!==capabilityKey){capabilities=null;checked=0;}
  if(loading||Date.now()-checked<30000)return;
  loading=true;capabilityKey=current;
  try{const result=await api('/api/nav/resident/capabilities',undefined,true);if(current===connection())capabilities=result;}
  catch(_){if(current===connection())capabilities={gos:{message:'GOS 寻路服务暂不可用'}};}
  finally{loading=false;checked=Date.now();paint();}
 }
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
