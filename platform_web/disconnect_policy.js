'use strict';
// The default remains desktop-controlled stop-on-loss. Non-default policies
// require an advertised onboard executor; they are never silently downgraded.
(function(){
 const labels={stop:'断联停车（默认）',finish:'断联后完成任务并停车',return:'断联后立即沿原路返回',finish_return:'断联后完成任务，再沿原路返回'};
 let goalPolicy='stop',capabilities=null,capabilityKey='',loading=false,checked=0;
 const connection=()=>[S?.server_instance,S?.connection_generation,S?.mode].join('|');
 const policy=()=>$('#disconnect-policy')?.value||'stop';
 const basePage=navPage;
 navPage=function(){
  const selected=navIntent==='goal'?goalPolicy:S?.routes?.find(r=>r.id===routeId)?.options?.disconnect_policy||'stop';
  const field=`<label class="field">电脑断联后<select id="disconnect-policy">${Object.entries(labels).map(([key,label])=>`<option value="${key}" ${key===selected?'selected':''}>${label}</option>`).join('')}</select></label><p id="disconnect-policy-help" class="help" role="status">默认断联停车。独立执行需要机器人端任务服务。</p>`;
  const markup=basePage(),opening=markup.match(/<aside[^>]*class="route-editor"[^>]*>/);
  return opening?markup.replace(opening[0],opening[0]+field):markup.replace('</aside>',field+'</aside>');
 };
 const options=routeOptions;routeOptions=function(){return {...options(),disconnect_policy:policy()};};
 const key=goalKey;goalKey=function(){const result=key();return result?result+'|'+policy():result;};
 const request=api;
 api=async function(path,data,...rest){
  if(path==='/api/nav/goal'||path==='/api/nav/start'){
   const selected=path==='/api/nav/start'?data?.disconnect_policy||S?.routes?.find(r=>r.id===data?.id)?.options?.disconnect_policy||'stop':policy();
   if(!Object.hasOwn(labels,selected))throw Error('未知断联策略');
   if(selected!=='stop'&&!(capabilityKey===connection()&&capabilities?.protocol===1&&capabilities?.available===true))throw Error('机器人端尚未就绪，不能启用断联继续或返回');
   if(path==='/api/nav/goal'&&selected!=='stop'&&data?.engine!=='native')throw Error('请使用机器人端寻路并重新规划');
   data={...data,disconnect_policy:selected};
  }
  return request(path,data,...rest);
 };
 function paint(){
  const select=$('#disconnect-policy');if(page!=='navigation'||!select)return;
  const available=capabilityKey===connection()&&capabilities?.protocol===1&&capabilities?.available===true;
  for(const option of select.options)option.disabled=option.value!=='stop'&&!available;
  select.disabled=['running','cancelling'].includes(S?.nav?.status);
  const gos=capabilities?.gos;
  const engineChoice=navIntent==='goal'?$('#point-navigation-engine'):$('#route-navigation-engine');
  const gosOption=engineChoice?.querySelector('option[value="gos"]');
  if(gosOption)gosOption.disabled=!gos?.available;
  if(engineChoice?.value==='gos'){
   text('disconnect-policy-help',gos?.available?'GOS 在机内规划并绕障。电脑持续断联时停车。':gos?.message||'GOS 导航服务尚未就绪');
  }else{
   text('disconnect-policy-help',available?(policy()==='stop'?'电脑断联时请求停车。':'连续5秒未收到电脑心跳时执行所选策略。机器人本地感知、定位或控制异常仍停车；返程沿已走轨迹，不穿越未知区域。'):(capabilities?.message||'正在核对机器人端任务服务；独立执行选项暂不可用'));
  }
  const engine=$('#point-navigation-engine');if(engine)engine.disabled=policy()!=='stop';
 }
 async function refresh(){
  if(page!=='navigation')return;
  const current=connection();if(current!==capabilityKey){capabilities=null;checked=0;}
  if(loading||Date.now()-checked<30000)return;
  loading=true;capabilityKey=current;
  try{const result=await request('/api/nav/resident/capabilities',undefined,true);if(current===connection())capabilities=result;}
  catch(_){if(current===connection())capabilities={available:false,message:'机器人端尚未加载驻机任务服务'};}
  finally{loading=false;checked=Date.now();paint();}
 }
 const previousRender=render;render=function(...args){previousRender.apply(this,args);paint();refresh();};
 const previousUpdate=update;update=function(...args){previousUpdate.apply(this,args);paint();refresh();};
 document.addEventListener('change',event=>{
  if(event.target.id!=='disconnect-policy')return;
  window.M20NavigationPreparation?.cancel('断联策略已改变，请重新规划');
  if(navIntent==='goal')goalPolicy=event.target.value;
  if(event.target.value!=='stop'){
   for(const [id,value] of Object.entries({'nav-mode':'1','nav-speed':'1','nav-gait':'flat','nav-manner':'0','route-arrival':'continue'})){const field=$('#'+id);if(field)field.value=value;}
   pointNavigationEngine='native';const engine=$('#point-navigation-engine');if(engine)engine.value='native';
  }
  invalidateGoalPlan();paint();if(navIntent==='goal'&&points.length===1)refreshGoalPlan();
 });
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
